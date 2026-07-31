# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Compositor: fuses real and virtual, displays the result.

Receives everything over the bus, owns no camera:
  - RGB frames from the source (CAMERA_RGB), with a capture timestamp.
  - Depth frames from the source (CAMERA_DEPTH), same timestamp as their RGB.
  - Detections from perception (DETECTIONS), tagged with the RGB frame's
    timestamp they were computed on.
  - Robot state from the sim (ROBOT_STATE).

It pairs depth with RGB by timestamp, turns detections into measured obstacles
(distance from the paired depth at each box centre), sends those obstacles to the
sim for avoidance, renders the virtual robot over the RGB frame, draws the boxes
and distances, and displays. Compositing and display are one process so no image
is produced onto the bus here; only obstacles go back out.

To keep display fluid, the incoming frames are drained "latest wins": only the
most recent RGB and depth are kept, so a slow render never builds a backlog.
"""

from __future__ import annotations

import json
import logging
import os
import time

import cv2
import glfw
import mujoco
import numpy as np
from OpenGL import GL
from edgebot import topics
from edgebot.bus import Publisher, Subscriber


log = logging.getLogger("compositor")


# ---------------------------------------------------------------------------
# GPU composition: offscreen robot render, camera textures, one shader pass.
# ---------------------------------------------------------------------------
def configure_model_for_offscreen(model: mujoco.MjModel, scale: int,
                                  width: int, height: int) -> None:
    """Size the offscreen buffer to scale*resolution with MSAA disabled.

    Called before mjr_makeContext. MuJoCo would otherwise resolve edges against
    its black background (MSAA), reintroducing the fringe; offsamples=0 keeps the
    edges sharp so the composition shader can resolve them against the camera.
    """
    model.vis.global_.offwidth = width * scale
    model.vis.global_.offheight = height * scale
    model.vis.quality.offsamples = 0  # no MSAA: shader does the anti-aliasing


# GLSL: a full-screen triangle, no vertex buffer needed (gl_VertexID trick).
_VERT = """
#version 330 core
out vec2 uv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;                      // 0..1 across the screen
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
"""

# GLSL: composite robot over camera with per-subsample depth occlusion.
# robot_col/robot_depth are the 2x MuJoCo render; cam_col/cam_depth the D455.
# For each output pixel we look at a 2x2 block of the high-res robot render (the
# 4 subsamples). Each subsample contributes the robot colour where the robot is
# in front of the camera depth, otherwise the camera colour. Averaging the 4
# gives anti-aliased edges blended with the real scene, never with black.
_FRAG = """
#version 330 core
in vec2 uv;
out vec4 frag;

uniform sampler2D robot_col;    // 2x MuJoCo colour
uniform sampler2D robot_depth;  // 2x MuJoCo depth (0..1, non-linear)
uniform sampler2D cam_col;      // D455 colour (BGR uploaded; swizzled below)
uniform sampler2D cam_depth;    // D455 depth in metres (R32F)

uniform vec2  out_size;         // output resolution (1x)
uniform float depth_bias_m;     // robot wins if robot_z < cam_z - bias
uniform float znear;            // MuJoCo projection near
uniform float zfar;             // MuJoCo projection far

// Floor overlay ('f'): tint pixels whose measured depth matches the expected
// floor depth (computed from camera geometry) so the operator sees the ground.
uniform int   show_floor;       // 0/1
uniform float cam_height;       // camera height above the floor (m)
uniform float cam_pitch;        // downward tilt (rad)
uniform float fx_i;             // intrinsics at camera resolution
uniform float fy_i;
uniform float ppx_i;
uniform float ppy_i;
uniform vec2  cam_res;          // camera colour/depth resolution (w,h)
uniform float floor_tol;        // tolerance (m) for floor match
uniform float floor_alpha;      // overlay strength, 0..1
uniform float floor_tol_rel;    // legacy, kept for the depth criterion
uniform float floor_h_tol;      // |height above ground| gate, metres
uniform sampler2D floor_paint;  // 1.0 force floor, 0.5 force not-floor, 0 auto
// Scale check: up to three horizontal reference lines, given as image rows in
// 0..1 counted from the TOP. Negative means unused.
uniform int   subsamples;       // per axis, so subsamples^2 coverage levels
uniform float depth_far;        // depth value of the background: 1.0 or 0.0
uniform vec3  ref_rows;
uniform float ref_px;           // half thickness, in normalised rows
uniform int   has_paint;        // 0 when no mask was painted

// Convert MuJoCo's non-linear depth-buffer value to a metric eye-space depth.
float linearize(float d) {
    float z_ndc = d * 2.0 - 1.0;
    return (2.0 * znear * zfar) / (zfar + znear - z_ndc * (zfar - znear));
}

// Expected floor depth Z for the camera pixel at uv (perpendicular depth), from
// the ground-plane geometry. Returns a large value where there is no floor.
float expected_floor_z(vec2 cuv) {
    // cuv is already in image convention (v = 0 at the top row), the same one
    // floor.py uses, so no further flip here.
    float u = cuv.x * cam_res.x;
    float v = cuv.y * cam_res.y;
    float x = (u - ppx_i) / fx_i;
    float y = (v - ppy_i) / fy_i;
    float cp = cos(cam_pitch), sp = sin(cam_pitch);
    float world_dir_z = y * (-cp) + 1.0 * (-sp);
    if (world_dir_z >= 0.0) return 1e9;
    return -cam_height / world_dir_z;
}

void main() {
    // One step of the offscreen render, whatever supersampling it was made at.
    vec2 texel = 1.0 / (out_size * float(max(1, subsamples)));
    vec3 acc = vec3(0.0);
    // The camera frame is uploaded straight from numpy, whose row 0 is the TOP
    // of the image, while GL texel row 0 is the BOTTOM. Flip v when sampling it
    // so the camera comes out upright in both present() and the readback, and
    // so it matches the row convention expected_floor_z() uses.
    vec2 cuv = vec2(uv.x, 1.0 - uv.y);
    // Camera colour at this output pixel. The frame is uploaded as RAW BGR
    // bytes declared as GL_RGB: this Mesa PTL driver silently drops BGR
    // uploads (no error, no write), so the channel swap is done here instead.
    vec3 cam_rgb = texture(cam_col, cuv).bgr;
    float cam_z = texture(cam_depth, cuv).r;        // metres (0 = no data)

    int   ss  = max(1, subsamples);
    float inv = 1.0 / float(ss * ss);
    for (int dy = 0; dy < ss; ++dy) {
        for (int dx = 0; dx < ss; ++dx) {
            vec2 suv = uv + (vec2(float(dx), float(dy)) + 0.5) * texel;
            vec4 rc = texture(robot_col, suv);
            float rd = texture(robot_depth, suv).r;
            // Coverage comes from DEPTH, not from colour. MuJoCo leaves the
            // offscreen depth at 1.0 where it drew nothing, so anything nearer
            // is robot whatever its shade. Testing luminance instead punched
            // holes through everything genuinely black on the G1: the helmet,
            // the knee covers, the joints and the soles were read as background
            // and the camera showed through them.
            //
            // Colour remains the fallback for the case the depth blit failed:
            // an exactly zero depth means no depth information at all, whereas
            // 1.0 means "background", which must stay background.
            // Normalise the convention first. This driver hands back a
            // REVERSED buffer, background 0.0 and near-geometry just above it,
            // where the classic mapping puts the background at 1.0. Feeding the
            // raw value to linearize() put the robot 9 mm from the lens and
            // silently disabled occlusion.
            float rds = (depth_far > 0.5) ? rd : 1.0 - rd;
            float robot_z = linearize(rds);
            bool depth_valid = rds > 0.0 && rds < 0.9999;
            float lum = max(rc.r, max(rc.g, rc.b));
            bool drawn = depth_valid || (rds >= 1.0 && lum > 0.02);
            bool occluded = depth_valid && (cam_z > 0.0)
                            && (robot_z > cam_z + depth_bias_m);
            acc += (drawn && !occluded) ? rc.rgb : cam_rgb;
        }
    }
    vec3 col = acc * inv;

    // Floor overlay: blend red where the measured depth matches expected floor.
    if (show_floor == 1) {
        float ez = expected_floor_z(cuv);
        // Height criterion: reconstruct the point behind this pixel and ask how
        // far above the ground plane it sits. Only the vertical component of the
        // ray matters; with no camera roll the horizontal angle is parallel to
        // the floor. A single height gate is correct at every distance, because
        // it maps to a depth tolerance of floor_h_tol / |world_dir_z|, which
        // widens with distance on its own.
        float yv = (cuv.y * cam_res.y - ppy_i) / fy_i;
        float world_dir_z = -yv * cos(cam_pitch) - sin(cam_pitch);
        float height = cam_height + cam_z * world_dir_z;
        bool is_floor = cam_z > 0.0 && world_dir_z < -1e-3
                        && abs(height) < floor_h_tol;
        // Hand-painted corrections from `make calibrate`. Polished tiles reflect
        // the IR pattern away and return no depth at all, so those pixels can
        // never be decided from the sensor: the operator paints them once and
        // the result is reused here.
        if (has_paint == 1) {
            float pv = texture(floor_paint, cuv).r;
            if (pv > 0.75)       is_floor = true;    // forced floor
            else if (pv > 0.25)  is_floor = false;   // forced not floor
        }
        if (is_floor) {
            col = mix(col, vec3(1.0, 0.0, 0.0), floor_alpha);
        }
    }

    // Scale reference lines, drawn last so nothing hides them.
    if (ref_rows.x >= 0.0 && abs(cuv.y - ref_rows.x) < ref_px) {
        col = mix(col, vec3(1.0, 1.0, 0.2), 0.85);   // camera height, the horizon
    }
    if (ref_rows.y >= 0.0 && abs(cuv.y - ref_rows.y) < ref_px) {
        col = mix(col, vec3(0.2, 1.0, 0.3), 0.85);   // expected top of the robot
    }
    if (ref_rows.z >= 0.0 && abs(cuv.y - ref_rows.z) < ref_px) {
        col = mix(col, vec3(0.3, 0.7, 1.0), 0.85);   // expected ground at its feet
    }
    if (false) {
    }
    frag = vec4(col, 1.0);
}
"""


def _restore_gl_state() -> None:
    """Put the GL state back the way MuJoCo's renderer expects to find it.

    The composition pass leaves texture unit 3 active with four textures bound
    to units 0..3, plus a program, a VAO and a bound framebuffer. mjr_render
    assumes a clean-ish state, in particular that GL_TEXTURE0 is the active
    unit, and does not reset any of this itself. The first frame after context
    creation is therefore fine and every later one is not: MuJoCo renders with
    the compositor's textures still bound underneath it, producing a uniform
    single-channel image instead of the robot.

    Whoever dirties the state cleans it, so this runs at the end of every pass
    rather than in the caller's loop.
    """
    for unit in (3, 2, 1, 0):
        GL.glActiveTexture(GL.GL_TEXTURE0 + unit)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
    GL.glActiveTexture(GL.GL_TEXTURE0)
    GL.glBindVertexArray(0)
    GL.glUseProgram(0)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
    GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
    GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
    GL.glDisable(GL.GL_SCISSOR_TEST)
    GL.glDisable(GL.GL_BLEND)
    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glDepthMask(GL.GL_TRUE)
    GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)


def _compile(src: str, kind) -> int:
    sh = GL.glCreateShader(kind)
    GL.glShaderSource(sh, src)
    GL.glCompileShader(sh)
    if not GL.glGetShaderiv(sh, GL.GL_COMPILE_STATUS):
        raise RuntimeError(GL.glGetShaderInfoLog(sh).decode())
    return sh


class GLCompositor:
    """Owns the FBOs, textures, PBOs and shader for GPU compositing.

    Call order per frame: capture_robot(...) -> upload_camera(...) -> present().
    The GL context (window) and the MjrContext must already exist.
    """

    def __init__(self, mjr_context: mujoco.MjrContext,
                 width: int, height: int, scale: int = 2,
                 depth_bias_m: float = 0.025) -> None:
        self.w, self.h, self.scale = width, height, scale
        self.depth_format, self.depth_has_stencil = 0, False
        self.depth_far = 1.0     # background depth value, probed after the first blit
        self._depth_probed = False
        self._depth_blit_warned = False
        self._depth_blit_ok = False
        self.mjr = mjr_context
        self.depth_bias_m = depth_bias_m
        self.znear, self.zfar = 0.01, 50.0  # kept in sync with the model below

        self._make_program()
        self._make_robot_fbo()
        self._make_camera_textures()
        # A VAO is required by the core profile even for attribute-less draws.
        self._vao = GL.glGenVertexArrays(1)

    # -- setup ---------------------------------------------------------------
    def _make_program(self) -> None:
        self.prog = GL.glCreateProgram()
        vs = _compile(_VERT, GL.GL_VERTEX_SHADER)
        fs = _compile(_FRAG, GL.GL_FRAGMENT_SHADER)
        GL.glAttachShader(self.prog, vs)
        GL.glAttachShader(self.prog, fs)
        GL.glLinkProgram(self.prog)
        if not GL.glGetProgramiv(self.prog, GL.GL_LINK_STATUS):
            raise RuntimeError(GL.glGetProgramInfoLog(self.prog).decode())
        GL.glDeleteShader(vs)
        GL.glDeleteShader(fs)
        self._uni = {n: GL.glGetUniformLocation(self.prog, n) for n in (
            "robot_col", "robot_depth", "cam_col", "cam_depth",
            "out_size", "depth_bias_m", "znear", "zfar",
            "show_floor", "cam_height", "cam_pitch", "fx_i", "fy_i",
            "ppx_i", "ppy_i", "cam_res", "floor_tol", "floor_alpha", "floor_tol_rel",
            "floor_h_tol", "floor_paint", "has_paint",
            "ref_rows", "ref_px", "subsamples", "depth_far")}
        # Floor-overlay parameters, set via configure_floor().
        self._paint_tex = None
        self._floor = dict(show=0, height=1.5, pitch=0.122, fx=386.0, fy=386.0,
                           ppx=325.6, ppy=239.6, res=(640.0, 480.0), tol=0.15,
                           alpha=float(os.environ.get("FLOOR_ALPHA", "0.35")),
                           tol_rel=float(os.environ.get("FLOOR_TOL_REL", "0.04")),
                           has_paint=0, ref_rows=(-1.0, -1.0, -1.0),
                           h_tol=float(os.environ.get("FLOOR_H_TOL", "0.08")))

    def _probe_mujoco_depth(self):
        """Internal format of MuJoCo's offscreen depth buffer, and whether it
        carries stencil. Returns a safe default if the query is unavailable."""
        WITH_STENCIL = {0x88F0, 0x8CAD}          # DEPTH24_STENCIL8, DEPTH32F_STENCIL8
        try:
            GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, int(self.mjr.offFBO))
            fmt = None
            for att in (GL.GL_DEPTH_STENCIL_ATTACHMENT, GL.GL_DEPTH_ATTACHMENT):
                kind = GL.glGetFramebufferAttachmentParameteriv(
                    GL.GL_READ_FRAMEBUFFER, att,
                    GL.GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE)
                if int(kind) == int(GL.GL_NONE):
                    continue
                name = int(GL.glGetFramebufferAttachmentParameteriv(
                    GL.GL_READ_FRAMEBUFFER, att,
                    GL.GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME))
                if int(kind) == int(GL.GL_RENDERBUFFER):
                    GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, name)
                    fmt = int(GL.glGetRenderbufferParameteriv(
                        GL.GL_RENDERBUFFER, GL.GL_RENDERBUFFER_INTERNAL_FORMAT))
                    GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, 0)
                else:
                    GL.glBindTexture(GL.GL_TEXTURE_2D, name)
                    fmt = int(GL.glGetTexLevelParameteriv(
                        GL.GL_TEXTURE_2D, 0, GL.GL_TEXTURE_INTERNAL_FORMAT))
                    GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                break
            GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, 0)
            if fmt:
                log.info("MuJoCo offscreen depth format is 0x%04x%s", fmt,
                         ", packed with stencil" if fmt in WITH_STENCIL else "")
                return fmt, fmt in WITH_STENCIL
        except Exception as exc:
            log.warning("could not query MuJoCo's depth format (%s)", exc)
        log.info("falling back to DEPTH24_STENCIL8 for the robot depth texture")
        return int(GL.GL_DEPTH24_STENCIL8), True

    def _probe_depth_convention(self) -> None:
        """Read a corner of the blitted depth to learn where the background sits.

        A corner is background by construction, so its value IS the far value.
        Depending on the driver that is 1.0 (classic) or 0.0 (reversed), and
        every depth comparison downstream depends on knowing which.
        """
        try:
            GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, self.robot_fbo)
            px = GL.glReadPixels(0, 0, 1, 1, GL.GL_DEPTH_COMPONENT, GL.GL_FLOAT)
            GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, 0)
            far = float(np.asarray(px, np.float32).ravel()[0])
            self.depth_far = 1.0 if far > 0.5 else 0.0
            log.info("robot depth background reads %.4f -> %s convention", far,
                     "classic (far = 1)" if self.depth_far > 0.5
                     else "REVERSED (far = 0)")
        except Exception as exc:
            log.warning("could not probe the depth convention (%s), assuming "
                        "the classic one", exc)

    def _make_robot_fbo(self) -> None:
        """FBO with sampleable colour+depth textures at 2x, blit target."""
        sw, sh = self.w * self.scale, self.h * self.scale
        self.robot_fbo = GL.glGenFramebuffers(1)
        self.robot_col_tex = GL.glGenTextures(1)
        self.robot_depth_tex = GL.glGenTextures(1)

        GL.glBindTexture(GL.GL_TEXTURE_2D, self.robot_col_tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGB8, sw, sh, 0,
                        GL.GL_RGB, GL.GL_UNSIGNED_BYTE, None)
        _nearest()
        # Match MuJoCo's own depth format instead of assuming one. Blitting
        # depth between framebuffers is only legal when the formats are
        # IDENTICAL, and a mismatch fails with GL_INVALID_OPERATION while copying
        # nothing. Assuming DEPTH_COMPONENT24 left this texture at zero for the
        # whole life of the project; assuming DEPTH24_STENCIL8 instead merely
        # moved the error. So ask the driver what MuJoCo actually allocated.
        self.depth_format, self.depth_has_stencil = self._probe_mujoco_depth()
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.robot_depth_tex)
        GL.glTexStorage2D(GL.GL_TEXTURE_2D, 1, self.depth_format, sw, sh)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAX_LEVEL, 0)
        if self.depth_has_stencil:
            # Sample the depth part, not the stencil part.
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_DEPTH_STENCIL_TEXTURE_MODE,
                               GL.GL_DEPTH_COMPONENT)
        _nearest()

        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.robot_fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                                  GL.GL_TEXTURE_2D, self.robot_col_tex, 0)
        GL.glFramebufferTexture2D(
            GL.GL_FRAMEBUFFER,
            GL.GL_DEPTH_STENCIL_ATTACHMENT if self.depth_has_stencil
            else GL.GL_DEPTH_ATTACHMENT,
            GL.GL_TEXTURE_2D, self.robot_depth_tex, 0)
        st = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)
        if st != GL.GL_FRAMEBUFFER_COMPLETE:
            log.warning("robot FBO incomplete: 0x%x", int(st))
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def _make_camera_textures(self) -> None:
        """Colour (RGB8) and depth (R32F) textures + PBOs for async upload."""
        self.cam_col_tex = GL.glGenTextures(1)
        self.cam_depth_tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.cam_col_tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGB8, self.w, self.h, 0,
                        GL.GL_RGB, GL.GL_UNSIGNED_BYTE, None)
        _linear()
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.cam_depth_tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_R32F, self.w, self.h, 0,
                        GL.GL_RED, GL.GL_FLOAT, None)
        _linear()
        # Double-buffered PBOs so uploads don't stall on the previous frame.
        self._col_pbos = GL.glGenBuffers(2)
        self._depth_pbos = GL.glGenBuffers(2)
        for pbo, nbytes in ((self._col_pbos[0], self.w * self.h * 3),
                            (self._col_pbos[1], self.w * self.h * 3),
                            (self._depth_pbos[0], self.w * self.h * 4),
                            (self._depth_pbos[1], self.w * self.h * 4)):
            GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, pbo)
            GL.glBufferData(GL.GL_PIXEL_UNPACK_BUFFER, nbytes, None,
                            GL.GL_STREAM_DRAW)
        GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
        self._pbo_idx = 0

    # -- per-frame -----------------------------------------------------------
    def set_depth_bias(self, bias_m: float) -> None:
        self.depth_bias_m = bias_m

    def configure_floor(self, cam_height, pitch_rad, fx, fy, ppx, ppy,
                        cam_res, tol=0.15) -> None:
        """Set the camera geometry used to detect the floor for the red overlay."""
        self._floor.update(height=cam_height, pitch=pitch_rad, fx=fx, fy=fy,
                           ppx=ppx, ppy=ppy, res=cam_res, tol=tol)

    def load_floor_paint(self, path: str) -> bool:
        """Upload the mask painted during calibration, if there is one.

        Values: 255 forces the pixel to floor, 128 forces it to not-floor, 0
        leaves the geometric test to decide.
        """
        import cv2 as _cv
        img = _cv.imread(path, _cv.IMREAD_GRAYSCALE) if os.path.exists(path) else None
        if img is None:
            self._floor["has_paint"] = 0
            return False
        if img.shape[:2] != (self.h, self.w):
            img = _cv.resize(img, (self.w, self.h), interpolation=_cv.INTER_NEAREST)
        if self._paint_tex is None:
            self._paint_tex = GL.glGenTextures(1)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._paint_tex)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_R8, self.w, self.h, 0,
                        GL.GL_RED, GL.GL_UNSIGNED_BYTE,
                        np.ascontiguousarray(img))
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
        self._floor["has_paint"] = 1
        forced_on = int((img > 200).sum())
        forced_off = int(((img > 80) & (img <= 200)).sum())
        log.info("floor paint loaded from %s: %d px forced floor, %d px forced clear",
                 path, forced_on, forced_off)
        return True

    def set_scale_refs(self, rows) -> None:
        """Three image rows (0..1 from the top) to mark, or negatives to hide.

        Used to check the composition scale against something physical: a point
        at the camera's own height always projects to the same row whatever its
        distance, so that line is the horizon, and the other two say where the
        top and the base of an object of known height at a known distance must
        appear. If the rendered robot does not sit between them, the scale is
        wrong, and no amount of looking will settle it.
        """
        self._floor["ref_rows"] = tuple(rows)

    def set_show_floor(self, on: bool) -> None:
        self._floor["show"] = 1 if on else 0

    def capture_robot(self, viewport_full) -> None:
        """Blit MuJoCo's offscreen buffer into our sampleable robot FBO.

        MuJoCo has just rendered the robot into its offscreen FBO. We copy colour
        and depth into robot_fbo, whose textures the shader samples. Colour and
        depth are blitted SEPARATELY: a depth-format mismatch then only affects
        the depth copy, not the colour, and never aborts the whole operation.
        """
        sw, sh = self.w * self.scale, self.h * self.scale
        src = int(self.mjr.offFBO)
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, src)
        GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, self.robot_fbo)
        # Colour first (this must succeed).
        GL.glBlitFramebuffer(0, 0, sw, sh, 0, 0, sw, sh,
                             GL.GL_COLOR_BUFFER_BIT, GL.GL_NEAREST)
        # Depth and stencil together: the source is packed, so they copy as one.
        # Report a failure once rather than swallowing it, because a silent
        # failure degrades the cut-out to a luminance test and shows up as holes
        # in everything dark on the robot.
        bits = GL.GL_DEPTH_BUFFER_BIT
        if self.depth_has_stencil:
            bits |= GL.GL_STENCIL_BUFFER_BIT
        GL.glGetError()
        try:
            GL.glBlitFramebuffer(0, 0, sw, sh, 0, 0, sw, sh, bits, GL.GL_NEAREST)
            err = GL.glGetError()
        except GL.error.GLError as exc:
            err = int(getattr(exc, "err", 1))
        if err and not self._depth_blit_warned:
            self._depth_blit_warned = True
            log.warning("robot depth blit failed (GL error 0x%x, format 0x%04x): "
                        "the silhouette falls back to a luminance test, so dark "
                        "parts of the robot may show holes and occlusion by the "
                        "real scene is disabled", err, self.depth_format)
        elif not err and not self._depth_blit_ok:
            self._depth_blit_ok = True
            log.info("robot depth blit OK: the silhouette is cut out by depth")
        if not err and not self._depth_probed:
            self._depth_probed = True
            self._probe_depth_convention()
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, 0)
        GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, 0)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def upload_camera(self, colour_bgr: np.ndarray, depth_m: np.ndarray) -> None:
        """Upload the D455 colour (BGR uint8) and depth (float metres).

        Uses glTexSubImage2D straight from the numpy arrays. The textures are
        pre-allocated once, so no per-frame GPU allocation happens here.
        """
        colour_bgr = np.ascontiguousarray(colour_bgr)
        # A BGR upload is silently ignored by this driver: the call returns no
        # error and the texture keeps its previous contents. Upload the same
        # bytes as GL_RGB (a plain memcpy for the driver) and swap the channels
        # in the fragment shader instead. Explicit unit + tight alignment so
        # nothing inherited from MuJoCo's renderer can shift the rows.
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glPixelStorei(GL.GL_UNPACK_ROW_LENGTH, 0)
        GL.glPixelStorei(GL.GL_UNPACK_SKIP_ROWS, 0)
        GL.glPixelStorei(GL.GL_UNPACK_SKIP_PIXELS, 0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.cam_col_tex)
        GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, self.w, self.h,
                           GL.GL_RGB, GL.GL_UNSIGNED_BYTE, colour_bgr)
        depth_f = np.ascontiguousarray(depth_m, dtype=np.float32)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.cam_depth_tex)
        GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, self.w, self.h,
                           GL.GL_RED, GL.GL_FLOAT, depth_f)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    def present_image(self, bgr: np.ndarray, win_w: int, win_h: int) -> None:
        """Blit a plain BGR image to the window (used in calibration mode).

        Uploads the numpy frame to the camera colour texture and draws it with a
        pass-through of the composition shader (robot texture empty), so the same
        window shows CPU-composed calibration overlays.
        """
        # Upload as the camera colour, zero depth so nothing is occluded, and an
        # empty robot (depth cleared to 1 = not drawn) -> shader shows camera only.
        h, w = bgr.shape[:2]
        if (w, h) != (self.w, self.h):
            import cv2
            bgr = cv2.resize(bgr, (self.w, self.h))
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.cam_col_tex)
        GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, self.w, self.h,
                           GL.GL_RGB, GL.GL_UNSIGNED_BYTE,
                           np.ascontiguousarray(bgr))
        # Clear the robot depth texture to "far" so the shader draws camera only.
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.robot_fbo)
        GL.glClearDepth(1.0)
        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        # Zero camera depth so cam_z<=0 -> but that shows robot; instead we rely on
        # robot depth==1 (not drawn) so acc gets cam_rgb. Present as usual.
        self.present(win_w, win_h)

    def _ensure_readback_fbo(self) -> None:
        """Create the offscreen FBO the composite is rendered into for readback."""
        if getattr(self, "_read_fbo", None) is not None:
            return
        self._read_fbo = GL.glGenFramebuffers(1)
        self._read_tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._read_tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGB8, self.w, self.h, 0,
                        GL.GL_RGB, GL.GL_UNSIGNED_BYTE, None)
        _nearest()
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._read_fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                                  GL.GL_TEXTURE_2D, self._read_tex, 0)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    def composite_to_array(self) -> np.ndarray:
        """Run the composition shader into an offscreen FBO and return the result.

        This is the proven-working path (matches the standalone unit test): render
        the composite to an FBO, read it back as a BGR uint8 image. The caller then
        shows it however it likes (e.g. cv2.imshow), sidestepping any on-screen GL
        context quirks. One readback per frame, negligible at this resolution.
        """
        import numpy as _np
        self._ensure_readback_fbo()
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._read_fbo)
        GL.glViewport(0, 0, self.w, self.h)
        GL.glClearColor(0.0, 0.0, 0.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_BLEND)
        GL.glDisable(GL.GL_SCISSOR_TEST)
        GL.glUseProgram(self.prog)
        GL.glBindVertexArray(self._vao)
        for unit, tex, name in ((0, self.robot_col_tex, "robot_col"),
                                (1, self.robot_depth_tex, "robot_depth"),
                                (2, self.cam_col_tex, "cam_col"),
                                (3, self.cam_depth_tex, "cam_depth")):
            GL.glActiveTexture(GL.GL_TEXTURE0 + unit)
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glUniform1i(self._uni[name], unit)
        GL.glUniform2f(self._uni["out_size"], float(self.w), float(self.h))
        GL.glUniform1f(self._uni["depth_bias_m"], self.depth_bias_m)
        GL.glUniform1f(self._uni["znear"], self.znear)
        GL.glUniform1f(self._uni["zfar"], self.zfar)
        f = self._floor
        GL.glUniform1i(self._uni["show_floor"], int(f["show"]))
        GL.glUniform1f(self._uni["cam_height"], float(f["height"]))
        GL.glUniform1f(self._uni["cam_pitch"], float(f["pitch"]))
        GL.glUniform1f(self._uni["fx_i"], float(f["fx"]))
        GL.glUniform1f(self._uni["fy_i"], float(f["fy"]))
        GL.glUniform1f(self._uni["ppx_i"], float(f["ppx"]))
        GL.glUniform1f(self._uni["ppy_i"], float(f["ppy"]))
        GL.glUniform2f(self._uni["cam_res"], float(f["res"][0]), float(f["res"][1]))
        GL.glUniform1f(self._uni["floor_tol"], float(f["tol"]))
        GL.glUniform1f(self._uni["floor_alpha"], float(f["alpha"]))
        GL.glUniform1f(self._uni["floor_tol_rel"], float(f["tol_rel"]))
        GL.glUniform1f(self._uni["floor_h_tol"], float(f["h_tol"]))
        GL.glUniform3f(self._uni["ref_rows"], *[float(v) for v in f["ref_rows"]])
        GL.glUniform1f(self._uni["ref_px"], 1.5 / float(self.h))
        GL.glUniform1i(self._uni["subsamples"], int(self.scale))
        GL.glUniform1f(self._uni["depth_far"], float(self.depth_far))
        GL.glUniform1i(self._uni["has_paint"], int(f.get("has_paint", 0)))
        if f.get("has_paint") and self._paint_tex is not None:
            GL.glActiveTexture(GL.GL_TEXTURE4)
            GL.glBindTexture(GL.GL_TEXTURE_2D, self._paint_tex)
            GL.glUniform1i(self._uni["floor_paint"], 4)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
        # Read back the composited RGB and return BGR (for cv2), flipped to image
        # orientation (GL origin is bottom-left).
        buf = GL.glReadPixels(0, 0, self.w, self.h, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
        img = _np.frombuffer(buf, _np.uint8).reshape(self.h, self.w, 3)
        img = _np.flipud(img)[:, :, ::-1]  # RGB->BGR, flip vertically
        _restore_gl_state()
        return _np.ascontiguousarray(img)

    def present(self, win_w: int, win_h: int) -> None:
        """Run the composition shader to the default framebuffer (the window)."""
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
        GL.glViewport(0, 0, win_w, win_h)
        # Clear to a distinct colour first. If the window shows THIS colour, the
        # shader draw below isn't covering the screen (shader/geometry issue). If
        # the window shows the composited image, all good. Set EDGEBOT_GL_DEBUG=1
        # to use a bright debug clear; otherwise clear to black.
        if os.environ.get("EDGEBOT_GL_DEBUG") == "1":
            GL.glClearColor(0.6, 0.0, 0.0, 1.0)
        else:
            GL.glClearColor(0.0, 0.0, 0.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        # MuJoCo's renderer leaves face culling and depth test enabled. Our
        # full-screen triangle must not be culled or depth-tested away, so reset
        # that state explicitly before drawing it.
        GL.glDisable(GL.GL_CULL_FACE)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_BLEND)
        GL.glDisable(GL.GL_SCISSOR_TEST)
        GL.glUseProgram(self.prog)
        GL.glBindVertexArray(self._vao)
        # Bind the four textures to units 0..3.
        for unit, tex, name in ((0, self.robot_col_tex, "robot_col"),
                                (1, self.robot_depth_tex, "robot_depth"),
                                (2, self.cam_col_tex, "cam_col"),
                                (3, self.cam_depth_tex, "cam_depth")):
            GL.glActiveTexture(GL.GL_TEXTURE0 + unit)
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glUniform1i(self._uni[name], unit)
        GL.glUniform2f(self._uni["out_size"], float(self.w), float(self.h))
        GL.glUniform1f(self._uni["depth_bias_m"], self.depth_bias_m)
        GL.glUniform1f(self._uni["znear"], self.znear)
        GL.glUniform1f(self._uni["zfar"], self.zfar)
        f = self._floor
        GL.glUniform1i(self._uni["show_floor"], int(f["show"]))
        GL.glUniform1f(self._uni["cam_height"], float(f["height"]))
        GL.glUniform1f(self._uni["cam_pitch"], float(f["pitch"]))
        GL.glUniform1f(self._uni["fx_i"], float(f["fx"]))
        GL.glUniform1f(self._uni["fy_i"], float(f["fy"]))
        GL.glUniform1f(self._uni["ppx_i"], float(f["ppx"]))
        GL.glUniform1f(self._uni["ppy_i"], float(f["ppy"]))
        GL.glUniform2f(self._uni["cam_res"], float(f["res"][0]), float(f["res"][1]))
        GL.glUniform1f(self._uni["floor_tol"], float(f["tol"]))
        GL.glUniform1f(self._uni["floor_alpha"], float(f["alpha"]))
        GL.glUniform1f(self._uni["floor_tol_rel"], float(f["tol_rel"]))
        GL.glUniform1f(self._uni["floor_h_tol"], float(f["h_tol"]))
        GL.glUniform3f(self._uni["ref_rows"], *[float(v) for v in f["ref_rows"]])
        GL.glUniform1f(self._uni["ref_px"], 1.5 / float(self.h))
        GL.glUniform1i(self._uni["subsamples"], int(self.scale))
        GL.glUniform1f(self._uni["depth_far"], float(self.depth_far))
        GL.glUniform1i(self._uni["has_paint"], int(f.get("has_paint", 0)))
        if f.get("has_paint") and self._paint_tex is not None:
            GL.glActiveTexture(GL.GL_TEXTURE4)
            GL.glBindTexture(GL.GL_TEXTURE_2D, self._paint_tex)
            GL.glUniform1i(self._uni["floor_paint"], 4)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
        _restore_gl_state()


def _nearest() -> None:
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)


def _linear() -> None:
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
    GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)


# ---------------------------------------------------------------------------
# Floor detection from depth.
# ---------------------------------------------------------------------------
class FloorDetector:
    """Builds a floor mask by comparing measured depth to expected floor depth."""

    def __init__(self, cam_height: float, pitch_deg: float,
                 fx: float, fy: float, ppx: float, ppy: float,
                 tolerance_m: float = 0.15, tolerance_rel: float = 0.0) -> None:
        self.H = cam_height
        self.pitch = np.radians(pitch_deg)
        self.fx, self.fy, self.ppx, self.ppy = fx, fy, ppx, ppy
        self.tol = tolerance_m
        # Stereo depth error grows roughly with the square of the distance, so a
        # single absolute tolerance is far too tight far away. The effective
        # tolerance is max(tolerance_m, tolerance_rel * expected_depth).
        self.tol_rel = tolerance_rel
        self._cache_shape = None
        self._expected = None  # cached expected-floor-depth map for the depth size

    def _expected_floor(self, dw: int, dh: int) -> np.ndarray:
        """Per-pixel perpendicular depth Z if the pixel were on the floor.

        Builds the full 3D ray for each pixel in the camera frame, rotates it to
        world by the camera tilt, and finds the depth Z at which the ray reaches
        the ground plane (world height 0). This accounts for the complete pixel
        angle, both horizontal and vertical offsets from the optical axis.
        """
        if self._cache_shape == (dw, dh):
            return self._expected
        us = np.arange(dw)
        vs = np.arange(dh)
        uu, vv = np.meshgrid(us, vs)
        # Intrinsics scaled to the depth resolution.
        fx = self.fx * dw / 640.0
        fy = self.fy * dh / 480.0
        ppx = self.ppx * dw / 640.0
        ppy = self.ppy * dh / 480.0
        # Ray in the camera optical frame (x right, y down, z optical axis). A
        # point at sensor-depth Z along this ray is (x*Z, y*Z, Z) in camera frame.
        x = (uu - ppx) / fx
        y = (vv - ppy) / fy
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        # Camera axes expressed in the world frame (X right, Y forward, Z up),
        # camera at height H looking forward tilted down by pitch:
        #   x_cam = (1, 0, 0)
        #   z_cam = (0, cos p, -sin p)   (optical axis, forward and down)
        #   y_cam = z_cam x x_cam        (points down)
        # World height of a point at depth Z is H + Z * world_dir_z, where
        # world_dir_z is the vertical component of (x*x_cam + y*y_cam + z*z_cam).
        # y_cam vertical component = -cos(pitch); z_cam vertical component = -sin(pitch).
        world_dir_z = x * 0.0 + y * (-cp) + 1.0 * (-sp)
        with np.errstate(divide="ignore", invalid="ignore"):
            expected = -self.H / world_dir_z  # depth Z where world height = 0
        expected[world_dir_z >= 0] = np.inf   # ray not heading down: no floor
        expected[expected <= 0] = np.inf
        self._cache_shape = (dw, dh)
        self._expected = expected
        return expected

    def _rays(self, dw: int, dh: int):
        """Per-pixel camera-frame ray components (x right, y down, z forward)."""
        uu, vv = np.meshgrid(np.arange(dw), np.arange(dh))
        fx, fy = self.fx * dw / 640.0, self.fy * dh / 480.0
        ppx, ppy = self.ppx * dw / 640.0, self.ppy * dh / 480.0
        return (uu - ppx) / fx, (vv - ppy) / fy

    def height_map(self, depth_m: np.ndarray) -> np.ndarray:
        """Height above the ground plane, in metres, for every pixel.

        The 3D point behind a pixel is Z*(x, y, 1) in the camera frame, where Z
        is the perpendicular depth RealSense reports. Only the vertical
        component of the ray matters for height: with no camera roll the
        horizontal angle x is parallel to the floor and contributes nothing.

        Working in height rather than in depth is what makes a single uniform
        threshold correct at every distance. A height tolerance maps to a depth
        tolerance of tol / |world_dir_z|, which widens with distance on its own,
        exactly as stereo error does, with no tuning parameter.
        """
        dh, dw = depth_m.shape
        _, y = self._rays(dw, dh)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        world_dir_z = y * (-cp) - sp
        return self.H + depth_m * world_dir_z

    def height_mask(self, depth_m: np.ndarray, tol_h: float = 0.08) -> np.ndarray:
        """Floor mask from the height criterion."""
        return (depth_m > 0) & (np.abs(self.height_map(depth_m)) < tol_h)

    def refine(self, mask: np.ndarray, depth_m: np.ndarray,
               close_px: int = 9, min_area_frac: float = 0.02) -> np.ndarray:
        """Close the holes a depth sensor leaves in an otherwise flat floor.

        Even with the SDK filters, specular highlights on polished tiles return
        no depth at all, so the raw mask is a floor riddled with gaps. Those gaps
        are not obstacles: an obstacle produces a *shorter* measured depth, not a
        missing one, and it is compact and connected. A hole with no depth,
        below the horizon, entirely surrounded by floor, is floor.

        So: morphological closing to bridge the speckle, then fill the enclosed
        holes, then keep only components large enough to be a floor rather than
        noise. Pixels that carry a valid depth inconsistent with the ground plane
        are never added back, which is what keeps obstacles out.
        """
        import cv2
        m = mask.astype(np.uint8)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)

        # Fill enclosed holes: flood from the border on the background, whatever
        # the flood cannot reach is an interior hole.
        h, w = m.shape
        ff = m.copy()
        cv2.floodFill(ff, np.zeros((h + 2, w + 2), np.uint8), (0, 0), 1)
        m = (m | (1 - ff)).astype(np.uint8)

        # Never claim a pixel that has a valid depth contradicting the plane:
        # that is an object standing on the floor, and it must stay out.
        contradicts = (depth_m > 0) & (np.abs(self.height_map(depth_m)) >= 0.20)
        m[contradicts] = 0

        # Drop specks; keep components covering at least min_area_frac.
        num, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        out = np.zeros_like(m, dtype=bool)
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] >= min_area_frac * h * w:
                out |= lab == i
        return out

    def fit_plane(self, depth_m: np.ndarray, inlier_m: float = 0.05,
                  iters: int = 80, seed: int = 0):
        """Measure the real ground plane instead of trusting the calibration.

        RANSAC on the reconstructed point cloud, restricted to rows below the
        horizon. Returns (height_m, pitch_deg, inlier_fraction) or None when
        there are too few valid points, which is itself the answer: the sensor
        is not seeing the floor at all.
        """
        dh, dw = depth_m.shape
        x, y = self._rays(dw, dh)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        below_horizon = (y * (-cp) - sp) < -1e-3
        sel = (depth_m > 0) & below_horizon
        if int(sel.sum()) < 500:
            return None
        Z = depth_m[sel]
        pts = np.stack([x[sel] * Z, y[sel] * Z, Z], axis=1)
        if len(pts) > 40000:                      # keep RANSAC cheap
            pts = pts[np.linspace(0, len(pts) - 1, 40000).astype(int)]
        rng = np.random.default_rng(seed)
        best = (0, None, 0.0)
        for _ in range(iters):
            i = rng.choice(len(pts), 3, replace=False)
            n = np.cross(pts[i[1]] - pts[i[0]], pts[i[2]] - pts[i[0]])
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n = n / norm
            d = float(n @ pts[i[0]])
            hits = int((np.abs(pts @ n - d) < inlier_m).sum())
            if hits > best[0]:
                best = (hits, n, d)
        hits, n, d = best
        if n is None:
            return None
        # Refine on the inliers, then orient the normal upwards. World up in the
        # camera frame is (0, -cos p, -sin p), so its y component is negative.
        inl = pts[np.abs(pts @ n - d) < inlier_m]
        c = inl.mean(axis=0)
        n = np.linalg.svd(inl - c)[2][2]
        d = float(n @ c)
        if n[1] > 0:
            n, d = -n, -d
        return abs(d), float(np.degrees(np.arctan2(-n[2], -n[1]))), hits / len(pts)

    def mask(self, depth_m: np.ndarray) -> np.ndarray:
        """Boolean floor mask for a depth image already converted to metres."""
        dh, dw = depth_m.shape
        expected = self._expected_floor(dw, dh)
        valid = depth_m > 0
        # Floor where measured depth is close to the expected floor depth.
        diff = np.abs(depth_m - expected)
        with np.errstate(invalid="ignore"):
            tol = np.maximum(self.tol, self.tol_rel * expected)
        return valid & np.isfinite(expected) & (diff < tol)

    def report(self, depth_m: np.ndarray, bands: int = 4) -> list[str]:
        """Per-band comparison of measured against expected floor depth.

        Splits the image into horizontal bands from top to bottom and reports,
        for each, how much depth is valid and how far it sits from the expected
        ground plane. Mostly-invalid bands mean the sensor sees nothing (glossy
        floors are the usual cause); a consistent non-zero median difference
        across bands means the camera height or pitch is wrong.
        """
        dh, dw = depth_m.shape
        expected = self._expected_floor(dw, dh)
        heights = self.height_map(depth_m)
        out = []
        edges = np.linspace(0, dh, bands + 1).astype(int)
        for i in range(bands):
            a, b = edges[i], edges[i + 1]
            d, e = depth_m[a:b], expected[a:b]
            ok = (d > 0) & np.isfinite(e)
            if not ok.any():
                out.append(f"rows {a:4d}-{b:4d}: no valid depth")
                continue
            dm, em = float(np.median(d[ok])), float(np.median(e[ok]))
            hh = heights[a:b][d > 0]
            out.append(
                f"rows {a:4d}-{b:4d}: valid {100.0 * (d > 0).mean():5.1f}% | "
                f"measured {dm:5.2f} m | expected {em:5.2f} m | "
                f"median diff {dm - em:+6.2f} m | median height "
                f"{float(np.median(hh)):+5.2f} m")
        return out

    def feet_screen_y(self, ground_dist_m: float, window_h: int) -> float:
        """Vertical screen position (pixels) where a floor point at the given
        horizontal ground distance appears, from camera geometry alone.

        This places the robot's feet on the real floor at any distance without
        reference objects: it inverts the floor geometry to find the image row
        whose ground point is at ground_dist_m.
        """
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        # For a floor point at horizontal distance d, its perpendicular depth is
        # Z = sqrt(d^2 + H^2) * cos(angle between ray and optical axis). Rather
        # than invert analytically, scan image rows for the matching distance.
        vs = np.linspace(0.0, 1.0, 400)
        best_v, best_err = 1.0, 1e9
        for vf in vs:
            v = vf * 480.0
            y = (v - self.ppy) / self.fy
            world_dir_z = y * (-cp) + 1.0 * (-sp)
            if world_dir_z >= 0:
                continue
            Z = -self.H / world_dir_z              # perpendicular depth to floor
            # Horizontal ground distance for that floor point.
            x0 = 0.0                                # centre column
            d = np.sqrt(max(0.0, (Z * Z) * (1 + x0 * x0 + y * y) - self.H * self.H))
            err = abs(d - ground_dist_m)
            if err < best_err:
                best_err, best_v = err, vf
        return best_v * window_h



ROBOT = os.environ.get("ROBOT", "g1")
SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/booster_t1/scene.xml",
    "g1_walker": "/models/g1_walker/scene.xml",
}

# The window must have the CAMERA's aspect ratio, not a 16:9 one. MuJoCo derives
# its horizontal field of view from fovy and the viewport aspect, so a 16:9
# viewport with the D455's 63.7 deg vertical FOV renders 95.7 deg horizontally
# against a camera that sees 79.3 deg. The robot then comes out too narrow, its
# lateral position drifts off the floor, and the error grows toward the edges.
# 4:3 makes both fields agree exactly, and the 640x480 frame scales uniformly.
WINDOW_W = int(os.environ.get("WINDOW_W", "960"))
WINDOW_H = int(os.environ.get("WINDOW_H", "720"))
WINDOW_NAME = "Edge AI Robotics"
# Bump this whenever the file changes. If the log shows an older tag than the one
# you just extracted, the container is running a stale image.
# Frames at which the full GPU diagnostic block runs. Frame 0 happens before the
# first cv2.imshow/waitKey, so comparing frame 0 with frame 1 isolates whatever
# the HighGUI window does to the GL state between iterations.
DIAG_FRAMES = {int(f) for f in os.environ.get(
    "DIAG_FRAMES", "30,120,300,600,900,1200").split(",")}
# Run headless: no cv2 window at all. The unit test passes without one, so this
# tells us directly whether cv2 HighGUI is what breaks the GPU writes.
# Display path. "glfw" presents the composite straight to the GL window that
# already owns the context, which is what gpu_compositor.py was designed for:
# no readback, no second toolkit, one context. "cv2" is the previous HighGUI
# path, kept for comparison. "none" is headless.
DISPLAY_MODE = "glfw"   # present in the render context, no readback
NO_CV2 = DISPLAY_MODE != "cv2"

# COCO class id -> readable label, for the classes the detector reports.
COCO_NAMES = {
    0: "person", 24: "backpack", 26: "handbag", 28: "suitcase",
    39: "bottle", 41: "cup", 56: "chair", 57: "couch", 59: "bed",
    60: "table", 63: "laptop", 73: "book",
}

CAM_DISTANCE = float(os.environ.get("CAM_DISTANCE", "1.0"))
# The robot starts this many metres ahead of the camera and walks away.

CAM_ELEVATION = float(os.environ.get("CAM_ELEVATION", "-3.0"))
# Azimuth 0: virtual camera behind the robot looking along +x (its forward
# direction), so the robot is seen from behind, walking away from the camera.
CAM_AZIMUTH = float(os.environ.get("CAM_AZIMUTH", "0.0"))
LOOK_AT_DROP = float(os.environ.get("LOOK_AT_DROP", "0.5"))
CAM_SMOOTHING = 0.08
HFOV_DEG = float(os.environ.get("HFOV_DEG", "89.7"))
WAITKEY_MS = int(os.environ.get("WAITKEY_MS", "1"))
# Max age difference to accept a depth frame as matching an RGB timestamp.
PAIR_TOLERANCE_S = float(os.environ.get("PAIR_TOLERANCE_S", "0.2"))


def deproject_direct(u_px: int, v_px: int, z_m: float,
                     fx: float, fy: float, ppx: float, ppy: float) -> float:
    """Convert a depth pixel + its Z value to the true radial (direct) distance.

    The depth sensor reports Z (perpendicular to the optical axis). Deprojecting
    with the intrinsics recovers the real 3D point, whose norm is the direct
    line-of-sight distance from the lens. Ignoring this underestimates distance
    for points far from the image centre (e.g. objects low in the frame).
    """
    x = (u_px - ppx) * z_m / fx
    y = (v_px - ppy) * z_m / fy
    return float((x * x + y * y + z_m * z_m) ** 0.5)


def _glfw_should_quit(window) -> bool:
    """True if the window was closed or q/Esc pressed."""
    if glfw.window_should_close(window):
        return True
    return (glfw.get_key(window, glfw.KEY_Q) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_ESCAPE) == glfw.PRESS)


def load_calibration() -> dict | None:
    path = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            calib = json.load(fh)
        log.info("calibration: vfov=%.1f pitch=%.1f height=%.2f",
                 calib.get("vfov_deg", 0), calib.get("pitch_deg", 0),
                 calib.get("camera_height_m", 0))
        return calib
    except (OSError, ValueError):
        return None


def scale_reference_rows(cam_h, pitch_deg, fy, ppy, dist_m, obj_h,
                         img_rows=480.0):
    """Image rows, 0..1 from the top, of three heights at a known distance.

    Returns (horizon, top of the object, its base). A point at the camera's own
    height projects to the same row whatever its distance, which is the horizon;
    the other two follow from the pinhole projection. Comparing the rendered
    robot against these settles the composition scale with a measurement instead
    of an impression.
    """
    p = np.radians(abs(pitch_deg))
    cp, sp = np.cos(p), np.sin(p)

    def row(h):
        zc = dist_m * cp - (h - cam_h) * sp
        if zc <= 1e-6:
            return -1.0
        yc = -dist_m * sp - (h - cam_h) * cp
        return float((ppy + fy * yc / zc) / img_rows)

    return row(cam_h), row(obj_h), row(0.0)


def height_from_row(cam_h, pitch_deg, fy, ppy, dist_m, row, img_rows=480.0):
    """Invert the projection: what world height does this image row correspond to?

    The projection of a point at height h and distance F is
        v = ppy + fy * (-F sin p - (h - H) cos p) / (F cos p - (h - H) sin p)
    which solves to
        h = H + F (u cos p + sin p) / (u sin p - cos p),  u = (v - ppy) / fy.
    Measuring the height that way assumes nothing about the model: it reports
    what is actually on screen, in metres, ready to compare with a mark on a
    real ruler.
    """
    p = np.radians(abs(pitch_deg))
    cp, sp = np.cos(p), np.sin(p)
    u = (row * img_rows - ppy) / fy
    den = u * sp - cp
    if abs(den) < 1e-9:
        return float("nan")
    return float(cam_h + dist_m * (u * cp + sp) / den)


def _log_floor_stats(detector, depth_metres, on: bool) -> None:
    """Report what fraction of the frame the floor geometry accepts."""
    log.info("floor overlay %s", "on" if on else "off")
    if not on or depth_metres is None:
        return
    try:
        mask = detector.height_mask(
            depth_metres, tol_h=float(os.environ.get("FLOOR_H_TOL", "0.08")))
        valid = int((depth_metres > 0).sum())
        log.info("floor mask: %d px (%.1f%% of frame, %.1f%% of valid depth)",
                 int(mask.sum()), 100.0 * mask.mean(),
                 100.0 * mask.sum() / valid if valid else 0.0)
        for line in detector.report(depth_metres):
            log.info("  %s", line)
        fit = detector.fit_plane(depth_metres)
        if fit is None:
            log.warning("  plane fit: not enough floor points below the horizon")
        else:
            h, pdeg, frac = fit
            log.info("  plane fit (measured): height=%.2f m pitch=%.1f deg "
                     "(%.0f%% inliers) | calibration says height=%.2f m pitch=%.1f deg",
                     h, pdeg, 100.0 * frac, cam_height, _cam_pitch_deg)
            if abs(h - cam_height) > 0.10 or abs(pdeg - _cam_pitch_deg) > 2.0:
                log.warning("  calibration disagrees with the measured plane; "
                            "try CAM_HEIGHT=%.2f CAM_PITCH=%.1f", h, pdeg)
    except Exception as exc:
        log.warning("floor mask failed: %s", exc)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")


    if not os.environ.get("DISPLAY"):
        raise SystemExit("compositor: no DISPLAY set; cannot open a window.")

    try:
        scene = SCENES[ROBOT]
    except KeyError:
        raise SystemExit(
            f"unknown robot {ROBOT!r}, expected one of {sorted(SCENES)}")
    if not os.path.exists(scene):
        raise SystemExit(
            f"{scene} is missing. Fetch the model first: "
            "run 'make build'")
    log.info("loading %s", scene)
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    # Hide the decor (sol, murs: tout geom du worldbody) by moving it to geom
    # group 4, which MjvOption hides by default ([1,1,1,0,0,0]). We must NOT use
    # group 2 for this: mujoco_menagerie puts every visual mesh of the G1 in
    # group 2 (class "visual"), so cutting group 2 deletes the whole robot.
    # A group is also cleaner than rgba alpha=0: a hidden geom writes neither
    # colour nor depth, so it cannot pollute the shader's occlusion test.
    hidden = 0
    for gid in range(model.ngeom):
        if model.geom_bodyid[gid] == 0:
            model.geom_group[gid] = 4
            hidden += 1
    log.info("decor hidden: %d worldbody geoms moved to group 4", hidden)

    model.vis.global_.offwidth = WINDOW_W
    model.vis.global_.offheight = WINDOW_H

    calib = load_calibration()

    # Robot feet are placed on the real floor by pure camera geometry (height +
    # pitch + intrinsics), via the floor detector's feet_screen_y(). No reference
    # objects needed; the red floor mask in calibration validates the geometry.
    if calib and calib.get("vfov_deg"):
        model.vis.global_.fovy = float(calib["vfov_deg"])
    cam_height = float(calib["camera_height_m"]) if calib and calib.get("camera_height_m") else float(os.environ.get("CAM_HEIGHT", "1.2"))
    # The real camera is tilted DOWN by this pitch. The virtual camera matches it
    # so the floors line up. Clamped to a safe range: too steep an angle flips the
    # view past vertical and the robot appears upside down.
    _raw_pitch = abs(float(calib["pitch_deg"])) if calib and calib.get("pitch_deg") else float(os.environ.get("CAM_PITCH", "12"))
    cam_pitch = float(np.clip(_raw_pitch, 0.0, 45.0))
    # Vertical FOV for the mouse-to-ground projection (from calibration).
    calib_vfov = float(calib["vfov_deg"]) if calib and calib.get("vfov_deg") else 63.7
    # Camera intrinsics for deprojecting depth pixels to true 3D points. The depth
    # sensor reports Z (perpendicular to the optical axis), not radial distance;
    # deprojection with fx/fy/ppx/ppy recovers the real (X,Y,Z), hence true
    # distances. Falls back to FOV-derived values if intrinsics are absent.
    _intr = (calib or {}).get("intrinsics", {})
    fx = float(_intr.get("fx", 386.0))
    fy = float(_intr.get("fy", 386.0))
    ppx = float(_intr.get("ppx", 320.0))
    ppy = float(_intr.get("ppy", 240.0))

    # Floor detector: builds a mask of navigable floor by comparing measured depth
    # to the expected floor depth. Shown as a red overlay during ROI calibration.
    _cam_pitch_deg = abs(float(calib["pitch_deg"])) if calib and calib.get("pitch_deg") else 7.0
    # Floor detection for the 'f' overlay is done in the GPU shader (see
    # gpu.configure_floor below), using the same camera geometry.

    # --- GLFW window + MuJoCo GL context.
    SCALE = int(os.environ.get("RENDER_SCALE", "2"))
    if not glfw.init():
        raise RuntimeError("glfw init failed")
    # Hidden GLFW window: it exists only to provide the GL context MuJoCo renders
    # into. The composited frame is read back and shown with cv2 (the display path
    # that works reliably here), so the GLFW window itself is never shown.
    # WINDOW_VISIBLE=1 maps the GLFW window. Some drivers (here: Mesa forced into
    # probing an unsupported PTL device) refuse to make a context current on an
    # unmapped X11 drawable, which silently turns every GL call into a no-op.
    show_win = True
    glfw.window_hint(glfw.VISIBLE, glfw.TRUE if show_win else glfw.FALSE)
    window = glfw.create_window(WINDOW_W, WINDOW_H, WINDOW_NAME, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("glfw window creation failed")
    glfw.make_context_current(window)
    log.info("compositor GL context ready (render scale %dx), display via cv2 on DISPLAY=%s, "
             "window visible=%s", SCALE, os.environ.get("DISPLAY"), show_win)
    log.info("GL_VENDOR=%s | GL_RENDERER=%s | GL_VERSION=%s",
             GL.glGetString(GL.GL_VENDOR).decode(),
             GL.glGetString(GL.GL_RENDERER).decode(),
             GL.glGetString(GL.GL_VERSION).decode())

    # Offscreen buffer must be sized (2x, no MSAA) BEFORE the MjrContext is made.
    configure_model_for_offscreen(model, SCALE, WINDOW_W, WINDOW_H)
    mjr = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    # MuJoCo allocates the offscreen buffer at mjr_makeContext time and clamps it
    # to what the driver allows. If it came back smaller than the 2x rect we
    # render into, mjr_render writes into a buffer smaller than the region we
    # read back, and the readback is black. Detect that and drop to scale 1.
    log.info("MjrContext offscreen: %dx%d (requested %dx%d), offFBO=%d, offSamples=%d",
             int(mjr.offWidth), int(mjr.offHeight), WINDOW_W * SCALE, WINDOW_H * SCALE,
             int(mjr.offFBO), int(mjr.offSamples))
    if int(mjr.offWidth) < WINDOW_W * SCALE or int(mjr.offHeight) < WINDOW_H * SCALE:
        log.warning("offscreen buffer smaller than the 2x render rect; "
                    "falling back to render scale 1")
        SCALE = 1
    GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, int(mjr.offFBO))
    log.info("offFBO status=0x%x (0x8cd5 = complete)",
             int(GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)))
    GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)

    cam = mujoco.MjvCamera()
    cam.distance = CAM_DISTANCE
    cam.azimuth = CAM_AZIMUTH
    cam.elevation = -cam_pitch
    opt = mujoco.MjvOption()
    # Groups 3/4/5 are already off in the MjvOption default ([1,1,1,0,0,0]), so
    # the collision meshes (group 3) and the decor we moved to group 4 above are
    # hidden, and the robot's visual meshes (group 2) stay visible. Do NOT set
    # opt.geomgroup[2] = 0 here: that hides the robot, not the floor.
    opt.geomgroup[3] = 0
    opt.geomgroup[4] = 0
    scn = mujoco.MjvScene(model, maxgeom=20000)
    seg_scn = mujoco.MjvScene(model, maxgeom=20000)

    # GPU compositor: depth-occluded, anti-aliased compositing with no readback.
    gpu = GLCompositor(mjr, WINDOW_W, WINDOW_H, SCALE,
                       depth_bias_m=float(os.environ.get("DEPTH_BIAS_M", "0.025")))
    # Keep the shader's near/far in sync with the model's projection.
    gpu.znear = float(model.vis.map.znear * model.stat.extent)
    gpu.zfar = float(model.vis.map.zfar * model.stat.extent)

    # Calibration/floor overlay need CPU pixels. Creating a mujoco.Renderer makes
    # its own GL context, which can clash with the GLFW context, so we defer it:
    # built on first use (when 'c' or 'f' is pressed), not at startup.
    # Configure the floor detector geometry on the GPU for the 'f' overlay.
    _paint_path = os.environ.get("FLOOR_PAINT", "/config/floor_mask.png")
    gpu.load_floor_paint(_paint_path)
    floor_paint_cpu = (cv2.imread(_paint_path, cv2.IMREAD_GRAYSCALE)
                       if os.path.exists(_paint_path) else None)
    gpu.configure_floor(cam_height, np.radians(_cam_pitch_deg), fx, fy, ppx, ppy,
                        (640.0, 480.0),
                        tol=float(os.environ.get("FLOOR_TOL", "0.15")))


    # cv2 window for display. The composite is rendered offscreen (fast, proven)
    # and shown here; the GLFW window stays hidden (context only).
    if not NO_CV2:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    else:
        log.info("display mode %s: cv2 HighGUI is out of the loop", DISPLAY_MODE)

    pub = Publisher()
    cam_sub = Subscriber([topics.CAMERA_RGB, topics.CAMERA_DEPTH], rcvhwm=2)
    sub = Subscriber([topics.ROBOT_STATE, topics.DETECTIONS])

    bg = None           # latest camera colour (BGR uint8); None until first frame
    bg_t = 0.0
    depth_buf = None    # (t, HxW uint16, scale) latest depth
    depth_metres = None  # latest depth in metres (float32), for the GPU compositor
    detections = []     # latest bbox+conf+label
    det_t = 0.0
    show_floor = False    # 'f' toggles the red floor overlay at any time
    # Same geometry as the shader's expected_floor_z(), used only to report how
    # many pixels the overlay should be tinting.
    floor_det = FloorDetector(cam_height, _cam_pitch_deg, fx, fy, ppx, ppy,
                              tolerance_m=float(os.environ.get("FLOOR_TOL", "0.15")),
                              tolerance_rel=float(os.environ.get("FLOOR_TOL_REL", "0.04")))
    log.info("virtual camera at the calibrated pose: (0.00, 0.00, %.2f) m, "
             "pitch %.1f deg down; world origin is the floor under the camera "
             "and the robot starts there", cam_height, _cam_pitch_deg)
    log.info("floor geometry: height=%.2f m pitch=%.1f deg fx=%.1f fy=%.1f "
             "ppx=%.1f ppy=%.1f | expected vfov from fy = %.1f deg",
             cam_height, _cam_pitch_deg, fx, fy, ppx, ppy,
             2.0 * np.degrees(np.arctan(240.0 / fy)))
    prev_keys = {"r": False, "f": False, "h": False}
    show_scale = False   # 'h' draws the horizon and the expected robot height
    # The walkable region is republished periodically: the floor the camera sees
    # is what bounds the robot, and it is cheap to keep in step with the scene.
    frames = 0
    last_log = time.perf_counter()

    frame_min_dt = 1.0 / float(os.environ.get("MAX_FPS", "60"))
    last_frame_t = 0.0

    while True:
        # Explicit frame-rate cap, independent of vsync (which may not throttle in
        # a headless/container GL context). Prevents the loop spinning the GPU/CPU
        # at hundreds of fps and slowing the whole machine.
        _dt = time.perf_counter() - last_frame_t
        if _dt < frame_min_dt:
            time.sleep(frame_min_dt - _dt)
        last_frame_t = time.perf_counter()

        # Drain camera streams (low HWM already keeps these fresh), then the
        # small-message socket. Latest wins for the frames.
        while (msg := cam_sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.CAMERA_RGB:
                arr = np.frombuffer(payload["jpeg"], dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    if img.shape[0] != WINDOW_H or img.shape[1] != WINDOW_W:
                        img = cv2.resize(img, (WINDOW_W, WINDOW_H))
                    bg = img
                    bg_t = payload.get("t", time.time())
            elif topic == topics.CAMERA_DEPTH:
                d = np.frombuffer(payload["depth"], dtype=np.uint16).reshape(
                    payload["h"], payload["w"])
                depth_buf = (payload.get("t", time.time()), d, payload.get("scale", 0.001))
                # Depth in metres, resized to the window, for the GPU compositor's
                # occlusion test (upscaled nearest to keep hard object edges).
                dm = d.astype(np.float32) * payload.get("scale", 0.001)
                if dm.shape != (WINDOW_H, WINDOW_W):
                    dm = cv2.resize(dm, (WINDOW_W, WINDOW_H), interpolation=cv2.INTER_NEAREST)
                depth_metres = dm
        while (msg := sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.DETECTIONS:
                detections = payload.get("detections", [])
                det_t = payload.get("t", time.time())
            elif topic == topics.ROBOT_STATE:
                qpos = np.asarray(payload["qpos"], dtype=np.float64)
                if qpos.shape[0] == model.nq:
                    # The world origin is the floor point directly below the
                    # camera, and the virtual camera sits at the calibrated
                    # height above it, so the sim's own coordinates are used as
                    # they are: the robot starts at the camera's feet and walks
                    # away from the viewer.
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)

        # Distance for each detection, from the depth frame paired by timestamp.
        # cx/cy are normalised; depth is at its own resolution.
        obstacles = []
        depth_ok = depth_buf is not None and abs(depth_buf[0] - det_t) <= PAIR_TOLERANCE_S
        for d in detections:
            cxn, cyn = d.get("cx", 0.5), d.get("cy", 0.5)
            rng = None
            if depth_ok:
                _, dmap, scale = depth_buf
                dh, dw = dmap.shape
                px = min(dw - 1, max(0, int(cxn * dw)))
                py = min(dh - 1, max(0, int(cyn * dh)))
                raw = int(dmap[py, px])
                if raw > 0:
                    # raw*scale is Z (perpendicular); deproject to true distance.
                    # Scale intrinsics to the depth image resolution.
                    z = raw * scale
                    rng = deproject_direct(px, py, z,
                                           fx * dw / 640.0, fy * dh / 480.0,
                                           ppx * dw / 640.0, ppy * dh / 480.0)
            bearing = (cxn - 0.5) * HFOV_DEG
            obstacles.append({
                "cx": cxn, "cy": cyn, "w": d.get("w", 0.0), "height": d.get("h", 0.0),
                "score": d.get("score", 0.0), "class_id": d.get("class_id", 0),
                "range_m": rng, "bearing_deg": bearing,
                "measured": rng is not None, "camera": 0,
            })

        obstacles.sort(key=lambda o: o["range_m"] if o["range_m"] is not None else float("inf"))
        pub.send(topics.PERCEPTION_OBSTACLES, {"obstacles": obstacles, "stamp": time.time()})

        # Aim horizontally at camera height: the lookat point is at the robot's
        # x,y but at the camera's height H (not the robot's feet). With elevation
        # Fixed camera at the real D455 pose (height H, tilted down by the real
        # pitch). The lookat points at the GROUND (z=0) a fixed distance ahead, so
        # the virtual ground plane matches the real floor and the robot, standing
        # on z=0, is always on the floor, not floating up at table height.
        # Ground distance the camera naturally looks at, from its height and pitch.
        pitch_eff = max(0.25, cam_pitch)   # avoid a division by zero at 0 deg
        p = np.radians(pitch_eff)
        ground_ahead = cam_height / np.tan(p)
        cam.lookat[:] = np.array([ground_ahead, 0.0, 0.0])
        cam.elevation = -pitch_eff
        # Put the camera at world height H by setting the orbital distance so that
        # distance * sin(pitch) = H (camera_z = lookat_z + distance*sin(p) = H).
        cam.distance = cam_height / np.sin(p)

        # Camera world position (fixed), for the real distance readout.
        az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
        fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        cam_pos = cam.lookat - cam.distance * fwd
        robot_dist = float(np.linalg.norm(data.qpos[:3] - cam_pos))

        # Render the robot into MuJoCo's offscreen buffer (only the robot; the
        # ground plane is hidden via geomgroup[2]=0), then composite it over the
        # camera in the shader (depth occlusion + anti-aliasing) and present to
        # the GLFW window. Pressing 'f' tints detected floor pixels red.
        mujoco.mjv_updateScene(model, data, opt, None, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, scn)
        sw, sh = WINDOW_W * SCALE, WINDOW_H * SCALE
        # Diagnostic mode: CAMERA_ONLY=1 shows the raw camera (bg) directly,
        # bypassing the robot render and the shader, to check the camera path in
        # isolation inside the real compositor.
        if os.environ.get("CAMERA_ONLY") == "1":
            if bg is not None:
                if frames in (10, 30):
                    log.info("CAMERA_ONLY: bg shape=%s mean=%.1f", bg.shape, float(bg.mean()))
                    cv2.imwrite(f"/data/camera_only_{frames}.png", bg)
                if DISPLAY_MODE == "cv2":
                    cv2.imshow(WINDOW_NAME, bg)
            frames += 1
            if DISPLAY_MODE == "cv2":
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
            elif frames > 65:
                break
            continue
        try:
            # cv2.imshow/waitKey pump a GTK main loop between frames, which can
            # leave another GL context current on this thread. Re-assert ours
            # before touching the GPU: cheap, and a no-op when nothing stole it.
            glfw.make_context_current(window)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, mjr)
            mujoco.mjr_render(mujoco.MjrRect(0, 0, sw, sh), scn, mjr)
            gpu.capture_robot(None)
            gpu.upload_camera(
                bg if bg is not None else np.zeros((WINDOW_H, WINDOW_W, 3), np.uint8),
                depth_metres if depth_metres is not None
                else np.zeros((WINDOW_H, WINDOW_W), np.float32))
            gpu.set_show_floor(show_floor)
            if show_scale:
                dist = float(np.hypot(data.qpos[0], data.qpos[1]))
                gpu.set_scale_refs(scale_reference_rows(
                    cam_height, _cam_pitch_deg, fy, ppy, max(0.3, dist),
                    float(os.environ.get("ROBOT_HEIGHT", "1.30"))))
            else:
                gpu.set_scale_refs((-1.0, -1.0, -1.0))
            out = gpu.composite_to_array()
        except Exception as exc:
            log.error("GPU compositing failed: %s", exc)
            raise

        if frames in DIAG_FRAMES:
            try:
                outpath = f"/data/composite_frame_{frames}.png"
                cv2.imwrite(outpath, out)
                bgmean = float(bg.mean()) if bg is not None else -1
                # Read the robot-only offscreen render and the camera texture back,
                # exactly like the unit test, to see the real inputs in the loop.
                from OpenGL import GL as _GL
                import numpy as _np
                sw2, sh2 = WINDOW_W * SCALE, WINDOW_H * SCALE
                _GL.glBindFramebuffer(_GL.GL_READ_FRAMEBUFFER, int(mjr.offFBO))
                rbuf = _GL.glReadPixels(0, 0, sw2, sh2, _GL.GL_RGB, _GL.GL_UNSIGNED_BYTE)
                rimg = _np.frombuffer(rbuf, _np.uint8).reshape(sh2, sw2, 3)
                rnz = int((rimg.sum(axis=2) > 10).sum())
                _GL.glBindFramebuffer(_GL.GL_READ_FRAMEBUFFER, 0)
                cv2.imwrite(f"/data/comp_robot_{frames}.png", _np.flipud(rimg)[:, :, ::-1])

                # Where the robot lands on screen against where the geometry says
                # it should. glReadPixels returns rows bottom-up, so flip to image
                # rows, then express both in the 480-row space the intrinsics are
                # defined in. This settles the scale with a measurement.
                rows = _np.nonzero((rimg.sum(axis=2) > 10).any(axis=1))[0]
                if rows.size:
                    top = (sh2 - 1 - int(rows.max())) * 480.0 / sh2
                    bot = (sh2 - 1 - int(rows.min())) * 480.0 / sh2
                    dist = float(_np.hypot(data.qpos[0], data.qpos[1]))
                    hz, e_top, e_bot = scale_reference_rows(
                        cam_height, _cam_pitch_deg, fy, ppy, max(0.3, dist),
                        float(os.environ.get("ROBOT_HEIGHT", "1.31")))
                    log.info("scale check at %.2f m: rendered rows %.1f..%.1f "
                             "(%.1f tall), expected %.1f..%.1f (%.1f tall) -> "
                             "%.0f%% of the expected height, top off by %+.1f rows",
                             dist, top, bot, bot - top,
                             e_top * 480.0, e_bot * 480.0,
                             (e_bot - e_top) * 480.0,
                             100.0 * (bot - top) / max(1e-6, (e_bot - e_top) * 480.0),
                             top - e_top * 480.0)
                    # The same measurement in metres, which is what a ruler in
                    # the room actually shows.
                    h_top = height_from_row(cam_height, _cam_pitch_deg, fy, ppy,
                                            max(0.3, dist), top / 480.0)
                    h_bot = height_from_row(cam_height, _cam_pitch_deg, fy, ppy,
                                            max(0.3, dist), bot / 480.0)
                    log.info("  on screen the robot spans %.3f m to %.3f m above "
                             "the floor, so it stands %.3f m tall at %.2f m",
                             h_bot, h_top, h_top - h_bot, dist)
                    if e_bot * 480.0 > 479.0 or e_top * 480.0 < 1.0:
                        log.info("  (the robot does not fit in frame at this "
                                 "distance, so the rendered height is clipped "
                                 "and the percentage is meaningless)")
                # Read the camera colour TEXTURE back from the GPU. If the robot
                # render is empty the shader must output cam_rgb, so a black
                # composite with a non-black bg means either this texture is
                # black (upload failed) or the draw never landed.
                _GL.glActiveTexture(_GL.GL_TEXTURE0)
                _GL.glBindTexture(_GL.GL_TEXTURE_2D, gpu.cam_col_tex)
                tb = _GL.glGetTexImage(_GL.GL_TEXTURE_2D, 0, _GL.GL_RGB,
                                       _GL.GL_UNSIGNED_BYTE)
                timg = _np.frombuffer(tb, _np.uint8).reshape(WINDOW_H, WINDOW_W, 3)
                log.info("frame %d: composite mean=%.1f, bg mean=%.1f, robot px=%d (maxb %d), "
                         "cam_col tex mean=%.1f (max %d), depth=%s",
                         frames, float(out.mean()), bgmean, rnz, int(rimg.max()),
                         float(timg.mean()), int(timg.max()),
                         depth_metres is not None)
                # Framing check: is the robot actually inside the frustum?
                log.info("frame %d: scn.ngeom=%d robot xyz=(%.2f %.2f %.2f) "
                         "cam lookat=(%.2f %.2f %.2f) dist=%.2f az=%.1f el=%.1f fovy=%.1f",
                         frames, int(scn.ngeom),
                         float(data.qpos[0]), float(data.qpos[1]), float(data.qpos[2]),
                         float(cam.lookat[0]), float(cam.lookat[1]), float(cam.lookat[2]),
                         float(cam.distance), float(cam.azimuth), float(cam.elevation),
                         float(model.vis.global_.fovy))
                if bg is not None:
                    cv2.imwrite(f"/data/comp_rawcam_{frames}.png", bg)
            except Exception as exc:
                log.warning("could not save diagnostic frame: %s", exc)
        if DISPLAY_MODE == "glfw":
            # Present in the same context that just rendered: bind the default
            # framebuffer, run the composition shader again straight to the
            # window, swap. No CPU copy, no second GUI toolkit.
            _fbw, _fbh = glfw.get_framebuffer_size(window)
            gpu.present(_fbw, _fbh)
            glfw.swap_buffers(window)
            glfw.poll_events()
        elif DISPLAY_MODE == "cv2":
            cv2.imshow(WINDOW_NAME, out)

        frames += 1
        now = time.perf_counter()
        if now - last_log >= 30.0:
            fps = frames / (now - last_log)
            age = time.time() - bg_t if bg_t else -1
            log.info("composited %.1f fps, bg age %.2fs, depth paired=%s",
                     fps, age, depth_ok)
            frames = 0
            last_log = now

        # Keys: q/Esc quit, r reset the robot to the camera foot, f floor overlay.
        if DISPLAY_MODE == "glfw":
            if _glfw_should_quit(window):
                break
            _r = glfw.get_key(window, glfw.KEY_R) == glfw.PRESS
            _f = glfw.get_key(window, glfw.KEY_F) == glfw.PRESS
            _h = glfw.get_key(window, glfw.KEY_H) == glfw.PRESS
            if _r and not prev_keys["r"]:
                pub.send(topics.CMD_RESET, {"stamp": time.time()})
            if _f and not prev_keys["f"]:
                show_floor = not show_floor
                _log_floor_stats(floor_det, depth_metres, show_floor)
            if _h and not prev_keys["h"]:
                show_scale = not show_scale
                log.info("scale reference lines %s", "on" if show_scale else "off")
            prev_keys["r"], prev_keys["f"], prev_keys["h"] = _r, _f, _h
        elif DISPLAY_MODE == "cv2":
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                pub.send(topics.CMD_RESET, {"stamp": time.time()})
            if key == ord("f"):
                show_floor = not show_floor
                _log_floor_stats(floor_det, depth_metres, show_floor)
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
        else:
            if frames > 65:
                log.info("headless: 65 frames done, stopping")
                break

    if DISPLAY_MODE == "cv2":
        cv2.destroyAllWindows()
    glfw.terminate()
    pub.close()
    sub.close()
    cam_sub.close()

if __name__ == "__main__":
    main()

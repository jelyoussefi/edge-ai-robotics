"""YOLO decode: letterbox round trip, multi-class NMS, bearing, depth wiring.

Stubs OpenVINO and builds a fake YOLO11 output tensor, so it runs with no
runtime and no model. Verifies the box that goes into 640x640 network space
comes back at the right pixels, that NMS collapses duplicates, and that bearing
and measured depth are attached correctly.
"""
import sys, types, numpy as np
ov = types.ModuleType("openvino"); ov.Core = object
sys.modules["openvino"] = ov
sys.path.insert(0, "services/perception")
import detector as D


class FakeFrame:
    """Minimal stand-in for rs_source.Frame."""
    def __init__(self, color, depth, scale, intr):
        self.color, self.depth, self.depth_scale, self.intrinsics = color, depth, scale, intr
    @property
    def has_depth(self): return self.depth is not None


det = D.Detector.__new__(D.Detector)
det.conf, det.nms = 0.4, 0.45
det.net_w = det.net_h = 640

FRAME_W, FRAME_H = 1280, 720
color = np.zeros((FRAME_H, FRAME_W, 3), np.uint8)

truth = (600.0, 200.0, 760.0, 620.0)  # x1,y1,x2,y2 in original pixels
blob, scale, pad_x, pad_y = det._letterbox(color)
print(f"letterbox: scale={scale:.4f} pad=({pad_x},{pad_y})")

# Project truth into network coords, as the model would emit.
nx1, ny1 = truth[0]*scale + pad_x, truth[1]*scale + pad_y
nx2, ny2 = truth[2]*scale + pad_x, truth[3]*scale + pad_y
cx, cy, w, h = (nx1+nx2)/2, (ny1+ny2)/2, nx2-nx1, ny2-ny1

# YOLO11 output [1, 4+80, 8400], one confident person + a near-duplicate.
NC = 80
raw = np.zeros((1, 4 + NC, 8400), np.float32)
raw[0, :4, 0] = [cx, cy, w, h]
raw[0, 4 + 0, 0] = 0.93                 # class 0 person
raw[0, :4, 1] = [cx+3, cy+3, w, h]
raw[0, 4 + 0, 1] = 0.88                 # duplicate, NMS drops it
raw[0, :4, 2] = [100, 340, 40, 80]
raw[0, 4 + 56, 2] = 0.80             # class 56 chair, maps to frame (160,320)-(240,480)

class FakeCompiled:
    def __call__(self, _): return {"out": raw}
det.compiled = FakeCompiled(); det.output_port = "out"

# Depth image: put the person at 2.5m, the chair at 1.0m.
depth = np.zeros((FRAME_H, FRAME_W), np.uint16)
depth[int(truth[1]):int(truth[3]), int(truth[0]):int(truth[2])] = 2500  # *0.001 = 2.5m
depth[320:480, 160:240] = 1000  # chair at 1.0m
intr = {"hfov_deg": 69.0, "vfov_deg": 42.0, "fx": 600, "fy": 600, "ppx": 640, "ppy": 360}
frame = FakeFrame(color, depth, 0.001, intr)

out = det.infer(frame)
assert len(out) == 2, f"expected person+chair, got {len(out)}"

# Nearest first: chair (1.0m) before person (2.5m).
chair, person = out[0], out[1]
assert chair.class_id == 56 and person.class_id == 0, "sort by range failed"
assert abs(chair.range_m - 1.0) < 0.05, f"chair range {chair.range_m}"
assert abs(person.range_m - 2.5) < 0.05, f"person range {person.range_m}"
assert chair.measured and person.measured
print(f"two obstacles        -> chair {chair.range_m:.2f}m, person {person.range_m:.2f}m   ok")

# Person box round trip.
got = (person.cx*FRAME_W - person.width*FRAME_W/2, person.cy*FRAME_H - person.height*FRAME_H/2,
       person.cx*FRAME_W + person.width*FRAME_W/2, person.cy*FRAME_H + person.height*FRAME_H/2)
err = max(abs(a-b) for a,b in zip(got, truth))
assert err < 1.5, f"round trip error {err}px"
print(f"box round trip       -> max error {err:.2f}px            ok")

# Bearing: person centre-x is right of frame centre, so bearing > 0.
assert person.bearing_deg > 0, f"person is right of centre, bearing={person.bearing_deg}"
# Chair centre near x=100 of 1280 -> far left -> strongly negative.
assert chair.bearing_deg < -20, f"chair far left, bearing={chair.bearing_deg}"
print(f"bearing              -> person {person.bearing_deg:+.1f}, chair {chair.bearing_deg:+.1f}   ok")

# No-depth frame: same detections, range unknown, measured False.
nd = FakeFrame(color, None, 0.0, None)
out2 = det.infer(nd)
assert all(not o.measured and o.range_m == D.UNKNOWN_RANGE for o in out2)
print(f"video frame (no depth)-> {len(out2)} obstacles, range unknown  ok")

print("\nPASS")

"""Verify the letterbox / un-letterbox round trip with a stubbed OpenVINO."""
import sys, types, numpy as np, cv2

# Stub openvino so detector.py imports without the real runtime.
ov = types.ModuleType("openvino"); ov.Core = object
sys.modules["openvino"] = ov
sys.path.insert(0, "services/perception")
import detector as D

det = D.Detector.__new__(D.Detector)
det.conf, det.nms = 0.4, 0.45
det.net_w = det.net_h = 640

# Non-square frame, the case where letterboxing actually matters.
FRAME_W, FRAME_H = 1280, 720
frame = np.zeros((FRAME_H, FRAME_W, 3), np.uint8)

# Put a synthetic person box at a known place in the ORIGINAL frame.
truth = (600.0, 200.0, 760.0, 620.0)  # x1,y1,x2,y2
blob, scale, pad_x, pad_y = det._letterbox(frame)
print(f"letterbox: scale={scale:.4f} pad=({pad_x},{pad_y}) blob={blob.shape}")

# Forward-project truth into network coords, as the model would have seen it.
nx1, ny1 = truth[0]*scale + pad_x, truth[1]*scale + pad_y
nx2, ny2 = truth[2]*scale + pad_x, truth[3]*scale + pad_y
cx, cy = (nx1+nx2)/2, (ny1+ny2)/2
w, h = nx2-nx1, ny2-ny1

# Build a fake YOLOv8 output tensor: [1, 84, 8400], one confident person.
raw = np.zeros((1, 84, 8400), np.float32)
raw[0, :4, 0] = [cx, cy, w, h]
raw[0, 4, 0] = 0.93          # class 0 = person
raw[0, :4, 1] = [cx+3, cy+3, w, h]   # near-duplicate, NMS should drop it
raw[0, 4, 1] = 0.88

class FakeCompiled:
    def __call__(self, _): return {"out": raw}
det.compiled = FakeCompiled(); det.output_port = "out"

out = det.infer(frame, vfov_deg=50.0)
assert len(out) == 1, f"NMS failed: got {len(out)} boxes, expected 1"
d = out[0]

# Recover pixel coords and compare with truth.
got = (d.cx*FRAME_W - d.width*FRAME_W/2, d.cy*FRAME_H - d.height*FRAME_H/2,
       d.cx*FRAME_W + d.width*FRAME_W/2, d.cy*FRAME_H + d.height*FRAME_H/2)
err = max(abs(a-b) for a, b in zip(got, truth))
print(f"truth={tuple(round(v,1) for v in truth)}")
print(f"got  ={tuple(round(v,1) for v in got)}   max error={err:.2f}px")
assert err < 1.5, f"round trip error too large: {err}"

# Range estimate sanity: box 420px tall in a 720px frame at 50 deg VFOV.
print(f"range_m={d.range_m:.2f}  score={d.score:.2f}")
assert 1.0 < d.range_m < 4.0, f"range estimate implausible: {d.range_m}"

# Farther person => smaller box => larger range.
far = D.Detector.estimate_range(210, 720, 50.0)
near = D.Detector.estimate_range(420, 720, 50.0)
assert far > near * 1.9, "range should roughly double when box height halves"
print(f"monotonic: 420px -> {near:.2f}m, 210px -> {far:.2f}m")
print("\nPASS")

"""Depth sampling: central patch, median, invalid-pixel rejection."""
import sys, types, numpy as np
ov = types.ModuleType("openvino"); ov.Core = object
sys.modules["openvino"] = ov
sys.path.insert(0, "services/perception")
import detector as D

samp = D.Detector._sample_depth

# A 100x100 depth image, 0.001 m/unit (1mm), box covering the middle.
depth = np.zeros((100, 100), np.uint16)
# Fill the central region the sampler will look at (box 20..80 -> patch 35..65).
depth[30:70, 30:70] = 2000  # 2000 units * 0.001 = 2.0 m
r = samp(depth, 0.001, 20, 20, 80, 80)
assert abs(r - 2.0) < 1e-6, f"expected 2.0m, got {r}"
print(f"uniform patch        -> {r:.3f} m               ok")

# Salt-and-pepper zeros must be rejected by the median, not averaged in.
noisy = depth.copy()
noisy[40:60:2, 40:60:2] = 0  # punch holes
r = samp(noisy, 0.001, 20, 20, 80, 80)
assert abs(r - 2.0) < 1e-6, f"median should ignore zeros, got {r}"
print(f"with invalid pixels  -> {r:.3f} m (median ok)    ok")

# All-zero patch -> unknown (inf).
r = samp(np.zeros((100,100), np.uint16), 0.001, 20, 20, 80, 80)
assert r == D.UNKNOWN_RANGE, f"expected inf, got {r}"
print(f"no valid depth       -> inf                    ok")

# Central patch avoids edges: a near foreground border must not pull the range in.
edged = np.full((100,100), 3000, np.uint16)  # object at 3m
edged[20:25, :] = 500   # a 0.5m sliver at the top edge of the box
edged[75:80, :] = 500   # and bottom edge
r = samp(edged, 0.001, 20, 20, 80, 80)
assert abs(r - 3.0) < 1e-6, f"edges should be excluded, got {r}"
print(f"edge foreground      -> {r:.3f} m (edges cut)    ok")

# Degenerate tiny box -> unknown, no crash.
r = samp(depth, 0.001, 50, 50, 51, 51)
assert r == D.UNKNOWN_RANGE
print(f"degenerate box       -> inf (no crash)         ok")

print("\nPASS")

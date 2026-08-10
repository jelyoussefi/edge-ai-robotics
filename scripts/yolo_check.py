#!/usr/bin/env python3
# Quick YOLO11m-seg check: what does the detector actually see in this scene?
# Grabs one frame from the RealSense D455 (or a saved image), runs the model,
# and reports every detection with its class, confidence and mask coverage.
# Draws boxes + mask outlines to /out/yolo_check.png so you can see what is
# found and what is missed (the low draped coffee table is the suspect).
#
# Usage inside the perception image (it already has ultralytics/openvino):
#   docker compose run --rm --entrypoint python3 perception /work/yolo_check.py
# or against a saved frame:
#   ... /work/yolo_check.py --image /data/frame.png
#
# If you have a raw RealSense grab instead, point --image at it.

import argparse, sys, os
import numpy as np

def log(*a): print(*a, file=sys.stderr, flush=True)

ap = argparse.ArgumentParser()
ap.add_argument("--image", default=None,
                help="path to an image; if omitted, grab one frame from the D455")
ap.add_argument("--model", default="yolo11m-seg.pt",
                help="model name or path (ultralytics will fetch if needed)")
ap.add_argument("--conf", type=float, default=0.10,
                help="confidence floor - deliberately low to see near-misses")
ap.add_argument("--out", default="/out/yolo_check.png")
args = ap.parse_args()

# ---- get a frame -----------------------------------------------------------
img = None
if args.image:
    import cv2
    img = cv2.imread(args.image)
    if img is None:
        log(f"could not read {args.image}"); sys.exit(1)
    log(f"loaded {args.image}  {img.shape[1]}x{img.shape[0]}")
else:
    try:
        import pyrealsense2 as rs
        import cv2
        pipe = rs.pipeline(); cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipe.start(cfg)
        for _ in range(15):            # let auto-exposure settle
            frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        img = np.asanyarray(color.get_data())
        pipe.stop()
        log(f"grabbed D455 frame  {img.shape[1]}x{img.shape[0]}")
    except Exception as e:
        log(f"RealSense grab failed ({e}). Pass --image instead.")
        sys.exit(1)

# ---- run the model ---------------------------------------------------------
try:
    from ultralytics import YOLO
except ImportError:
    log("ultralytics not in this image. Use the perception image, or "
        "pip install ultralytics.")
    sys.exit(1)

log(f"loading {args.model} ...")
model = YOLO(args.model)
res = model.predict(img, conf=args.conf, verbose=False)[0]
names = res.names
H, W = img.shape[:2]

# ---- report ----------------------------------------------------------------
print(f"\nframe {W}x{H}   conf floor {args.conf}")
print(f"{'class':<16}{'conf':>7}{'box (x0,y0,x1,y1)':>28}{'mask %':>9}")
print("-" * 60)

boxes = res.boxes
masks = res.masks
n = 0 if boxes is None else len(boxes)
found_classes = {}
for i in range(n):
    cls = names[int(boxes.cls[i])]
    conf = float(boxes.conf[i])
    x0, y0, x1, y1 = [int(v) for v in boxes.xyxy[i].tolist()]
    if masks is not None and masks.data is not None:
        m = masks.data[i].cpu().numpy()
        cover = 100.0 * float(m.sum()) / (m.shape[0] * m.shape[1])
    else:
        cover = float("nan")
    print(f"{cls:<16}{conf:>7.2f}{f'({x0},{y0},{x1},{y1})':>28}{cover:>8.1f}%")
    found_classes[cls] = max(found_classes.get(cls, 0), conf)
    n += 0

print("-" * 60)
print(f"{len([1 for _ in range(n)])} detections above {args.conf}")
print("classes seen:", ", ".join(f"{c}({v:.2f})" for c, v in
                                  sorted(found_classes.items(), key=lambda k: -k[1]))
      or "none")

# the questions that matter for this scene
wanted = {"couch", "chair", "diningtable", "sofa", "bed", "bench"}
seen = set(found_classes) & wanted
missed = wanted - seen
print("\nfurniture-relevant classes FOUND:", ", ".join(sorted(seen)) or "none")
print("furniture-relevant classes NOT found:", ", ".join(sorted(missed)))
print("\nNote: a low, draped coffee table often reads as 'diningtable' or not "
      "at all. If it is missing here, geometry (height) must catch it, not YOLO.")

# ---- draw ------------------------------------------------------------------
import cv2
vis = img.copy()
if masks is not None and masks.data is not None:
    for i in range(len(masks.data)):
        m = masks.data[i].cpu().numpy()
        m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0, 200, 255), 2)
if boxes is not None:
    for i in range(len(boxes)):
        cls = names[int(boxes.cls[i])]
        conf = float(boxes.conf[i])
        x0, y0, x1, y1 = [int(v) for v in boxes.xyxy[i].tolist()]
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 120, 0), 2)
        cv2.putText(vis, f"{cls} {conf:.2f}", (x0, max(0, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 120, 0), 2)

os.makedirs(os.path.dirname(args.out), exist_ok=True)
cv2.imwrite(args.out, vis)
print(f"\nannotated image written to {args.out}")
print("orange box = detection, cyan outline = segmentation mask contour")

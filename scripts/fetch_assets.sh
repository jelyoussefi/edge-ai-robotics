#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Fetches the two assets perception needs:
#
#   1. A sample pedestrian video, so the pipeline can be developed and demoed
#      before the D457 cameras are working.
#   2. A YOLOv8n model exported to OpenVINO IR.
#
# The video comes from the Intel Robotics AI Suite multicam-demo component
# (Apache-2.0). If a local checkout is available, pass its path:
#
#   ./scripts/fetch_assets.sh /path/to/edge-ai-suites
#
# Otherwise the script sparse-clones just that component.

set -euo pipefail

SUITE_PATH="${1:-}"
ASSETS="assets"
SUITE_REPO="https://github.com/open-edge-platform/edge-ai-suites.git"
COMPONENT="robotics-ai-suite/components/multicam-demo"

mkdir -p "$ASSETS/videos" "$ASSETS/models"

# ----------------------------------------------------------------------------
# 1. Sample video
# ----------------------------------------------------------------------------
if [ -f "$ASSETS/videos/How_People_Walk.mp4" ]; then
    echo "  Sample video already present, skipping."
else
    if [ -n "$SUITE_PATH" ] && [ -d "$SUITE_PATH/$COMPONENT/videos" ]; then
        echo "  Copying sample video from $SUITE_PATH ..."
        cp "$SUITE_PATH/$COMPONENT/videos/"*.mp4 "$ASSETS/videos/"
    else
        echo "  Sparse-cloning $COMPONENT ..."
        tmp=$(mktemp -d)
        git clone --filter=blob:none --no-checkout --depth 1 "$SUITE_REPO" "$tmp/suite"
        git -C "$tmp/suite" sparse-checkout init --cone
        git -C "$tmp/suite" sparse-checkout set "$COMPONENT"
        git -C "$tmp/suite" checkout
        cp "$tmp/suite/$COMPONENT/videos/"*.mp4 "$ASSETS/videos/"
        cp "$tmp/suite/$COMPONENT/LICENSES/Apache-2.0.txt" "$ASSETS/videos/LICENSE" 2>/dev/null || true
        rm -rf "$tmp"
    fi
    echo "  Video assets are Apache-2.0, from the Intel Robotics AI Suite."
fi

# ----------------------------------------------------------------------------
# 2. YOLOv8n as OpenVINO IR
# ----------------------------------------------------------------------------
if [ -f "$ASSETS/models/yolov8n/FP16/yolov8n.xml" ]; then
    echo "  YOLOv8n IR already present, skipping."
else
    echo "  Exporting YOLOv8n to OpenVINO IR (runs in a container, nothing installed on the host) ..."
    docker run --rm -v "$(pwd)/$ASSETS/models:/out" -w /tmp python:3.12-slim bash -c '
        set -e
        pip install --no-cache-dir --quiet "ultralytics==8.3.0" "openvino==2025.3.0" "numpy<2.0"
        python - <<PY
from ultralytics import YOLO
m = YOLO("yolov8n.pt")
m.export(format="openvino", half=True, imgsz=640)
PY
        mkdir -p /out/yolov8n/FP16
        cp yolov8n_openvino_model/* /out/yolov8n/FP16/
    '
fi

echo ""
echo "  Assets ready:"
find "$ASSETS" -maxdepth 3 -type f \( -name '*.mp4' -o -name '*.xml' -o -name '*.bin' \) -printf '    %p\n'

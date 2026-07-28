#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Fetches the two assets perception needs:
#
#   1. A sample pedestrian video, so the pipeline can be developed and demoed
#      before the D457 cameras are working.
#   2. A YOLO11n model exported to OpenVINO IR.
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
# 2. YOLO as OpenVINO IR (model chosen via YOLO_MODEL, default yolo11m for
#    better detection of small/distant objects; yolo11n is faster if needed).
# ----------------------------------------------------------------------------
YOLO_MODEL="${YOLO_MODEL:-yolo11m}"
if [ -f "$ASSETS/models/$YOLO_MODEL/FP16/$YOLO_MODEL.xml" ]; then
    echo "  $YOLO_MODEL IR already present, skipping."
else
    echo "  Exporting $YOLO_MODEL to OpenVINO IR (runs in a container, nothing installed on the host) ..."
    echo "  This pulls PyTorch, so expect a few GB and a few minutes on first run."
    # HTTP_PROXY and friends are forwarded without values, so they pass through
    # from the host when set and are simply absent when not.
    docker run --rm \
        -e HTTP_PROXY -e HTTPS_PROXY -e NO_PROXY \
        -e http_proxy -e https_proxy -e no_proxy \
        -e YOLO_CONFIG_DIR=/tmp/yolo \
        -e DEBIAN_FRONTEND=noninteractive \
        -e YOLO_MODEL="$YOLO_MODEL" \
        -v "$(pwd)/$ASSETS/models:/out" -w /tmp python:3.12-slim bash -c '
        set -e
        # Ultralytics depends on opencv-python, which needs libGL and libglib
        # at import time. The slim image ships neither, so cv2 fails to load
        # and the ultralytics import dies before export ever starts.
        apt-get update -qq 2>/dev/null
        apt-get install -y -qq --no-install-recommends libgl1 libglib2.0-0 >/dev/null 2>&1
        pip install --no-cache-dir --quiet "ultralytics>=8.3.0,<9" "openvino>=2025.0" "numpy<2.0"
        python - <<PY
import os
from ultralytics import YOLO
name = os.environ["YOLO_MODEL"]
m = YOLO(f"{name}.pt")
m.export(format="openvino", half=True, imgsz=640)
PY
        mkdir -p /out/$YOLO_MODEL/FP16
        cp ${YOLO_MODEL}_openvino_model/* /out/$YOLO_MODEL/FP16/
    '
fi

echo ""
echo "  Assets ready:"
find "$ASSETS" -maxdepth 3 -type f \( -name '*.mp4' -o -name '*.xml' -o -name '*.bin' \) -printf '    %p\n'

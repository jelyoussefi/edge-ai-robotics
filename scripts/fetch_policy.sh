#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Fetches the assets the RL locomotion path needs:
#
#   1. The G1 walker policy (walker.onnx + its weight sidecar + metadata).
#   2. The matching 29-DoF G1 MuJoCo model and its meshes.
#
# Both come from the LuckyRobots G1 manipulation challenge (MIT). The policy is
# trained for that exact model, so the two must come from the same source. Our
# own flat walking scene (services/sim/g1_walker_scene.xml) includes the fetched
# g1.xml and drops the challenge's tables and props.
#
# Usage:
#   ./scripts/fetch_policy.sh                 # clone and extract
#   ./scripts/fetch_policy.sh /path/to/clone  # reuse an existing clone

set -euo pipefail

SRC="${1:-}"
REPO="https://github.com/luckyrobots/g1-manipulation-challenge.git"
POLICY_DIR="policies/g1_walker"
MODEL_DIR="models/g1_walker"

mkdir -p "$POLICY_DIR" "$MODEL_DIR"

if [ -f "$POLICY_DIR/walker.onnx.data" ] && [ -f "$MODEL_DIR/g1.xml" ]; then
    echo "  G1 walker policy and model already present, skipping."
    exit 0
fi

if [ -n "$SRC" ] && [ -f "$SRC/walker.onnx" ]; then
    echo "  Using existing checkout at $SRC ..."
    CLONE="$SRC"
    CLEANUP=""
else
    echo "  Cloning the G1 walker challenge ..."
    CLONE=$(mktemp -d)
    CLEANUP="$CLONE"
    git clone --depth 1 "$REPO" "$CLONE" >/dev/null 2>&1
fi

# 1. Policy: graph, weights, and the joint-order / scale metadata.
echo "  Extracting walker policy ..."
cp "$CLONE/walker.onnx"      "$POLICY_DIR/"
cp "$CLONE/walker.onnx.data" "$POLICY_DIR/"

# Derive walker_meta.json from the challenge config, so joint order, defaults
# and action scales are taken from source rather than hand-copied.
python3 - "$CLONE/model_config.json" "$POLICY_DIR/walker_meta.json" <<'PY'
import json, sys
cfg = json.load(open(sys.argv[1]))
jn = cfg["joint_names"]
meta = {
    "joint_names": jn,
    "default_joint_pos": [cfg["default_joint_pos"][n] for n in jn],
    "action_scales": [cfg["action_scales"][n] for n in jn],
}
json.dump(meta, open(sys.argv[2], "w"), indent=2)
print(f"    walker_meta.json: {len(jn)} joints")
PY

# 2. Model: the 29-DoF G1 and its meshes.
echo "  Extracting G1 model and meshes ..."
cp "$CLONE/g1.xml" "$MODEL_DIR/"
if [ -d "$CLONE/assets" ]; then
    cp -r "$CLONE/assets" "$MODEL_DIR/assets"
fi
# Place our clean scene beside the model so its <include file="g1.xml"/> resolves.
cp services/sim/g1_walker_scene.xml "$MODEL_DIR/scene.xml"

[ -n "$CLEANUP" ] && rm -rf "$CLEANUP"

echo ""
echo "  Done. Policy in $POLICY_DIR, model in $MODEL_DIR."
echo "  Source: LuckyRobots g1-manipulation-challenge (MIT)."
echo "  Run:  make run POLICY=rl ROBOT=g1_walker"

#!/usr/bin/env bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Robot models are fetched rather than vendored. They are large, and each one
# carries its own licence which has to be reviewed before redistribution.

set -euo pipefail

DEST="models/mujoco_menagerie"
REPO="https://github.com/google-deepmind/mujoco_menagerie.git"

if [ -d "$DEST/.git" ]; then
    echo "  Updating $DEST ..."
    git -C "$DEST" pull --ff-only
else
    echo "  Cloning MuJoCo Menagerie into $DEST ..."
    git clone --depth 1 "$REPO" "$DEST"
fi

echo ""
echo "  Available humanoids:"
for robot in unitree_g1 unitree_h1 booster_t1 berkeley_humanoid pal_talos; do
    [ -d "$DEST/$robot" ] && echo "    $robot"
done
echo ""
echo "  Each model directory has its own LICENSE. Review it before publishing"
echo "  anything that includes the meshes or renders of them."

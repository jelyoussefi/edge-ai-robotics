# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""The one place that decides what resolution the camera runs at.

`STREAM_RES` picks it. Three processes have to agree about the answer and they
are started separately: `source` opens the sensor, `calibrate` opens the same
sensor to read intrinsics and paint the floor mask, and `compositor` sizes its
window and scales the intrinsics by it. If any two of them disagree the failure
is not an error -- it is a floor plane that is quietly tilted, which is the
worst kind of bug this project has had.

Resolution is a CALIBRATION-BEARING setting, not a display preference. fx, fy,
ppx and ppy are all expressed in pixels of a particular raster, and 1280x720 is
not a scaled 640x480: it is a different aspect ratio and a different sensor
crop. Changing STREAM_RES therefore invalidates config/camera_calibration.json
and config/floor_mask.png, and both must be regenerated with

    make calibrate HEIGHT=<metres>

There is no way to convert one into the other, which is why nothing here tries.
"""
from __future__ import annotations

import os

# fps is part of the mode: the D455 does not offer every rate at every
# resolution, and a mode that the SDK silently refuses would take the whole
# pipeline down at start.
STREAM_MODES: dict[str, tuple[int, int, int]] = {
    "720p": (1280, 720, 30),
    "480p": (640, 480, 30),
}

DEFAULT_STREAM_RES = "720p"


def stream_mode(name: str | None = None) -> tuple[int, int, int]:
    """(width, height, fps) for STREAM_RES, or for an explicit name.

    An unknown value is a hard error rather than a silent fall back to the
    default: a typo in compose that quietly halves the resolution would be
    invisible in the picture and visible only as a calibration that no longer
    matches, days later.
    """
    key = (name or os.environ.get("STREAM_RES", DEFAULT_STREAM_RES)).strip()
    if key not in STREAM_MODES:
        raise SystemExit(
            f"STREAM_RES={key!r} is not one of {sorted(STREAM_MODES)}. "
            f"It selects the sensor resolution and the calibration is tied to "
            f"it, so it is not something to guess at.")
    return STREAM_MODES[key]


def stream_name() -> str:
    """The mode's name, for logging and for the console's status line."""
    return os.environ.get("STREAM_RES", DEFAULT_STREAM_RES).strip()

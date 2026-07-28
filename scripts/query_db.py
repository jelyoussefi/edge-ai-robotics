#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Inspect the recorder's SQLite database: sessions, calibration, telemetry.

Usage:
  python3 scripts/query_db.py                 # summary of all sessions
  python3 scripts/query_db.py --session 3     # detail for one session
  python3 scripts/query_db.py --calibration   # calibration history
"""
import argparse
import sqlite3
import os

DB = os.environ.get("RECORDER_DB", "data/edgebot.sqlite")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default=DB)
    ap.add_argument("--session", type=int, default=None)
    ap.add_argument("--calibration", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(args.db):
        raise SystemExit(f"no database at {args.db}")
    conn = sqlite3.connect(args.db)

    if args.calibration:
        print("=== calibration history ===")
        for r in conn.execute("SELECT id, datetime(ts,'unixepoch','localtime'), "
                              "camera_height_m, hfov_deg, vfov_deg, pitch_deg FROM calibration ORDER BY ts DESC"):
            print(f"#{r[0]} {r[1]} | height={r[2]}m hfov={r[3]:.1f} vfov={r[4]:.1f} pitch={r[5]:.1f}")
        return

    if args.session is not None:
        s = conn.execute("SELECT datetime(started_ts,'unixepoch','localtime'), robot FROM sessions WHERE id=?",
                        (args.session,)).fetchone()
        if not s:
            raise SystemExit(f"no session {args.session}")
        print(f"=== session {args.session}: {s[0]}, robot {s[1]} ===")
        tel = conn.execute("SELECT COUNT(*), MIN(distance_m), MAX(distance_m) FROM robot_telemetry WHERE session_id=?",
                          (args.session,)).fetchone()
        print(f"telemetry: {tel[0]} rows, distance {tel[1]:.2f}..{tel[2]:.2f}m" if tel[0] else "no telemetry")
        obs = conn.execute("SELECT class_id, COUNT(*), AVG(range_m) FROM obstacles WHERE session_id=? GROUP BY class_id",
                          (args.session,)).fetchall()
        print("obstacles by class:")
        for o in obs:
            print(f"  class {o[0]}: {o[1]} detections, avg range {o[2]:.2f}m" if o[2] else f"  class {o[0]}: {o[1]}")
        return

    print("=== sessions ===")
    for r in conn.execute("SELECT id, datetime(started_ts,'unixepoch','localtime'), robot, "
                          "(SELECT COUNT(*) FROM robot_telemetry WHERE session_id=sessions.id) FROM sessions ORDER BY started_ts DESC"):
        print(f"#{r[0]} {r[1]} | robot={r[2]} | {r[3]} telemetry rows")
    conn.close()


if __name__ == "__main__":
    main()

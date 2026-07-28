# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Recorder: persists calibration and runtime telemetry to SQLite.

A passive bus consumer that writes to a single SQLite file (no server, ideal on
a NUC). It records:
  - the calibration in use at startup (height, FOV, pitch, camera serial),
  - a session row per run,
  - robot telemetry (position, distance to camera, fallen) sampled at a low
    rate so the database stays small,
  - detected obstacles (class, distance, bearing, confidence).

It subscribes only to small messages (ROBOT_STATE, PERCEPTION_OBSTACLES); it
never touches the camera image streams, so it adds no load to the video path.
Distance for the robot is computed by the compositor and echoed here via a
dedicated telemetry field when present; otherwise the recorder stores position.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import sqlite3
import time

from edgebot import topics
from edgebot.bus import Subscriber

log = logging.getLogger("recorder")

DB_PATH = os.environ.get("RECORDER_DB", "/data/edgebot.sqlite")
CALIB_PATH = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
# Sample telemetry at this rate. Low on purpose: at 60Hz a 10-min run is 36k
# rows; a few Hz is plenty for history and replay.
SAMPLE_HZ = float(os.environ.get("RECORD_HZ", "3"))
ROBOT = os.environ.get("ROBOT", "g1")


def init_db(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS calibration (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts REAL NOT NULL,
            camera_serial TEXT,
            camera_height_m REAL,
            hfov_deg REAL,
            vfov_deg REAL,
            pitch_deg REAL
        );
        CREATE TABLE IF NOT EXISTS sessions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            started_ts REAL NOT NULL,
            ended_ts REAL,
            robot TEXT
        );
        CREATE TABLE IF NOT EXISTS robot_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id INTEGER NOT NULL,
            ts REAL NOT NULL,
            x REAL, y REAL, z REAL,
            distance_m REAL,
            fallen INTEGER,
            FOREIGN KEY (session_id) REFERENCES sessions(id)
        );
        CREATE TABLE IF NOT EXISTS obstacles (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id INTEGER NOT NULL,
            ts REAL NOT NULL,
            class_id INTEGER,
            range_m REAL,
            bearing_deg REAL,
            score REAL,
            FOREIGN KEY (session_id) REFERENCES sessions(id)
        );
        CREATE INDEX IF NOT EXISTS idx_tel_session ON robot_telemetry(session_id, ts);
        CREATE INDEX IF NOT EXISTS idx_obs_session ON obstacles(session_id, ts);
        """
    )
    conn.commit()


def record_calibration(conn: sqlite3.Connection) -> None:
    if not os.path.exists(CALIB_PATH):
        log.info("no calibration file at %s; skipping calibration record", CALIB_PATH)
        return
    try:
        with open(CALIB_PATH) as fh:
            c = json.load(fh)
    except (OSError, ValueError):
        log.warning("could not read calibration file")
        return
    conn.execute(
        "INSERT INTO calibration (ts, camera_serial, camera_height_m, hfov_deg, vfov_deg, pitch_deg) "
        "VALUES (?, ?, ?, ?, ?, ?)",
        (time.time(), c.get("serial"), c.get("camera_height_m"),
         c.get("hfov_deg"), c.get("vfov_deg"), c.get("pitch_deg")),
    )
    conn.commit()
    log.info("recorded calibration: height=%.2f vfov=%.1f pitch=%.1f",
             c.get("camera_height_m", 0), c.get("vfov_deg", 0), c.get("pitch_deg", 0))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    init_db(conn)
    record_calibration(conn)

    cur = conn.execute("INSERT INTO sessions (started_ts, robot) VALUES (?, ?)",
                       (time.time(), ROBOT))
    session_id = cur.lastrowid
    conn.commit()
    log.info("recording to %s, session %d, sampling %.1f Hz", DB_PATH, session_id, SAMPLE_HZ)

    sub = Subscriber([topics.ROBOT_STATE, topics.PERCEPTION_OBSTACLES])
    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    interval = 1.0 / SAMPLE_HZ
    last_sample = 0.0
    last_state = None
    tel_rows = 0
    obs_rows = 0

    while running:
        msg = sub.recv(200)
        if msg is not None:
            topic, payload = msg
            if topic == topics.ROBOT_STATE:
                last_state = payload
            elif topic == topics.PERCEPTION_OBSTACLES:
                # Record obstacles at the sample rate too (piggyback on the clock).
                now = time.perf_counter()
                if now - last_sample >= interval:
                    for o in payload.get("obstacles", []):
                        conn.execute(
                            "INSERT INTO obstacles (session_id, ts, class_id, range_m, bearing_deg, score) "
                            "VALUES (?, ?, ?, ?, ?, ?)",
                            (session_id, time.time(), o.get("class_id"),
                             o.get("range_m"), o.get("bearing_deg"), o.get("score")),
                        )
                        obs_rows += 1

        # Sample robot telemetry at the fixed rate.
        now = time.perf_counter()
        if now - last_sample >= interval and last_state is not None:
            qpos = last_state.get("qpos", [])
            x, y, z = (qpos[0], qpos[1], qpos[2]) if len(qpos) >= 3 else (None, None, None)
            conn.execute(
                "INSERT INTO robot_telemetry (session_id, ts, x, y, z, distance_m, fallen) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (session_id, time.time(), x, y, z,
                 last_state.get("distance_m"), int(bool(last_state.get("fallen")))),
            )
            tel_rows += 1
            last_sample = now
            conn.commit()
            if tel_rows % 30 == 0:
                log.info("session %d: %d telemetry rows, %d obstacle rows",
                         session_id, tel_rows, obs_rows)

    conn.execute("UPDATE sessions SET ended_ts = ? WHERE id = ?", (time.time(), session_id))
    conn.commit()
    log.info("session %d closed: %d telemetry, %d obstacles", session_id, tel_rows, obs_rows)
    conn.close()
    sub.close()


if __name__ == "__main__":
    main()

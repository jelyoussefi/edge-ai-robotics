#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Web console: watch the demo from any browser on the LAN.

The NUC sits away from the TV, so the demo needs a second viewer that is not a
screen. This serves the compositor's OWN annotated frames -- it never renders
anything itself, because a second render path would be free to disagree with the
first, and this project has spent a lot of measurement on making one picture
mean one thing.

NO AUTHENTICATION, LAN ONLY. It binds 0.0.0.0:8080 and anyone who can reach the
port can watch the camera and toggle the overlays. That is acceptable on a
demo network and nowhere else; do not expose it.
"""
from __future__ import annotations

import asyncio
import logging
import os
import time

from aiohttp import web

from edgebot import topics
from edgebot.bus import Publisher, Subscriber

from page import PAGE

log = logging.getLogger("web")

PORT = int(os.environ.get("WEB_PORT", "8080"))
BOUNDARY = "edgebotframe"

HISTORY = 60          # seconds of sparkline, at the collector's 1 Hz
FPS_WINDOW = 30       # frames averaged for the FPS tile

STATE: dict = {"frame": None, "t": 0.0, "platform": {}, "history": {},
               "arrivals": [],
               "laps": 0, "source": "?", "mode": "?", "map_known": 0,
               "map_occupied": 0, "goal": None, "path_len": 0.0,
               "clearance": 0.0, "robot": None}


async def pump(app):
    """One bus subscriber for everything, drained off the event loop.

    recv is blocking, so it runs in a thread: an aiohttp handler that stalls on
    a socket stops serving every other client, and the whole point here is
    several browsers at once.
    """
    sub = Subscriber([topics.COMPOSITED_FRAME, topics.ROBOT_STATE,
                      topics.SUITE_MAP, topics.SUITE_PATH, topics.PATROL_ROI,
                      topics.PLATFORM])
    loop = asyncio.get_running_loop()

    def _recv():
        return sub.recv(200)

    try:
        while True:
            msg = await loop.run_in_executor(None, _recv)
            if msg is None:
                continue
            topic, payload = msg
            if topic == topics.COMPOSITED_FRAME:
                STATE["frame"] = payload["jpeg"]
                STATE["t"] = float(payload.get("t", 0.0))
                # FPS is measured HERE, from arrivals, not taken from the
                # compositor's own counter: this is the rate the console is
                # actually able to show, which is the number a viewer of this
                # page is asking about.
                a = STATE["arrivals"]
                a.append(time.monotonic())
                del a[:-FPS_WINDOW]
            elif topic == topics.SUITE_MAP:
                STATE["map_known"] = int(payload.get("known", 0))
                STATE["map_occupied"] = int(payload.get("occupied", 0))
            elif topic == topics.SUITE_PATH:
                STATE["goal"] = payload.get("goal")
                STATE["path_len"] = float(payload.get("length_m", 0.0))
                STATE["clearance"] = float(payload.get("clearance_m", -1.0))
            elif topic == topics.PLATFORM:
                STATE["platform"] = payload
                # 60 s of history at the collector's 1 Hz. Kept here rather than
                # in the browser so a page opened mid-demo shows the last minute
                # immediately instead of drawing itself in from an empty chart,
                # and so two viewers see the SAME history rather than each their
                # own window since they happened to load.
                hist = STATE["history"]
                for k in ("cpu_pct", "gpu_pct", "npu_pct", "pkg_w", "gpu_w"):
                    v = payload.get(k)
                    series = hist.setdefault(k, [])
                    series.append(None if v is None else round(float(v), 1))
                    del series[:-HISTORY]
            elif topic == topics.ROBOT_STATE:
                q = payload.get("qpos") or []
                if len(q) >= 2:
                    STATE["robot"] = [round(float(q[0]), 2),
                                      round(float(q[1]), 2)]
    except asyncio.CancelledError:
        pass
    finally:
        sub.close()


async def stream(request):
    """MJPEG. One multipart response per client, each with its own pacing."""
    resp = web.StreamResponse(status=200, headers={
        "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        "Cache-Control": "no-store, no-cache, must-revalidate",
    })
    await resp.prepare(request)
    last = 0.0
    try:
        while True:
            frame, t = STATE["frame"], STATE["t"]
            # Only send frames that are NEW to this client. Re-sending the last
            # frame at the poll rate would triple the bandwidth for two viewers
            # and add nothing; a browser holds the previous image anyway.
            if frame is not None and t != last:
                last = t
                # X-Stamp carries the compositor's capture instant, the same
                # value it burned into the pixels. Browsers ignore an unknown
                # part header; scripts/web_latency.py reads it and subtracts it
                # from arrival, which measures everything up to the browser's
                # front door without needing OCR of our own overlay.
                await resp.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"X-Stamp: " + f"{t:.6f}".encode() + b"\r\n"
                    b"Content-Length: " + str(len(frame)).encode()
                    + b"\r\n\r\n" + frame + b"\r\n")
            else:
                await asyncio.sleep(0.005)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def status(request):
    # Deliberately no frame count, frame age or encode time. Those describe the
    # console's own plumbing, not the demo: a viewer wants to know what the
    # robot is doing, and a panel that reports on itself reads as instrumented
    # rather than finished. They are still measurable -- the compositor logs the
    # encode cost and scripts/web_latency.py reads the per-part stamp.
    return web.json_response({
        "source": os.environ.get("OBSTACLE_SOURCE", "ours"),
        "nav_mode": os.environ.get("NAV_MODE", "patrol"),
        "map_known": STATE["map_known"],
        "map_occupied": STATE["map_occupied"],
        "goal": STATE["goal"],
        "path_len_m": STATE["path_len"],
        "clearance_m": STATE["clearance"],
        "stream_res": os.environ.get("STREAM_RES", "720p"),
    })


async def platform(request):
    """Engine load and power, plus 60 s of history for the sparklines.

    Separate from /status because it has a different shape and a different
    reason to exist: /status answers "what is the robot doing", this answers
    "what is the board doing". Merging them would make one poll rate serve two
    questions that do not change at the same rate.
    """
    p = dict(STATE["platform"])
    p["history"] = STATE["history"]
    a = STATE["arrivals"]
    # Two arrivals are the minimum that define a rate, and a stalled stream
    # must read as no-data rather than as the last rate it had.
    p["fps"] = (round((len(a) - 1) / (a[-1] - a[0]), 1)
                if len(a) >= 2 and a[-1] - a[0] > 0
                and time.monotonic() - a[-1] < 2.0 else None)
    return web.json_response(p)


async def cmd(request):
    action = request.match_info["action"]
    if action not in ("floor", "detections", "cloud", "map", "reset"):
        raise web.HTTPBadRequest(text=f"unknown action {action}")
    request.app["pub"].send(topics.UI_CMD,
                            {"action": action, "stamp": time.time()})
    return web.json_response({"sent": action})


async def index(request):
    return web.Response(text=PAGE, content_type="text/html")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    app = web.Application()
    app["pub"] = Publisher()
    app.router.add_get("/", index)
    app.router.add_get("/stream", stream)
    app.router.add_get("/status", status)
    app.router.add_get("/platform", platform)
    app.router.add_post("/cmd/{action}", cmd)

    async def _start(app):
        app["pump"] = asyncio.create_task(pump(app))

    async def _stop(app):
        app["pump"].cancel()
        app["pub"].close()

    app.on_startup.append(_start)
    app.on_cleanup.append(_stop)
    log.info("web console on 0.0.0.0:%d (LAN only, no auth)", PORT)
    web.run_app(app, host="0.0.0.0", port=PORT, print=None)


if __name__ == "__main__":
    main()

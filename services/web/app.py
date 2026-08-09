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

log = logging.getLogger("web")

PORT = int(os.environ.get("WEB_PORT", "8080"))
BOUNDARY = "edgebotframe"

STATE: dict = {"frame": None, "t": 0.0, "encode_ms": 0.0, "frames": 0,
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
                      topics.SUITE_MAP, topics.SUITE_PATH, topics.PATROL_ROI])
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
                STATE["encode_ms"] = float(payload.get("encode_ms", 0.0))
                STATE["frames"] += 1
            elif topic == topics.SUITE_MAP:
                STATE["map_known"] = int(payload.get("known", 0))
                STATE["map_occupied"] = int(payload.get("occupied", 0))
            elif topic == topics.SUITE_PATH:
                STATE["goal"] = payload.get("goal")
                STATE["path_len"] = float(payload.get("length_m", 0.0))
                STATE["clearance"] = float(payload.get("clearance_m", -1.0))
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
                await resp.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode()
                    + b"\r\n\r\n" + frame + b"\r\n")
            else:
                await asyncio.sleep(0.005)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def status(request):
    age = time.time() - STATE["t"] if STATE["t"] else -1.0
    return web.json_response({
        "frames": STATE["frames"],
        "frame_age_s": round(age, 2),
        "encode_ms": STATE["encode_ms"],
        "source": os.environ.get("OBSTACLE_SOURCE", "ours"),
        "nav_mode": os.environ.get("NAV_MODE", "patrol"),
        "map_known": STATE["map_known"],
        "map_occupied": STATE["map_occupied"],
        "goal": STATE["goal"],
        "path_len_m": STATE["path_len"],
        "clearance_m": STATE["clearance"],
        "robot": STATE["robot"],
    })


async def cmd(request):
    action = request.match_info["action"]
    if action not in ("floor", "detections", "cloud", "map", "reset"):
        raise web.HTTPBadRequest(text=f"unknown action {action}")
    request.app["pub"].send(topics.UI_CMD,
                            {"action": action, "stamp": time.time()})
    return web.json_response({"sent": action})


async def index(request):
    return web.Response(text=PAGE, content_type="text/html")


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Edge AI Robotics - console</title><style>
:root{--bg:#0b0e14;--panel:#141922;--line:#232b39;--fg:#e6edf7;--dim:#8b98ad;
--accent:#00c7fd}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.5 "Intel One Text",system-ui,-apple-system,Segoe UI,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:14px 20px;
background:var(--panel);border-bottom:1px solid var(--line)}
header b{font-weight:600;letter-spacing:.2px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--accent)}
main{display:flex;flex-wrap:wrap;gap:18px;padding:18px}
.view{flex:1 1 640px;min-width:320px}
img{width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
aside{flex:0 0 260px;min-width:240px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:6px;
padding:14px;margin-bottom:14px}
.card h2{margin:0 0 10px;font-size:12px;text-transform:uppercase;
letter-spacing:.08em;color:var(--dim);font-weight:600}
.row{display:flex;justify-content:space-between;gap:10px;padding:3px 0}
.row span:last-child{color:var(--accent);font-variant-numeric:tabular-nums}
button{width:100%;margin:4px 0;padding:9px 12px;background:#1b2230;
color:var(--fg);border:1px solid var(--line);border-radius:5px;cursor:pointer;
font:inherit;text-align:left}
button:hover{border-color:var(--accent);color:var(--accent)}
kbd{background:#0b0e14;border:1px solid var(--line);border-radius:3px;
padding:0 5px;color:var(--dim);float:right;font-size:12px}
.note{color:var(--dim);font-size:12px;padding:0 20px 20px}
</style></head><body>
<header><span class="dot"></span><b>Edge AI Robotics</b>
<span style="color:var(--dim)">console</span></header>
<main>
<div class="view"><img src="/stream" alt="live"></div>
<aside>
<div class="card"><h2>Status</h2><div id="s"></div></div>
<div class="card"><h2>Overlays</h2>
<button onclick="cmd('floor')">Floor<kbd>f</kbd></button>
<button onclick="cmd('detections')">Detections<kbd>s</kbd></button>
<button onclick="cmd('cloud')">Suite cloud<kbd>p</kbd></button>
<button onclick="cmd('map')">Map + path<kbd>m</kbd></button>
<button onclick="cmd('reset')">Reset robot<kbd>r</kbd></button>
</div></aside></main>
<p class="note">LAN only, no authentication. The keyboard on the machine keeps
working; these buttons send the same actions.</p>
<script>
function cmd(a){fetch('/cmd/'+a,{method:'POST'})}
const F=[['frame_age_s','frame age','s'],['encode_ms','jpeg encode','ms'],
['source','obstacles',''],['nav_mode','nav mode',''],
['map_known','map cells known',''],['map_occupied','occupied',''],
['goal','active goal',''],['path_len_m','path','m'],
['clearance_m','clearance','m'],['robot','robot',''],['frames','frames','']];
async function tick(){
 try{const r=await fetch('/status');const j=await r.json();
  document.getElementById('s').innerHTML=F.map(([k,l,u])=>{
   let v=j[k]; if(v===null||v===undefined)v='-';
   if(Array.isArray(v))v=v.join(', ');
   return `<div class="row"><span>${l}</span><span>${v}${u&&v!=='-'?' '+u:''}</span></div>`;
  }).join('');
 }catch(e){}
 setTimeout(tick,1000);}
tick();
</script></body></html>"""


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    app = web.Application()
    app["pub"] = Publisher()
    app.router.add_get("/", index)
    app.router.add_get("/stream", stream)
    app.router.add_get("/status", status)
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

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

HISTORY = 60          # seconds of sparkline, at the collector's 1 Hz

STATE: dict = {"frame": None, "t": 0.0, "platform": {}, "history": {},
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
                for k in ("cpu_pct", "gpu_pct", "npu_pct", "pkg_w"):
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
        "robot": STATE["robot"],
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
/* Platform panel. Gauges and sparklines are hand-written SVG: a chart library
   would be a bigger download than everything else on this page put together,
   and an arc plus a polyline is all that is being drawn. */
.eng{padding:9px 0;border-top:1px solid var(--line)}
.eng:first-child{border-top:0;padding-top:2px}
.eng .hd{display:flex;align-items:baseline;gap:8px}
.eng .nm{font-weight:600;letter-spacing:.04em;font-size:12px}
.eng .pc{margin-left:auto;color:var(--accent);font-variant-numeric:tabular-nums;
font-size:15px}
.eng .sub{color:var(--dim);font-size:11px;display:flex;gap:10px;margin-top:2px;
flex-wrap:wrap}
.eng .na{color:#6b7789;font-size:11px;font-style:italic}
.gw{display:flex;align-items:center;gap:10px;margin-top:5px}
.spark{flex:1;height:30px;display:block}
svg.gauge{flex:0 0 46px;height:46px}
</style></head><body>
<header><span class="dot"></span><b>Edge AI Robotics</b>
<span style="color:var(--dim)">console</span></header>
<main>
<div class="view"><img src="/stream" alt="live"></div>
<aside>
<div class="card"><h2>Platform</h2><div id="pf"></div>
<div class="sub" style="margin-top:8px;color:var(--dim);font-size:11px"
     id="pfsrc"></div></div>
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
const F=[['source','obstacles',''],['nav_mode','nav mode',''],
['map_known','map cells known',''],['map_occupied','occupied',''],
['goal','active goal',''],['path_len_m','path','m'],
['clearance_m','clearance','m'],['robot','robot','']];
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

// ---- Platform panel -------------------------------------------------------
// An arc and a polyline. Everything is built as an SVG string and handed to
// innerHTML once per second: 4 engines x 2 shapes is far too little work to
// justify a canvas, a chart library, or incremental DOM updates.
const R=17, C=2*Math.PI*R;
function gauge(pct){
 // null -> an empty ring, never a ring at zero. A zero-length arc and a
 // missing measurement must not look the same.
 const has = pct!==null && pct!==undefined;
 const off = has ? C*(1-Math.max(0,Math.min(100,pct))/100) : C;
 return `<svg class="gauge" viewBox="0 0 46 46">
  <circle cx="23" cy="23" r="${R}" fill="none" stroke="#232b39" stroke-width="5"/>
  <circle cx="23" cy="23" r="${R}" fill="none" stroke="${has?'#00c7fd':'#2a3444'}"
   stroke-width="5" stroke-linecap="round" stroke-dasharray="${C}"
   stroke-dashoffset="${off}" transform="rotate(-90 23 23)"/>
  <text x="23" y="27" text-anchor="middle" font-size="11"
   fill="${has?'#e6edf7':'#6b7789'}">${has?Math.round(pct):'--'}</text></svg>`;
}
function spark(series,max){
 const s=(series||[]).filter(v=>v!==null&&v!==undefined);
 if(s.length<2) return '<svg class="spark"></svg>';
 const W=100,H=30,m=max||Math.max(1,...s);
 // x spans the full width whatever the sample count, so a page opened 5 s ago
 // and one open for a minute show the same 60 s window, just less filled.
 const n=series.length, step=W/Math.max(1,n-1);
 let d='',pen=false;
 series.forEach((v,i)=>{
  if(v===null||v===undefined){pen=false;return;}
  const x=(i*step).toFixed(1), y=(H-(v/m)*(H-2)-1).toFixed(1);
  d+=(pen?'L':'M')+x+' '+y+' '; pen=true;
 });
 return `<svg class="spark" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none">
  <path d="${d}" fill="none" stroke="#00c7fd" stroke-width="1.2"
   vector-effect="non-scaling-stroke" opacity=".85"/></svg>`;
}
function engine(name,pct,hist,subs,missing){
 const sub = subs.filter(x=>x).join(' &middot; ');
 return `<div class="eng"><div class="hd"><span class="nm">${name}</span>
  <span class="pc">${pct===null||pct===undefined?'--':pct.toFixed(0)+'%'}</span></div>
  <div class="gw">${gauge(pct)}${spark(hist,100)}</div>
  <div class="sub">${sub}</div>
  ${missing.map(m=>`<div class="na">${m}</div>`).join('')}</div>`;
}
function w(v){return (v===null||v===undefined)?null:v.toFixed(1)+' W';}
async function ptick(){
 try{
  const j=await(await fetch('/platform')).json();
  const h=j.history||{}, un=j.unavailable||{};
  // A field that is absent gets the collector's own reason printed. The two
  // reasons it can give -- the driver does not expose it, or the container
  // runtime blocked it -- are different problems and are shown as written.
  const miss=k=>un[k]?[`${k.replace(/_/g,' ')}: ${un[k]}`]:[];
  let html='';
  html+=engine('CPU', j.cpu_pct, h.cpu_pct,
    [w(j.pkg_w)?`package ${w(j.pkg_w)}`:null,
     w(j.core_w)?`core ${w(j.core_w)}`:null,
     w(j.dram_w)?`dram ${w(j.dram_w)}`:null,
     (j.cpu_per_core||[]).length?`${j.cpu_per_core.length} threads`:null],
    miss('pkg_w'));
  html+=engine('GPU  Arc', j.gpu_pct, h.gpu_pct,
    [j.gpu_mhz!==null&&j.gpu_mhz!==undefined?`${j.gpu_mhz.toFixed(0)} MHz act`:null,
     j.gpu_mhz_req?`${j.gpu_mhz_req.toFixed(0)} MHz req`:null,
     w(j.uncore_w)?`uncore ${w(j.uncore_w)}`:null],
    miss('gpu_w').concat(miss('gpu_pct')));
  html+=engine('NPU  AI Boost', j.npu_pct, h.npu_pct,
    [j.npu_mhz?`${j.npu_mhz.toFixed(0)} MHz`:null,
     j.npu_mem_mb?`${j.npu_mem_mb.toFixed(0)} MB`:null],
    miss('npu_w').concat(miss('npu_pct')));
  document.getElementById('pf').innerHTML=html;
  const src=j.sources||{};
  document.getElementById('pfsrc').textContent =
    j.stamp? 'read from '+(src.cpu||'')+', powercap, drm, accel' : 'no telemetry yet';
 }catch(e){}
 setTimeout(ptick,1000);}
ptick();
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

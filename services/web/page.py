# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""The console page, kept out of app.py so the server stays readable.

Layout contract, and the reason every rule below is written the way it is:
**the video and both panels must be visible together, with no scrollbars, at
any browser zoom.** Browser zoom shrinks the CSS viewport, so a layout that
fits at 100 % and overflows at 150 % is a layout that was measured in pixels
somewhere. Nothing here is: the shell is a grid of fractional columns inside a
100dvh column, every panel carries `min-height: 0` so a grid child may actually
shrink below its content, text is sized with `clamp()` against `vmin`, and the
video is `object-fit: contain` inside a flexible cell rather than a width.

The widgets follow reference/intel-toolkit/metrics-panel.js: one row per
metric, carrying `data-max`, with a `<prefix>-<kind>-val` number and a
`<prefix>-<kind>-bar` whose width is the value against that maximum. A missing
value writes an em dash and a zero-width bar -- never a zero that reads as a
measurement. The sparkline is the one addition, hand-written inline SVG.
"""
from __future__ import annotations

# data-max values are DISPLAY CEILINGS, not measurements, and are labelled as
# such: percentages cap at 100 by definition; the wattage ceilings come from
# what this board actually draws (package measured at 56.6 W under the full
# demo, iGPU at 7.9 W) rounded up to a round number so a bar near the top means
# "working hard" rather than "at a limit the silicon knows about".
PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Edge AI Robotics</title><style>
:root{--bg:#0b0e14;--panel:#141922;--line:#232b39;--fg:#e6edf7;--dim:#8b98ad;
--accent:#00c7fd;--bar:#00c7fd;--barbg:#1b2230}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--fg);
font:400 clamp(9px,1.05vmin,15px)/1.45 "Intel One Text",system-ui,
-apple-system,Segoe UI,sans-serif;
display:flex;flex-direction:column}

header{flex:0 0 auto;display:flex;align-items:center;gap:.8em;
padding:.7em 1.2em;background:var(--panel);border-bottom:1px solid var(--line)}
header b{font-weight:600;letter-spacing:.02em;font-size:1.15em}
header .sp{margin-left:auto;color:var(--dim);font-size:.9em}
.dot{width:.6em;height:.6em;border-radius:50%;background:var(--accent);
flex:0 0 auto}

/* Three fractional columns: status | video | platform. minmax(0,Nfr) rather
   than Nfr so a column is allowed to be narrower than its content instead of
   pushing the grid wider than the viewport, which is what creates a
   scrollbar at high zoom. */
main{flex:1 1 auto;min-height:0;display:grid;
grid-template-columns:minmax(0,1fr) minmax(0,2.9fr) minmax(0,1fr);
gap:.8em;padding:.8em}
.col{min-width:0;min-height:0;overflow:hidden;display:flex;
flex-direction:column;gap:.6em}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:.4em;
padding:.8em .9em;min-height:0;overflow:hidden}
.view{min-width:0;min-height:0;display:flex;align-items:center;
justify-content:center}
/* width/height 100% + contain, NOT max-width with auto: the latter refuses to
   scale a 1280x720 composite up, so on a large screen (or at 50% zoom) the
   video sat at native size in the middle of an empty column while the panels
   grew around it. contain still preserves the aspect ratio and never crops. */
.view img{width:100%;height:100%;object-fit:contain;background:transparent}

/* Rows: label, value, bar. Same shape as the reference's metrics-row. */
.metrics-row{padding:.28em 0}
.rl{display:flex;align-items:baseline;gap:.6em}
.rl .k{color:var(--dim);white-space:nowrap;overflow:hidden;
text-overflow:ellipsis}
.rl .v{margin-left:auto;color:var(--fg);font-variant-numeric:tabular-nums;
white-space:nowrap;font-weight:600}
.rl .u{color:var(--dim);font-size:.85em;margin-left:.15em}
.track{height:.42em;margin-top:.28em;background:var(--barbg);
border-radius:.21em;overflow:hidden}
.fill{height:100%;width:0;background:var(--bar);border-radius:.21em;
transition:width .35s ease}
.na{color:#6b7789;font-style:italic;font-size:.85em;padding:.1em 0 .2em}
.sep{height:1px;background:var(--line);margin:.5em 0 .35em}
.grp{color:var(--accent);font-size:.82em;letter-spacing:.08em;
text-transform:uppercase;font-weight:600;padding-bottom:.15em}
.sparkwrap{position:relative;margin-top:.35em;background:#0f141d;
border-radius:.25em;overflow:hidden}
.spark{width:100%;height:2.6em;display:block}
.sparkwrap .cap{position:absolute;right:.35em;top:.15em;color:#5a6678;
font-size:.72em;letter-spacing:.04em}
.foot{flex:0 0 auto;color:var(--dim);font-size:.82em;padding:.35em 1.2em .6em;
text-align:center}
</style></head><body>

<header><span class="dot"></span><b>Edge AI Robotics</b>
<span style="color:var(--dim)">Unitree G1 &middot; MuJoCo &middot; RealSense D455</span>
<span class="sp" id="hdr"></span></header>

<main>
  <div class="col"><div class="panel" id="status"></div></div>
  <div class="col view"><img src="/stream" alt="live composite"></div>
  <div class="col"><div class="panel" id="platform"></div></div>
</main>

<p class="foot">LAN only, no authentication &middot; overlays from the keyboard
on the machine: <b>f</b> floor &nbsp; <b>s</b> detections &nbsp; <b>p</b> cloud
&nbsp; <b>m</b> map &nbsp; <b>r</b> reset</p>

<script>
// ---- the reference's setMetricRow, kept to the letter --------------------
// A null or NaN value writes an em dash and collapses the bar. It must never
// render as 0: an idle engine and an unmeasurable one have to look different.
function setMetricRow(prefix, kind, raw){
  const val = document.getElementById(prefix+'-'+kind+'-val');
  const bar = document.getElementById(prefix+'-'+kind+'-bar');
  const row = val ? val.closest('.metrics-row') : null;
  if (raw == null || isNaN(raw)) {
    if (val) val.textContent = '\\u2014';
    if (bar) bar.style.width = '0%';
    return;
  }
  const n = Number(raw);
  const max = row ? Number(row.dataset.max) || 100 : 100;
  if (val) val.textContent = (Math.abs(n) < 10 ? n.toFixed(1) : Math.round(n));
  if (bar) bar.style.width = Math.max(0, Math.min(100, (n/max)*100)) + '%';
}

function row(prefix, kind, label, unit, max){
  return `<div class="metrics-row" data-max="${max}">
    <div class="rl"><span class="k">${label}</span>
      <span class="v" id="${prefix}-${kind}-val">\\u2014</span>
      <span class="u">${unit}</span></div>
    <div class="track"><div class="fill" id="${prefix}-${kind}-bar"></div></div>
  </div>`;
}

// ---- 60 s sparkline, hand-written SVG ------------------------------------
// x always spans the full width whatever the sample count, so a page opened
// five seconds ago and one open for a minute show the same 60 s window - the
// young one is simply less filled. Gaps (nulls) break the path rather than
// interpolating across a measurement that was never taken.
function spark(series, max){
  const s = series || [];
  const pts = s.filter(v => v != null);
  if (pts.length < 2) return '<div class="sparkwrap"><svg class="spark"></svg></div>';
  const W = 100, H = 26, m = max || Math.max(1, ...pts);
  const step = W / Math.max(1, s.length - 1);
  let d = '', pen = false;
  s.forEach((v,i) => {
    if (v == null) { pen = false; return; }
    const x = (i*step).toFixed(2);
    const y = (H - (Math.max(0,Math.min(max||m, v))/(max||m))*(H-2) - 1).toFixed(2);
    d += (pen ? 'L' : 'M') + x + ' ' + y + ' '; pen = true;
  });
  // Area under the line as well as the line: at low values the stroke alone
  // is a faint scratch near the floor of the box, and the point of a sparkline
  // is to be readable without being read.
  const area = d ? d + `L ${W} ${H} L 0 ${H} Z` : '';
  return `<div class="sparkwrap"><svg class="spark" viewBox="0 0 ${W} ${H}"
      preserveAspectRatio="none">
    <path d="${area}" fill="var(--accent)" opacity=".14"/>
    <path d="${d}" fill="none" stroke="var(--accent)" stroke-width="1.2"
      vector-effect="non-scaling-stroke" opacity=".95"/></svg>
    <span class="cap">60 s</span></div>`;
}

// ---- panels --------------------------------------------------------------
// Built once, then only the values are written, so the browser is not
// re-parsing the whole panel every second and the bar transition can run.
const PLATFORM_HTML =
  `<div class="grp">CPU</div>`
  + row('cpu','load','load','%',100)
  + row('cpu','pkg','package power','W',80)
  + `<div id="cpu-spark"></div><div id="cpu-na"></div><div class="sep"></div>`
  + `<div class="grp">GPU &middot; Arc</div>`
  + row('gpu','load','busy','%',100)
  + row('gpu','pw','power','W',30)
  + row('gpu','mhz','frequency','MHz',2500)
  + `<div id="gpu-spark"></div><div id="gpu-na"></div><div class="sep"></div>`
  + `<div class="grp">NPU &middot; AI Boost</div>`
  + row('npu','load','busy','%',100)
  + row('npu','mhz','frequency','MHz',2050)
  + row('npu','mem','memory','MB',2048)
  + `<div id="npu-spark"></div><div id="npu-na"></div>`;

const STATUS_ROWS = [
  ['source','obstacles'], ['nav_mode','nav mode'],
  ['map_known','map cells known'], ['map_occupied','occupied'],
  ['goal','active goal'], ['path_len_m','path length'],
  ['clearance_m','clearance'], ['stream_res','stream'],
];

document.getElementById('platform').innerHTML = PLATFORM_HTML;

async function tickStatus(){
  try{
    const j = await (await fetch('/status')).json();
    document.getElementById('status').innerHTML =
      STATUS_ROWS.map(([k,l]) => {
        let v = j[k];
        if (v == null) v = '\\u2014';
        else if (Array.isArray(v)) v = v.join(', ');
        else if (typeof v === 'number') v = (Number.isInteger(v) ? v : v.toFixed(2));
        return `<div class="metrics-row"><div class="rl">
          <span class="k">${l}</span><span class="v">${v}</span></div></div>`;
      }).join('');
  }catch(e){}
  setTimeout(tickStatus, 1000);
}

function na(id, un, keys){
  // Print the collector's own reason verbatim. It distinguishes "not exposed
  // by this driver" from "blocked by the container runtime", which are
  // different problems and must not collapse into one grey box.
  const msgs = keys.filter(k => un[k]).map(k => `${k.replace(/_/g,' ')}: ${un[k]}`);
  document.getElementById(id).innerHTML =
    msgs.map(m => `<div class="na">${m}</div>`).join('');
}

async function tickPlatform(){
  try{
    const j = await (await fetch('/platform')).json();
    const h = j.history || {}, un = j.unavailable || {};
    setMetricRow('cpu','load', j.cpu_pct);
    setMetricRow('cpu','pkg',  j.pkg_w);
    setMetricRow('gpu','load', j.gpu_pct);
    setMetricRow('gpu','pw',   j.gpu_w);
    setMetricRow('gpu','mhz',  j.gpu_mhz);
    setMetricRow('npu','load', j.npu_pct);
    setMetricRow('npu','mhz',  j.npu_mhz);
    setMetricRow('npu','mem',  j.npu_mem_mb);
    document.getElementById('cpu-spark').innerHTML = spark(h.cpu_pct, 100);
    document.getElementById('gpu-spark').innerHTML = spark(h.gpu_pct, 100);
    document.getElementById('npu-spark').innerHTML = spark(h.npu_pct, 100);
    na('cpu-na', un, ['pkg_w']);
    na('gpu-na', un, ['gpu_pct','gpu_w','gpu_mhz']);
    na('npu-na', un, ['npu_pct','npu_w','npu_mhz']);
    document.getElementById('hdr').textContent =
      j.stamp ? 'PCM + qmassa' : 'telemetry offline';
  }catch(e){}
  setTimeout(tickPlatform, 1000);
}
tickStatus(); tickPlatform();
</script></body></html>"""

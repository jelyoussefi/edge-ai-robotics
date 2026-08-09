# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""The console page, kept out of app.py so the server stays readable.

Layout contract, and the reason every rule below is written the way it is:
**the header and all three cards must be visible together, with no scrollbars,
at any browser zoom.** Browser zoom shrinks the CSS viewport, so a layout that
fits at 100 % and overflows at 150 % is a layout that was measured in pixels
somewhere. The structure is a fixed three-column grid inside a full-height
column: `minmax(0,Nfr)` tracks so a column may be narrower than its content,
`min-height:0` on every child, all spacing in `em`, and type sized with
`clamp()` against `vmin`.

Type sizes, and the header's spacing, are given as clamps whose CEILING is the
size asked for -- the title tops out at 2.6rem, the subtitle at 1rem, the gap
under the title at 0.5rem and the header's bottom padding at 1.2rem -- rather
than as flat rem values.
A flat rem does not shrink with the viewport, so at 150 % zoom on a small screen
a 2.2rem heading plus a 16:9 video plus two cards is taller than the viewport,
and the fit requirement and the size requirement would contradict each other.
The clamp honours the size wherever there is room for it and gives way where
there is not, which is what "~2.2rem" plus "no scrollbars at 150 %" can both
mean at once.

Palette: a light page, two deep-Intel-blue gauge panels, and the video card left
white so the picture stays the brightest thing on the screen. Every card carries
the same luminous Energy Blue edge.

The page stayed light rather than going dark with the panels, which is the one
judgement call here. The title gradient ends on #1A4B8C at both sides; on a dark
navy page those ends sink into the background and the words "Edge" and
"Robotics" stop being readable, while the middle stays bright. Keeping the page
light preserves the title that was asked for. The cost is that the outer bloom
of the glow is subtler on light than it would be on dark -- the hairline and the
tight ring still read clearly, which is what makes the edge look lit.

Colours inside the dark panels are set by REDEFINING the custom properties on
`.card.metrics`, not by overriding each rule. The bar fill, the group label and
the sparkline stroke all already read `var(--accent)`, so one declaration moves
all three to Energy Blue and there is no second copy of the widget CSS to drift.

Borders and radii stay in px exactly as specified -- they do not drive layout,
so they cannot cause an overflow.

The widgets follow reference/intel-toolkit/metrics-panel.js: one row per metric,
carrying `data-max`, with a `<prefix>-<kind>-val` number and a
`<prefix>-<kind>-bar` whose width is the value against that maximum. A missing
value writes an em dash and a zero-width bar -- never a zero that reads as a
measurement. The sparkline is the one addition, hand-written inline SVG.
"""
from __future__ import annotations

# data-max values are DISPLAY CEILINGS, not measurements, and are labelled as
# such: percentages cap at 100 by definition; the wattage ceilings come from
# what this board actually draws (package measured at 55.4 W under the full
# demo, iGPU at 7.3 W) rounded up to a round number so a bar near the top means
# "working hard" rather than "at a limit the silicon knows about".
PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Edge AI Robotics</title><style>
:root{
  --bg:#EEF2F6; --card:#FFFFFF; --line:#E3E8EC;
  --fg:#1B2733; --dim:#5A6B7B; --na:#93A2B0;
  --accent:#1A4B8C;          /* the blue at both ends of the title gradient */
  --energy:#00C7FD;          /* Intel Energy Blue: the glow, and the bars */
  --olive:#8A9A3B;           /* the subtitle */
  --barbg:#E9EEF3;
  /* The luminous edge: a solid hairline, a tight ring just outside it, a soft
     bloom, then an ordinary drop shadow for depth. The first three are the
     glow; the last is what stops the card floating with no weight. */
  --glow:0 0 0 1px rgba(0,199,253,0.25),
         0 0 18px rgba(0,199,253,0.35),
         0 4px 20px rgba(0,40,90,0.15);
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--fg);
font:400 clamp(9px,1.05vmin,15px)/1.45 "Intel One Text",system-ui,
-apple-system,Segoe UI,sans-serif;
display:flex;flex-direction:column}

/* ---- header: two centred lines, stacked, above the columns ------------- */
header{flex:0 0 auto;text-align:center;
padding:1.1em 1em clamp(.6rem,2vmin,1.2rem)}
.title{
  font-size:clamp(1.15rem,3.9vmin,2.6rem);
  margin-bottom:clamp(.25rem,.9vmin,.5rem);
  font-weight:700;line-height:1.12;letter-spacing:.01em;
  /* The gradient is painted through the glyphs. display:inline-block matters:
     background-clip:text on an inline box clips to the line box rather than
     the text, and the fill comes out cut off on the descenders. */
  display:inline-block;
  background:linear-gradient(90deg,#1A4B8C 0%,#C99A5B 50%,#1A4B8C 100%);
  -webkit-background-clip:text;background-clip:text;
  color:transparent;-webkit-text-fill-color:transparent;
}
.subtitle{font-size:clamp(.6rem,1.5vmin,1rem);color:var(--olive);
letter-spacing:.06em;font-weight:500}

/* ---- three top-aligned cards: status | video | platform ---------------- */
/* align-items:start is the grid spelling of flex-start: each card is only as
   tall as its own content, so the three tops line up instead of the short ones
   stretching to match the tall one. minmax(0,Nfr) rather than Nfr so a column
   may be NARROWER than its content instead of widening the grid past the
   viewport, which is what produces a scrollbar at high zoom. */
/* An explicit 1fr ROW as well as the columns. Without it the row height is
   content-driven, so `max-height:100%` on a card resolves against an
   indefinite height -- a circular constraint that Chrome settles at zero, and
   the video measured 2202x0. With a definite row the percentage means what it
   says and acts as the backstop it was meant to be. */
main{flex:1 1 auto;min-height:0;display:grid;
grid-template-columns:minmax(0,1fr) minmax(0,2.9fr) minmax(0,1fr);
grid-template-rows:minmax(0,1fr);
align-items:start;gap:1em;padding:0 1em 1em}
.card{background:var(--card);border:1px solid rgba(0,199,253,0.55);
border-radius:10px;box-shadow:var(--glow);padding:1em 1.1em;
min-width:0;min-height:0;max-height:100%;overflow:hidden}

/* The two gauge panels are deep Intel blue; the video card stays white so the
   picture is the brightest thing on the page. Everything inside the panels
   re-themes by REDEFINING the custom properties rather than by overriding each
   rule: the bar, the group label and the sparkline stroke all already read
   var(--accent), so one declaration moves all three to Energy Blue and there is
   no second copy of the widget CSS to keep in step. */
.card.metrics{
  background:linear-gradient(160deg,#00285A 0%,#004A86 100%);
  --fg:#FFFFFF;                       /* values */
  --dim:#BCD6EE;                      /* labels */
  --accent:var(--energy);             /* bars, group labels, sparkline */
  --barbg:rgba(255,255,255,0.12);     /* translucent track */
  --line:rgba(255,255,255,0.18);      /* separators */
  --na:#9FC0DE;                       /* the "not exposed" notes */
  color:var(--fg);
}
/* The video card hugs the video: height from the 16:9 image rather than
   stretched to the row, so there is no white letterbox band above and below a
   widescreen frame inside a white card. */
.card.view{align-self:start;padding:.7em}
.view img{width:100%;height:auto;display:block;border-radius:6px;
background:#0b0e14}

/* ---- rows: label, value, bar. The reference's metrics-row shape. ------- */
.metrics-row{padding:.28em 0}
.rl{display:flex;align-items:baseline;gap:.6em}
.rl .k{color:var(--dim);white-space:nowrap;overflow:hidden;
text-overflow:ellipsis}
.rl .v{margin-left:auto;color:var(--fg);font-variant-numeric:tabular-nums;
white-space:nowrap;font-weight:700}
.rl .u{color:var(--dim);font-size:.85em;margin-left:.15em}
.track{height:.42em;margin-top:.28em;background:var(--barbg);
border-radius:.21em;overflow:hidden}
.fill{height:100%;width:0;background:var(--accent);border-radius:.21em;
transition:width .35s ease}
.na{color:var(--na);font-style:italic;font-size:.85em;padding:.1em 0 .2em}
.sep{height:1px;background:var(--line);margin:.55em 0 .4em}
.grp{color:var(--accent);font-size:.82em;letter-spacing:.08em;
text-transform:uppercase;font-weight:700;padding-bottom:.15em}
.sparkwrap{position:relative;margin-top:.35em;background:#F2F6F9;
border:1px solid var(--line);border-radius:6px;overflow:hidden}
.card.metrics .sparkwrap{background:rgba(255,255,255,0.07)}
.spark{width:100%;height:2.6em;display:block}
.sparkwrap .cap{position:absolute;right:.35em;top:.1em;color:var(--na);
font-size:.72em;letter-spacing:.04em}
.foot{flex:0 0 auto;color:var(--dim);font-size:.82em;padding:0 1em .7em;
text-align:center}
</style></head><body>

<header>
  <div class="title">Edge AI Robotics</div>
  <div class="subtitle">Unitree G1 &middot; MuJoCo &middot; RealSense D455</div>
</header>

<main>
  <div class="card metrics" id="status"></div>
  <div class="card view"><img src="/stream" alt="live composite"></div>
  <div class="card metrics" id="platform"></div>
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
  // Area under the line as well as the line: at low values the stroke alone is
  // a faint scratch near the floor of the box, and the point of a sparkline is
  // to be readable without being read.
  const area = d ? d + `L ${W} ${H} L 0 ${H} Z` : '';
  return `<div class="sparkwrap"><svg class="spark" viewBox="0 0 ${W} ${H}"
      preserveAspectRatio="none">
    <path d="${area}" fill="var(--accent)" opacity=".12"/>
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
  }catch(e){}
  setTimeout(tickPlatform, 1000);
}
tickStatus(); tickPlatform();
</script></body></html>"""

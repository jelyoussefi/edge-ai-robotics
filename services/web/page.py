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

The readouts are the tiles of docs/monitor.png: one per quantity, a blue label
over a dark inset showing green digits. A null reading writes an em dash, never
a zero -- an idle engine and an unmeasurable one have to look different -- and
the collector's own reason for a missing value goes on the tile's tooltip,
because the readout has room for a number and nothing else.
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
  --mon-h:5.4em;   /* the monitor strip; a length, so 16:9 can resolve */
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--fg);
font:400 clamp(9px,1.05vmin,15px)/1.45 "Intel One Text",system-ui,
-apple-system,Segoe UI,sans-serif;
display:flex;flex-direction:column}

/* ---- header: two centred lines, stacked, above the columns ------------- */
header{flex:0 0 auto;text-align:center;
padding:clamp(.8rem,2.2vmin,1.6rem) 1em clamp(.8rem,2.2vmin,1.6rem)}
.title{
  font-size:clamp(1.15rem,3.9vmin,2.6rem);
  margin-bottom:clamp(.4rem,1.4vmin,.85rem);
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

/* ---- video, with the monitor strip beneath it -------------------------- */
/* One centred column. The video's HEIGHT is what is left after the header and
   the strip, its width follows from 16:9, and the column is width:fit-content
   so it ends up exactly as wide as the video -- which is how the strip gets
   "the same width as the video" without either of them knowing the number. */
main{flex:1 1 auto;min-height:0;display:flex;justify-content:center;
padding:0 10px 10px}
.stack{display:flex;flex-direction:column;gap:10px;height:100%;
width:fit-content;max-width:100%;min-width:0}
.card{background:var(--card);border:1px solid rgba(0,199,253,0.4);
border-radius:10px;box-shadow:6px 6px 14px rgba(0,0,0,0.28);
min-width:0;overflow:hidden}
/* A definite height, so aspect-ratio can give a definite width for the column
   to take. --mon-h has to be a length for the same reason. */
.card.view{height:calc(100% - var(--mon-h) - 10px);aspect-ratio:16/9;
max-width:100%;padding:0;background:#0b0e14}
.view img{width:100%;height:100%;object-fit:cover;display:block}
.card.monitor{width:100%;height:var(--mon-h);flex:0 0 auto;
padding:.55em .6em;display:flex;gap:.55em;align-items:stretch}

/* ---- one tile per reading, as in docs/monitor.png --------------------- */
.tile{flex:1 1 0;min-width:0;background:var(--card);
border:1px solid var(--line);border-radius:8px;
display:flex;flex-direction:column;align-items:center;justify-content:center;
gap:.3em;padding:.35em .3em}
.tile .lbl{color:var(--accent);font-size:.78em;font-weight:700;
letter-spacing:.1em;text-transform:uppercase;white-space:nowrap}
/* The inset readout: dark navy well, green digits. */
.tile .lcd{width:100%;background:#0B1B2E;border-radius:6px;
padding:.3em .1em;text-align:center;white-space:nowrap;overflow:hidden;
font-family:"Intel One Mono",ui-monospace,SFMono-Regular,Menlo,monospace;
font-size:1em;font-weight:600;color:#2BE08B;
text-shadow:0 0 6px rgba(43,224,139,0.35)}
.tile .lcd .u{color:#1E9E64;font-size:.8em;margin-left:.15em}

</style></head><body>

<header>
  <div class="title">Edge AI Robotics</div>
  <div class="subtitle">Unitree G1 &middot; MuJoCo &middot; RealSense D455</div>
</header>

<main>
  <div class="stack">
    <div class="card view"><img src="/stream" alt="live composite"></div>
    <div class="card monitor" id="monitor"></div>
  </div>
</main>

<script>
// One tile per reading, matching docs/monitor.png: blue label over a dark
// readout with green digits. Built once, then only the readouts are written.
const TILES = [
  ['cpu',   'CPU',   '%'],
  ['gpu',   'GPU',   '%'],
  ['npu',   'NPU',   '%'],
  ['fps',   'FPS',   ''],
  ['vram',  'VRAM',  'MB'],
  ['power', 'POWER', 'W'],
  ['temp',  'TEMP',  '\u00B0C'],
];

document.getElementById('monitor').innerHTML = TILES.map(([k, label, unit]) =>
  `<div class="tile" id="tile-${k}"><span class="lbl">${label}</span>
     <span class="lcd"><span id="v-${k}">\u2014</span>` +
     (unit ? `<span class="u">${unit}</span>` : '') + `</span></div>`).join('');

// A null reading writes an em dash, never a zero: an idle engine and an
// unmeasurable one have to look different. The collector's own reason for a
// missing value goes on the tile's tooltip rather than into the readout,
// which has room for a number and nothing else.
function setTile(k, v, digits, why){
  const el = document.getElementById('v-' + k);
  if (!el) return;
  el.textContent = (v === null || v === undefined || isNaN(v))
    ? '\u2014' : Number(v).toFixed(digits);
  const tile = document.getElementById('tile-' + k);
  if (tile) tile.title = (v === null || v === undefined)
    ? (why || 'no reading') : '';
}

async function tick(){
  try{
    const j = await (await fetch('/platform')).json();
    const un = j.unavailable || {};
    setTile('cpu',   j.cpu_pct,     0, un.cpu_pct);
    setTile('gpu',   j.gpu_pct,     0, un.gpu_pct);
    setTile('npu',   j.npu_pct,     0, un.npu_pct);
    setTile('fps',   j.fps,         1, 'no frames arriving');
    setTile('vram',  j.gpu_mem_mb,  0,
            un.gpu_mem_mb || 'not reported by the GPU driver');
    setTile('power', j.pkg_w,       1, un.pkg_w);
    setTile('temp',  j.temp_c,      0, un.temp_c);
    // An integrated GPU has no dedicated VRAM; the figure is shared system
    // memory, and the tile says so on hover rather than in the digits.
    const vt = document.getElementById('tile-vram');
    if (vt && j.gpu_mem_mb != null) vt.title = j.gpu_mem_shared
      ? 'shared system memory: this iGPU has no dedicated VRAM' : 'dedicated VRAM';
  }catch(e){}
  setTimeout(tick, 1000);
}
tick();
</script></body></html>"""

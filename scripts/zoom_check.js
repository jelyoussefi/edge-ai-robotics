// Emulate browser zoom: at Z% zoom a 1920x1080 screen has 1920/Z CSS px of
// width. deviceScaleFactor keeps the rendered image at screen resolution so
// the screenshots are comparable.
const puppeteer = require('/home/pptruser/node_modules/puppeteer');
(async () => {
  const url = process.argv[2];
  const browser = await puppeteer.launch({
    headless: 'new',
    args: ['--no-sandbox','--disable-dev-shm-usage','--disable-gpu']
  });
  for (const z of [50, 100, 150]) {
    const page = await browser.newPage();
    const w = Math.round(1920 / (z/100)), h = Math.round(1080 / (z/100));
    await page.setViewport({width: w, height: h, deviceScaleFactor: z/100});
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    // Long enough for the MJPEG stream to decode a first frame AND for the
    // web service to accumulate two history samples, or the video measures
    // zero-height and the sparklines are legitimately empty.
    await new Promise(r => setTimeout(r, 9000));
    const m = await page.evaluate(() => {
      const d = document.documentElement;
      const r = (sel) => { const e = document.querySelector(sel);
        if (!e) return null; const b = e.getBoundingClientRect();
        return {w: Math.round(b.width), h: Math.round(b.height),
                top: Math.round(b.top), bottom: Math.round(b.bottom),
                inView: b.top >= -1 && b.left >= -1 &&
                        b.bottom <= window.innerHeight + 1 &&
                        b.right <= window.innerWidth + 1}; };
      const vid = r('.card.view'), mon = r('.card.monitor');
      return {
        scrollW: d.scrollWidth, clientW: d.clientWidth,
        scrollH: d.scrollHeight, clientH: d.clientHeight,
        video: vid, monitor: mon, header: r('header'),
        // "the strip must be the same width as the video"
        widthDelta: (vid && mon) ? Math.abs(vid.w - mon.w) : null,
        videoAspect: vid && vid.h ? +(vid.w / vid.h).toFixed(3) : null,
        tiles: document.querySelectorAll('.card.monitor .tile').length,
        labels: [...document.querySelectorAll('.tile .lbl')]
                  .map(e => e.textContent.trim()).join(','),
        readouts: [...document.querySelectorAll('.tile .lcd')]
                  .map(e => e.textContent.trim().replace(/\s+/g, '')).join(' '),
        statusPanel: document.querySelectorAll('#status').length,
        titlePx: parseFloat(getComputedStyle(
                   document.querySelector('.title')).fontSize),
      };
    });
    const overflowX = m.scrollW > m.clientW, overflowY = m.scrollH > m.clientH;
    console.log(`zoom ${z}% (viewport ${w}x${h} css px)`);
    console.log(`  scrollbars: x=${overflowX} y=${overflowY}`);
    console.log(`  video   ${m.video.w}x${m.video.h} aspect=${m.videoAspect}` +
                ` inView=${m.video.inView}`);
    console.log(`  monitor ${m.monitor.w}x${m.monitor.h} inView=` +
                `${m.monitor.inView}  width delta vs video=${m.widthDelta}px`);
    console.log(`  tiles=${m.tiles} [${m.labels}]`);
    console.log(`  readouts: ${m.readouts}`);
    console.log(`  status panel present=${m.statusPanel}  title=${m.titlePx}px`);
    await page.screenshot({path: `/out/zoom-${z}.png`});
    await page.close();
  }
  await browser.close();
})();

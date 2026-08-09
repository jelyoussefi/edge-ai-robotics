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
      const vis = (sel) => { const e = document.querySelector(sel);
        if (!e) return null; const r = e.getBoundingClientRect();
        return {w: Math.round(r.width), h: Math.round(r.height),
                inView: r.top >= -1 && r.left >= -1 &&
                        r.bottom <= window.innerHeight + 1 &&
                        r.right <= window.innerWidth + 1}; };
      return {
        scrollW: d.scrollWidth, clientW: d.clientWidth,
        scrollH: d.scrollHeight, clientH: d.clientHeight,
        video: vis('.view img'), status: vis('#status'),
        platform: vis('#platform'), header: vis('header'),
        title: vis('.title'), subtitle: vis('.subtitle'),
        // Top alignment: the three cards must share a top edge.
        tops: [...document.querySelectorAll('main > .card')]
                .map(e => Math.round(e.getBoundingClientRect().top)),
        titlePx: parseFloat(getComputedStyle(
                   document.querySelector('.title')).fontSize),
        subPx: parseFloat(getComputedStyle(
                 document.querySelector('.subtitle')).fontSize),
        bodyPx: parseFloat(getComputedStyle(document.body).fontSize),
        statusRows: document.querySelectorAll('#status .metrics-row').length,
        platRows: document.querySelectorAll('#platform .metrics-row').length,
        sparks: document.querySelectorAll('#platform svg.spark path').length,
        overlayButtons: document.querySelectorAll('button').length,
        headings: document.querySelectorAll('h2').length,
      };
    });
    const overflowX = m.scrollW > m.clientW, overflowY = m.scrollH > m.clientH;
    console.log(`zoom ${z}% (viewport ${w}x${h} css px)`);
    console.log(`  scrollbars: x=${overflowX} y=${overflowY}  ` +
                `(scroll ${m.scrollW}x${m.scrollH} vs client ${m.clientW}x${m.clientH})`);
    console.log(`  video   ${m.video.w}x${m.video.h} fullyInView=${m.video.inView}`);
    console.log(`  status  ${m.status.w}x${m.status.h} fullyInView=${m.status.inView} rows=${m.statusRows}`);
    console.log(`  platform ${m.platform.w}x${m.platform.h} fullyInView=${m.platform.inView} rows=${m.platRows} sparklines=${m.sparks}`);
    console.log(`  header  ${m.header.w}x${m.header.h} fullyInView=${m.header.inView}` +
                ` title=${m.titlePx.toFixed(1)}px subtitle=${m.subPx.toFixed(1)}px` +
                ` body=${m.bodyPx.toFixed(1)}px`);
    console.log(`  card tops=${JSON.stringify(m.tops)} aligned=` +
                `${new Set(m.tops).size === 1}`);
    console.log(`  buttons=${m.overlayButtons} h2Titles=${m.headings}`);
    await page.screenshot({path: `/out/zoom-${z}.png`});
    await page.close();
  }
  await browser.close();
})();

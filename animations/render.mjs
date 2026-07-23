// Headless 4K/60fps frame exporter.
//
// Loads the animation page in real Chromium, steps the animation deterministically
// one frame at a time (animation time = frameIndex / fps, never wall-clock), grabs
// each canvas as a PNG, writes a numbered frame sequence + a zip, and (unless
// --no-mp4) runs ffmpeg to stitch the frames straight into an mp4.
//
// Usage:
//   node render.mjs --scene fdm --fps 60 --width 3840 --height 2160
//   node render.mjs --scene sla --params '{"duration":6,"camMode":"orbit"}'
//   node render.mjs --scene sls --limit 30 --width 1280 --height 720   (quick test)
//   node render.mjs --scene mjf --crf 14 --no-zip                      (mp4 only, higher quality)

import puppeteer from 'puppeteer';
import archiver from 'archiver';
import { spawn } from 'node:child_process';
import { createServer } from './serve.mjs';
import { mkdir as mkdirP, rm as rmP } from 'node:fs/promises';
import fs from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = fileURLToPath(new URL('.', import.meta.url));

function parseArgs(argv) {
  const a = { scene: 'fdm', fps: 60, width: 3840, height: 2160, params: {}, limit: 0, zip: true, mp4: true, crf: 16, out: null };
  for (let i = 2; i < argv.length; i++) {
    const k = argv[i];
    const v = argv[i + 1];
    if (k === '--scene') { a.scene = v; i++; }
    else if (k === '--fps') { a.fps = +v; i++; }
    else if (k === '--width') { a.width = +v; i++; }
    else if (k === '--height') { a.height = +v; i++; }
    else if (k === '--limit') { a.limit = +v; i++; }
    else if (k === '--out') { a.out = v; i++; }
    else if (k === '--crf') { a.crf = +v; i++; }
    else if (k === '--no-zip') { a.zip = false; }
    else if (k === '--no-mp4') { a.mp4 = false; }
    else if (k === '--params') { a.params = JSON.parse(v); i++; }
  }
  return a;
}

async function zipDir(dir, zipPath) {
  await new Promise((resolve, reject) => {
    const output = fs.createWriteStream(zipPath);
    const archive = archiver('zip', { zlib: { level: 6 } });
    output.on('close', resolve);
    archive.on('error', reject);
    archive.pipe(output);
    archive.directory(dir, false);
    archive.finalize();
  });
}

// Stitch the frame sequence into an mp4. Resolves { ok, code }; if ffmpeg isn't
// installed we resolve { ok:false, missing:true } so the caller can fall back to
// printing the command instead of crashing the whole export.
function runFfmpeg(args) {
  return new Promise((resolve) => {
    const proc = spawn('ffmpeg', args, { stdio: ['ignore', 'ignore', 'inherit'] });
    proc.on('error', (e) => resolve({ ok: false, missing: e.code === 'ENOENT', error: e }));
    proc.on('close', (code) => resolve({ ok: code === 0, code }));
  });
}

async function main() {
  const args = parseArgs(process.argv);
  const outName = args.out || `${args.scene}_${args.width}x${args.height}_${args.fps}fps`;
  const framesDir = join(ROOT, 'out', outName, 'frames');
  await rmP(join(ROOT, 'out', outName), { recursive: true, force: true });
  await mkdirP(framesDir, { recursive: true });

  // internal static server
  const server = createServer();
  await new Promise((r) => server.listen(0, r));
  const port = server.address().port;
  const url = `http://localhost:${port}/index.html?headless=1`;

  const browser = await puppeteer.launch({
    headless: 'new',
    args: [
      '--no-sandbox',
      '--enable-unsafe-swiftshader',
      '--ignore-gpu-blocklist',
      '--use-gl=angle',
      `--window-size=${args.width},${args.height}`,
    ],
  });
  const page = await browser.newPage();
  page.on('console', (m) => { const tx = m.text(); if (m.type() === 'error') console.log('  [page]', tx); });
  page.on('pageerror', (e) => console.log('  [pageerror]', e.message));
  await page.setViewport({ width: Math.min(args.width, 3840), height: Math.min(args.height, 2160), deviceScaleFactor: 1 });

  await page.goto(url, { waitUntil: 'load' });
  await page.waitForFunction('window.KODA && window.KODA.ready === true', { timeout: 30000 });

  // configure scene + params + exact pixel size
  await page.evaluate((scene, p, w, h) => {
    window.KODA.setScene(scene);
    window.KODA.setParams(p);
    window.KODA.setSize(w, h);
  }, args.scene, args.params, args.width, args.height);

  const duration = await page.evaluate(() => window.KODA.getDuration());
  let total = Math.round(duration * args.fps) + 1;
  if (args.limit > 0) total = Math.min(total, args.limit);

  console.log(`\n  Scene: ${args.scene}   ${args.width}×${args.height} @ ${args.fps}fps`);
  console.log(`  Duration: ${duration}s → ${total} frames`);
  console.log(`  Writing to: out/${outName}/frames\n`);

  const t0 = Date.now();
  for (let i = 0; i < total; i++) {
    const dataUrl = await page.evaluate((idx, fps) => {
      window.KODA.renderFrame(idx, fps);
      return document.getElementById('c').toDataURL('image/png');
    }, i, args.fps);
    const b64 = dataUrl.slice(dataUrl.indexOf(',') + 1);
    const name = `frame_${String(i).padStart(5, '0')}.png`;
    fs.writeFileSync(join(framesDir, name), Buffer.from(b64, 'base64'));
    if (i % 20 === 0 || i === total - 1) {
      const pct = (((i + 1) / total) * 100).toFixed(0);
      const rate = (i + 1) / ((Date.now() - t0) / 1000);
      process.stdout.write(`\r  ${pct}%  (${i + 1}/${total})  ${rate.toFixed(1)} fps render   `);
    }
  }
  console.log('\n');

  await browser.close();
  await new Promise((r) => server.close(r));

  let zipPath = null;
  if (args.zip) {
    zipPath = join(ROOT, 'out', outName, `${outName}_frames.zip`);
    process.stdout.write('  Zipping frames… ');
    await zipDir(framesDir, zipPath);
    console.log('done');
  }

  const mp4Path = join(ROOT, 'out', outName, `${outName}.mp4`);
  const framePattern = join(framesDir, 'frame_%05d.png');
  const ffArgs = [
    '-y', '-framerate', String(args.fps), '-i', framePattern,
    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', String(args.crf),
    '-movflags', '+faststart', mp4Path,
  ];

  if (args.mp4) {
    process.stdout.write('  Encoding mp4 with ffmpeg… ');
    const res = await runFfmpeg(ffArgs);
    if (res.ok) {
      console.log('done');
      console.log(`\n  🎬 mp4:    ${mp4Path}`);
    } else if (res.missing) {
      console.log('skipped — ffmpeg not found on PATH.');
      console.log('\n  Install ffmpeg, then stitch the frames with:');
      console.log(`  ffmpeg ${ffArgs.join(' ')}`);
    } else {
      console.log(`failed (exit ${res.code}). Stitch manually with:`);
      console.log(`  ffmpeg ${ffArgs.join(' ')}`);
    }
  } else {
    console.log('\n  Stitch with ffmpeg:');
    console.log(`  ffmpeg ${ffArgs.join(' ')}`);
  }
  if (zipPath) console.log(`  📦 frames: ${zipPath}`);
  console.log('');
}

main().catch((e) => { console.error(e); process.exit(1); });

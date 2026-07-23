// Static file server for live preview + a /api/render endpoint so the in-app
// "Export video" button can run the headless renderer on this machine.
// `npm run serve`, then open the URL.
import http from 'node:http';
import { readFile, writeFile, rm } from 'node:fs/promises';
import { spawn } from 'node:child_process';
import { extname, join, normalize } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const ROOT = fileURLToPath(new URL('.', import.meta.url));
const PORT = process.env.PORT || 5173;

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.css': 'text/css', '.json': 'application/json', '.png': 'image/png',
  '.svg': 'image/svg+xml', '.ico': 'image/x-icon', '.wasm': 'application/wasm',
};

let rendering = false;

function handleRender(req, res) {
  if (rendering) { res.writeHead(409, { 'Content-Type': 'text/plain' }); return res.end('A render is already in progress.'); }
  let body = '';
  req.on('data', (c) => { body += c; if (body.length > 1e6) req.destroy(); });
  req.on('end', () => {
    let job;
    try { job = JSON.parse(body); } catch { res.writeHead(400); return res.end('bad json'); }
    const args = [join(ROOT, 'render.mjs'),
      '--scene', String(job.scene || 'fdm'),
      '--width', String(job.width || 3840),
      '--height', String(job.height || 2160),
      '--fps', String(job.fps || 60),
      '--crf', String(job.crf ?? 16),
      '--params', JSON.stringify(job.params || {})];
    if (job.zip === false) args.push('--no-zip');
    if (job.mp4 === false) args.push('--no-mp4');

    rendering = true;
    res.writeHead(200, { 'Content-Type': 'text/plain; charset=utf-8', 'Cache-Control': 'no-cache' });
    const child = spawn(process.execPath, args, { cwd: ROOT });
    let done = false;
    const pipe = (d) => { try { res.write(d); } catch {} };
    child.stdout.on('data', pipe);
    child.stderr.on('data', pipe);
    child.on('close', (code) => { done = true; rendering = false; res.end(`\n__DONE__ exit ${code}\n`); });
    child.on('error', (e) => { done = true; rendering = false; res.end(`\n__ERROR__ ${e.message}\n`); });
    // kill only if the client actually disconnects before the render finishes
    res.on('close', () => { if (!done && !child.killed) child.kill(); });
  });
}

// Save/clear the per-scene defaults the app loads on startup (presets.json).
function handlePresets(req, res) {
  let body = '';
  req.on('data', (c) => { body += c; if (body.length > 4e6) req.destroy(); });
  req.on('end', async () => {
    try {
      const data = JSON.parse(body || '{}');
      const file = join(ROOT, 'presets.json');
      if (data.__reset) { await rm(file, { force: true }); }
      else { await writeFile(file, JSON.stringify(data, null, 2)); }
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end('{"ok":true}');
    } catch (e) {
      res.writeHead(500); res.end(String(e.message));
    }
  });
}

export function createServer() {
  return http.createServer(async (req, res) => {
    if (req.method === 'POST' && req.url.split('?')[0] === '/api/render') return handleRender(req, res);
    if (req.method === 'POST' && req.url.split('?')[0] === '/api/presets') return handlePresets(req, res);
    try {
      let path = decodeURIComponent(req.url.split('?')[0]);
      if (path === '/') path = '/index.html';
      const file = normalize(join(ROOT, path));
      if (!file.startsWith(ROOT)) { res.writeHead(403); return res.end('forbidden'); }
      const data = await readFile(file);
      res.writeHead(200, { 'Content-Type': MIME[extname(file)] || 'application/octet-stream' });
      res.end(data);
    } catch {
      res.writeHead(404); res.end('not found');
    }
  });
}

// Run directly? (robust on Windows: compare proper file URLs)
if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  createServer().listen(PORT, () => {
    console.log(`\n  AM animations preview → http://localhost:${PORT}/\n`);
  });
}

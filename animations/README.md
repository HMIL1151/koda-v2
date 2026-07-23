# Additive Manufacturing process animations

Four short, stylized-isometric explainer animations of AM processes — **FDM, SLA,
SLS, MJF** — each building the same small gear. Designed for ~5–10 s of voiceover
per clip. Tune everything live in the browser, then export a **4K / 60 fps** PNG
frame sequence (+ zip) and stitch to mp4 with ffmpeg.

All four share one gear and one "build" engine, so the *process* is the only
variable between them:

| Scene | What it shows |
|-------|---------------|
| **FDM** | A hot nozzle sweeps each layer; a molten bead glows at the active layer. |
| **SLA** | A UV laser draws the cross-section on a resin surface; the part rises out of the (translucent) resin. |
| **SLS** | A single laser dot rasters across a powder tray; a recoater spreads fresh powder each layer. |
| **MJF** | A carriage jets dark fusing agent across the whole layer, then a wide IR lamp sweeps and fuses it. |

## 1. Tune (live preview)

```bash
npm install          # first time only (installs Puppeteer + its Chromium)
npm run serve        # → http://localhost:5173/
```

Open the URL. Pick a process (top tabs) and adjust the panel:

- **Speed & Duration** — build length, **hold at end** (freezes on the finished
  part before looping), preview speed, and how many layers are shown slowly
  before the speed-ramp finishes the part.
- **Print / Build Rate** — part size & height, layer height, tool sweep speed.
- **Camera & Framing** — mode (**static iso / slow orbit / slow push-in**),
  azimuth, elevation, zoom, orbit speed, push-in amount.
- **Colors & Labels** — background, part color, process accent, machine-part toggle.
- **Title & Overlays** — **editable title text**, size/X/Y, callout size, and the
  **camera frame guide** (dashed 16:9 crop + rule-of-thirds; preview only, never exported).
- **Callouts** — pick a callout to edit: toggle it, **edit its text**, set its
  **label position** (screen-anchored, so it stays put during a slow orbit), and
  choose the **target**: *Point on part (3D)* — an X/Y/Z on the part that the
  leader tracks as the camera orbits — or *Fixed in shot* — a stationary screen
  position the leader always points at. So a label on the moving hot-end can keep
  its box still while the leader follows the nozzle, or pin to a fixed spot.
- **Materials Panel / Ratings Panel** — toggle the on-screen panels and pick which
  materials show; tune each process's star ratings (0–5) per category.

The **end-hold** keeps the finished part on screen while the **camera keeps
orbiting** (orbit clips don't freeze at the end). FDM prints its final layer as a
**solid top shell** (no infill on top). SLS/MJF play each layer as *recoat →
fuse*: the recoater covers the surface with powder, then the laser/lamp redraws
the part on it.

Each scene keeps its own settings independently (switching tabs doesn't reset
them). **💾 Save as default** (Defaults group) writes every scene's current
settings to `presets.json`, which is loaded on startup — in both the preview and
the headless exports — so your tuning sticks. **↺ Reset** restores the built-in
defaults.

Everything is rendered **in the WebGL canvas** (title, callouts, panels), so what
you see is exactly what the exported frames contain. The composition is framed to
**16:9** — a preview window of any shape is letterboxed to match the export.

### Export straight from the app

The **Export video** box renders on this machine (requires `npm run serve`):
pick a resolution / fps / quality and hit **🎬 Export video**. A **progress bar**
and streaming log track the render; when it finishes you'll find the frames, zip, and mp4 under
`out/<name>/`. No terminal needed. (**Copy CLI cmd** copies the equivalent
`render.mjs` command if you'd rather run it yourself.)

> Timing is **decoupled from wall-clock time**: the exporter advances the
> animation by exactly `1/fps` per frame, so 4K frames come out frame-accurate no
> matter how slowly they render. What you tune in the browser is what you get.

## 2. Export (4K / 60 fps frames → zip)

```bash
# Full 4K clip (uses each scene's tuned defaults)
node render.mjs --scene fdm

# Override any parameters (same keys as the panel / the copied command)
node render.mjs --scene sla --fps 60 --width 3840 --height 2160 \
  --params '{"duration":7,"camMode":"orbit","orbitSpeed":10,"partColor":"#8fd4e8"}'

# Fast low-res sanity check (first 30 frames, no zip)
node render.mjs --scene sls --width 1280 --height 720 --limit 30 --no-zip
```

One command gives you everything. Output goes to `out/<name>/`:

- `frames/frame_00000.png …` — the numbered 4K sequence
- `<name>_frames.zip` — zipped frames (skip with `--no-zip`)
- `<name>.mp4` — the stitched video, **encoded automatically** (skip with `--no-mp4`)

The mp4 step shells out to `ffmpeg`. If ffmpeg isn't on your PATH the render
still succeeds (frames + zip) and prints the exact command to run once it's
installed — nothing is lost.

### Flags

| Flag | Default | Notes |
|------|---------|-------|
| `--scene` | `fdm` | `fdm` \| `sla` \| `sls` \| `mjf` |
| `--fps` | `60` | frames per second |
| `--width` / `--height` | `3840` / `2160` | output resolution |
| `--params` | `{}` | JSON of any panel parameters to override |
| `--crf` | `16` | x264 quality — lower = better/bigger (visually lossless ~14–18) |
| `--limit` | `0` | render only the first N frames (testing) |
| `--out` | auto | output folder name under `out/` |
| `--no-zip` | — | skip zipping the frames |
| `--no-mp4` | — | skip the ffmpeg encode (frames only) |

At ~4K the renderer runs several frames/second (software-safe WebGL via Chromium),
so an 8 s clip is a couple of minutes; the ffmpeg encode adds a few seconds.

## 3. The mp4

It's already made for you in `out/<name>/<name>.mp4` (H.264, `yuv420p`,
`+faststart` for clean web/editor scrubbing). Want a different master? Re-encode
the frames however you like, e.g. a ProRes 4444 with alpha:

```bash
ffmpeg -framerate 60 -i out/<name>/frames/frame_%05d.png \
  -c:v prores_ks -profile:v 4444 out/<name>/<name>_prores.mov
```

## Layout

```
animations/
  index.html          # app shell + control-panel styles
  serve.mjs           # static dev server (live preview) + /api/render endpoint
  render.mjs          # headless 4K frame exporter (Puppeteer) + ffmpeg encode
  src/
    app.js            # renderer, lights, scene manager, panel, 16:9 framing, window.KODA API
    config.js         # single source of truth for params, per-scene materials/ratings/callouts
    timeline.js       # deterministic time → build-state (frame-accurate)
    camera.js         # orthographic iso rig: static / orbit / push-in
    part.js           # discrete capped layer slabs (solid) + FDM wall/infill part
    hud.js            # 2D overlay (title / callouts / panels / frame guide), composited into WebGL
    scenes/
      fdm.js sla.js sls.js mjf.js
      powderbed.js    # shared base for SLS + MJF
      util.js         # build plate, disposal helpers
  vendor/three.module.js
```

To add a parameter: add it to `SHARED_CONTROLS` (or a scene's `extra`) in
`src/config.js` — it appears in the panel, the copied command, and the exporter
automatically.

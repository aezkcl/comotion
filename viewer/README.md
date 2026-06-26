# CoMotion Path Viewer

Web-based visualization for CoMotion path result JSON files.

## Running the Viewer

**Important:** Serve the project from the **repository root** so asset paths in the JSON (e.g. `resources/panda/...`) resolve correctly.

```bash
# From repo root
npx serve .
# or
python -m http.server 8000
```

Then open: `http://localhost:8000/viewer/`

When the page URL path contains `viewer` (e.g. `/viewer/`), the viewer resolves `resources/` and `?file=` paths against the **parent** of that segment (the repo root). If this script is loaded as `/viewer/js/app.js`, the same repo root is inferred from `import.meta.url`.

If you serve **only** the `viewer/` directory (e.g. `python -m http.server --directory viewer`), the repository includes a symlink **`viewer/resources` → `../resources`** so `/resources/panda/...` is still valid. On Windows without symlink support, either serve from the **repository root** or set `?assetBase=` to a URL where `resources/` is reachable.

Direct app runs write viewer-compatible result JSON when `--output-paths` or
`--output-endpoint-paths` is passed. For example:

```bash
./build/apps/mobile_robot_2d_crossing \
  --scenario parallel \
  --num-robots 4 \
  --output-endpoint-paths \
  --output-dir benchmarks/results/viewer_demo
```

Then load the generated result with:
`http://localhost:8000/viewer/?file=benchmarks/results/viewer_demo/mobile_robot_2d_crossing_parallel_n4_seed0_EndpointPath_result.json`

## Loading Results

- **File picker:** Click "Load JSON" and select a `*_result.json` file.
- **URL parameter:** Add `?file=<path-to-result-json>` (path relative to server root).

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Left | Step backward 1 |
| Right | Step forward 1 |
| Shift+Left | Step backward 10 |
| Shift+Right | Step forward 10 |
| Home | Jump to first timestep |
| End | Jump to last timestep |
| Space | Play / Pause |

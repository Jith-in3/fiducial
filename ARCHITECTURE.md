# Architecture — Visual Fiducial Implementations (LFTag & TopoTag)

> **Audience:** engineers extending, debugging, or reviewing this codebase.
> **Scope:** structure, data flow, configurable parameters, file placement, design rationale.
> Companion docs: [README.md](README.md), [USAGE.md](USAGE.md).
> Reference papers: [LFTag](https://arxiv.org/abs/2006.00842), [TopoTag](https://arxiv.org/abs/1908.01450).
> Organized C4-style, top-down (context → components → code).

> **Caveat (from the author):** this is a "quick and dirty implementation, not very well
> tested, and the localization is somewhat dubious at times" ([README.md](README.md)). Treat
> the code — and this document's description of it — as experimental, not production-grade.

---

## 1. System context (C4 level 1)

`fiducial` is a Rust library that **finds, decodes, and localizes** two families of visual
fiducial markers — **LFTag** and **TopoTag** — in camera images. Around the library sit two
Python tool sets: a **tag generator** (LFTag only) and a **camera-calibration** pair of scripts.
The library has no binary of its own; it is exercised through the two example programs in
[examples/](examples/).

The core detection entry point is [`find_lftags_debug`](src/debug.rs#L249) (LFTag) with a
parallel [`find_topotags_debug`](src/debug.rs#L39) (TopoTag). Both take an RGB image plus a
pinhole camera model and return an annotated RGBA image, printing decoded ids, poses, and timing
to stdout.

| Actor / system | Direction | Contract |
|----------------|-----------|----------|
| USB / V4L webcam | in | Raw RGB frames via `camera_capture`, 640×480 @ 30 fps ([examples/webcam.rs:23-28](examples/webcam.rs#L23-L28)) |
| Image files (jpg/png) | in | Paths passed as CLI args, read via `image::open`, resized to 640×480 ([examples/lftag.rs:32-33](examples/lftag.rs#L32-L33)) |
| Camera intrinsics | in | `cv_pinhole::CameraIntrinsics` (focals, principal point, skew), hardcoded per example |
| System OpenCV 4 | link | `solvePnP` / `Rodrigues` for pose ([src/localize.rs:41-42](src/localize.rs#L41-L42)) |
| Annotated frame | out | `ImageBuffer<Rgba<u8>>` returned; shown in a Piston window (webcam) or discarded (lftag) |
| Console | out | `tag decoded: <id>`, `pose: <...>`, per-frame timing ([src/debug.rs](src/debug.rs)) |
| Generated tags | out | PNG + `.dat` sidecar written by [LFtag/generate.py](LFtag/generate.py) |

```
             +-----------------------------------------------------+
  camera --> |                    fiducial (lib)                   | --> annotated RGBA image
  or files   |  threshold -> segment -> detect -> decode -> pose   | --> stdout (id, pose, ms)
             +-----------------------------------------------------+
                    ^                                   
                    | intrinsics from calibration/ (Python, OpenCV)
                    |
             LFtag/generate.py --> tagN_xxxxx.png + tagN_xxxxx.dat  (printable markers)
```

---

## 2. Component view (C4 level 2/3)

### 2.1 Rust detection library (`src/`)

| Module | Source | Responsibility |
|--------|--------|----------------|
| Library root | [src/lib.rs](src/lib.rs) | Module wiring; image binarization helpers `add_border` and `create_threshold` (adaptive local threshold map). |
| Segmentation | [src/segmentation.rs](src/segmentation.rs) | Single-pass connected-component labelling that also builds a **containment tree** (`child`/`sibling` links) and per-component moments. Produces `Vec<FeatureVector>`. |
| Decode | [src/decode.rs](src/decode.rs) | Walks the component tree to find tag candidates (`detect_lftag`, `detect_topotag`), then geometrically decodes each into an id + pose (`DecodedLFTag`, `DecodedTopotag`). |
| Localize | [src/localize.rs](src/localize.rs) | Pose estimation: `opencv_localize` (OpenCV `solvePnP`, IPPE) for LFTag; `localize` (`lambda-twist` + ARRSAC) for TopoTag; plus coordinate-axis drawing. |
| Debug / orchestration | [src/debug.rs](src/debug.rs) | Top-level pipelines `find_lftags_debug` / `find_topotags_debug` that chain the stages and render annotations. Despite the name, this is the **primary public API** the examples call. |

**Dependency direction** (no cycles):

```
lib.rs ──> segmentation ──> (none)
       └─> decode ──> segmentation, localize
       └─> localize ──> (opencv, cv-*)
       └─> debug ──> segmentation, decode, localize, lib
examples/*.rs ──> debug
```

### 2.2 Python tooling

| Tool | Source | Responsibility |
|------|--------|----------------|
| Tag generator | [LFtag/generate.py](LFtag/generate.py) | Renders LFTag markers to PNG (+`.dat` metadata sidecar). `generate()` for one tag, `generate_class()` for a whole id range. |
| Frame capture | [calibration/collect_img.py](calibration/collect_img.py) | Grabs webcam frames (every 60th, up to ~20) to `*.jpg` for calibration. |
| Camera calibration | [calibration/camera_cal.py](calibration/camera_cal.py) | Runs OpenCV chessboard calibration (8×6 inner corners) over the captured `*.jpg` and prints the camera matrix. |

The Python tools are **standalone** — they do not import the Rust library. Their only coupling is
conceptual: the generator produces markers the library detects, and the calibration output feeds
the intrinsics hardcoded in the examples.

---

## 3. Data flow

There is no long-lived startup/shutdown state in the library; the meaningful flow is
**per image/frame**, inside [`find_lftags_debug`](src/debug.rs#L249). TopoTag follows the same
shape with different detector/decoder calls.

```
RGB frame
  │  input_color.convert()                      -> grayscale Luma<u8>            (debug.rs:252)
  ├─ sobel_gradients(gray)                       -> gradient map (u16), used later for a
  │                                                 decode quality metric        (debug.rs:253)
  │  sharpen3x3(gray)                             -> mild sharpen, s=0.1          (debug.rs:22-26,254)
  ▼
create_threshold(gray)                           -> local mean threshold map,
  │                                                 16×16 blocks, bilinear interp (lib.rs:38-108)
  │  per-pixel binarize (hard_thresh = 10)        -> binary image                (debug.rs:261-277)
  │  add_border(output)                           -> 1px black frame (closes CCs) (lib.rs:25-36)
  ▼
segmentation::to_topo(binary, gray)              -> Vec<FeatureVector> with a
  │                                                 containment tree + moments    (segmentation.rs:169)
  ▼
decode::detect_lftag(topo, root, &mut tags, classes)
  │   recursive: a component whose child count matches a class' node count and
  │   which is "colored" becomes a Root candidate, keyed by bounding box.        (decode.rs:791-849)
  ▼
dilate_fv(...) per node                          -> refines each node centroid by
  │                                                 dilating its connected blob   (debug.rs:211-247,299-315)
  ▼
DecodedLFTag::decode_lftag(root, camera, gradient)                               (decode.rs:315-556)
  │   1. keep the N largest blobs by zom·area; reject if area spread too large
  │   2. find corner points; reject on angle / geometry / collinearity checks
  │   3. brute-force over candidate keypoint sub-positions (iproduct):
  │        opencv_localize (solvePnP IPPE) -> project data cells -> match blobs
  │        -> accumulate 2-bit data + a gradient-normalized residual
  │   4. pick min-residual candidate; reject if quality metric over class limit
  │   5. final_pose via solvePnP over matched data points + baseline
  ▼
annotate output (crosshairs, id text, coord axes)                               (debug.rs:334-407)
  ▼
Some(Rgba image)   +   stdout: "id = <n>", pose, "<ms>ms"
```

**Error / reject branches** — the pipeline is a funnel; a frame yields fewer tags at each gate:

- [`create_threshold`](src/lib.rs#L38) returns `None` (whole pipeline returns `None`) if either
  image dimension is **not divisible by 16** ([src/lib.rs:42-47](src/lib.rs#L42-L47)).
- `decode_lftag` returns `None` on any of: area-spread, geometry, angle, collinearity, or
  residual/quality-metric checks (see [decode.rs:334-519](src/decode.rs#L334-L519)). Rejected
  candidates are simply skipped — no tag is drawn.

**Measured latency:** the only instrumentation is wall-clock timing printed per frame
(`now.elapsed()`), e.g. [examples/lftag.rs:39](examples/lftag.rs#L39) and the `<ms>ms` overlay at
[debug.rs:408-418](src/debug.rs#L408-L418). No fixed numbers are committed in the repo. The
`lftag` example forces a **single Rayon thread** ([examples/lftag.rs:23](examples/lftag.rs#L23)).

---

## 4. Configurable parameters

Almost everything is a **source constant** — this project predates any config file or CLI flags.
Tier C dominates.

### Tier A — runtime config files (no rebuild)
None. There are no config files, environment variables, or CLI flags that alter detection
behavior. The `lftag` example takes image paths as positional args, but that is input, not config.

### Tier B — inputs you pass in code (no source-constant edit, but still a recompile)
| Knob | Where | Default | Effect |
|------|-------|---------|--------|
| Camera intrinsics | [examples/webcam.rs:39-43](examples/webcam.rs#L39-L43), [examples/lftag.rs:34-38](examples/lftag.rs#L34-L38) | webcam: real calibrated values; lftag: generic `focals=320,320 pp=320,240` | Pose accuracy; must match the real camera. |
| Tag classes searched | [debug.rs:291](src/debug.rs#L291) (LFTag 3×3, 4×4), [debug.rs:82](src/debug.rs#L82) (TopoTag 4×4, 3×3) | as listed | Which marker sizes are detected. |
| Capture resolution | [examples/webcam.rs:13-14](examples/webcam.rs#L13-L14), [examples/lftag.rs:33](examples/lftag.rs#L33) | 640×480 | Must be divisible by 16 (see §3). |

### Tier C — source constants (require rebuild)
| Constant | `path:line` | Default | Meaning |
|----------|-------------|---------|---------|
| `threshold_map_res_div` | [src/lib.rs:40](src/lib.rs#L40) | `16` | Block size for the adaptive threshold map; also the divisibility constraint on image size. |
| `hard_thresh` (LFTag) | [src/debug.rs:260](src/debug.rs#L260) | `10` | Absolute floor/ceiling for binarization; handles saturation. |
| `hard_thresh` (TopoTag) | [src/debug.rs:49](src/debug.rs#L49) | `50` | Same, for the TopoTag path. |
| sharpen strength `s` | [src/debug.rs:23](src/debug.rs#L23) | `0.1` | Unsharp amount applied before thresholding. |
| child-area ratios (LFTag) | [src/decode.rs:805,808](src/decode.rs#L805-L808) | `2`, `500` | Which children count as structural vs. node blobs. |
| child-area ratio (TopoTag) | [src/decode.rs:704](src/decode.rs#L704) | `30` | Min child area relative to parent. |
| collinearity min | [src/decode.rs:357](src/decode.rs#L357) | `20.0` | Reject near-collinear node sets. |
| quality metric limit | [src/decode.rs:507,513](src/decode.rs#L507-L513) | `5e-4` (3×3), `1e-3` (4×4) | Residual gate; higher = more permissive, more false positives. |
| ARRSAC config / RNG seed | [src/localize.rs:95](src/localize.rs#L95) | `Config::new(0.0001)`, seed `[0;16]` | TopoTag pose RANSAC (deterministic seed). |

**Most likely to change for a new deployment:** (1) the **camera intrinsics** in the example you
run, (2) the **capture resolution** (keep it a multiple of 16), and (3) the **tag classes** list
if you print a size other than 3×3/4×4.

---

## 5. File placement

```
.
├── Cargo.toml            # crate manifest: deps + pinned versions (see §9)
├── Cargo.lock            # fully pinned dependency graph (committed)
├── README.md             # overview + quick start (this doc set's entry point)
├── ARCHITECTURE.md       # this file
├── USAGE.md              # build / run / port / troubleshoot
├── LICENSE               # MIT
├── .gitignore            # ignores /target only
├── src/
│   ├── lib.rs            # module wiring + thresholding helpers
│   ├── segmentation.rs   # connected components + containment tree -> FeatureVector
│   ├── decode.rs         # candidate detection + geometric decode + tag classes
│   ├── localize.rs       # solvePnP / lambda-twist pose estimation + axis drawing
│   ├── debug.rs          # find_lftags_debug / find_topotags_debug pipelines + rendering
│   └── DejaVuSans.ttf    # font baked in via include_bytes! for id overlays (debug.rs:30)
├── examples/
│   ├── webcam.rs         # live camera demo -> Piston window (calls find_lftags_debug)
│   └── lftag.rs          # batch: decode image paths from argv, print timing (+ hidden #[bench])
├── LFtag/
│   └── generate.py       # LFTag marker generator -> PNG + .dat  (see §7)
└── calibration/
    ├── collect_img.py    # capture webcam frames for calibration
    └── camera_cal.py     # OpenCV chessboard calibration -> prints camera matrix
```

Non-obvious placements:
- **`debug.rs` is the real API.** The name suggests scaffolding, but `find_lftags_debug` /
  `find_topotags_debug` are what every consumer calls — the "debug" refers to the annotated
  image output, not to the code being throwaway.
- **`DejaVuSans.ttf` lives in `src/`** because it is embedded at compile time with `include_bytes!`
  ([src/debug.rs:30](src/debug.rs#L30)); it is not a runtime asset.
- **The directory is `LFtag/`** (lowercase `t`), even though prose in the paper/README often
  writes "LFTag". Scripts and paths must use `LFtag/`.

---

## 6. Key data structures

| Type | Definition | Role |
|------|------------|------|
| `FeatureVector` | [src/segmentation.rs:11-25](src/segmentation.rs#L11-L25) | One connected component: `area`, `bounding_box`, `color` (fg/bg), moments `fom`/`zom` (→ centroid via `get_com*`), and tree links `child`/`sibling`. The unit passed from segmentation into decoding. |
| `TopotagComponents` | [src/decode.rs:21-35](src/decode.rs#L21-L35) | Enum classifying a subtree as `Root` / `Baseline` / `Normal` / `None` during TopoTag detection. |
| `LFTagComponents` | [src/decode.rs:37-51](src/decode.rs#L37-L51) | Same idea for LFTag; `Root` carries the child `nodes`, background blob `bg`, and `class`. |
| `DecodedLFTag` | [src/decode.rs:232-242](src/decode.rs#L232-L242) | Final LFTag result: `data` (id), matched `node_pos`, `initial_pose` / `final_pose`, `class`. |
| `DecodedTopotag` | [src/decode.rs:53-62](src/decode.rs#L53-L62) | Final TopoTag result: `data` (id), node positions, corner ids, `class`. |
| `LFTagClass` / `TopotagClass` | [src/decode.rs:558-564](src/decode.rs#L558-L564), [src/decode.rs:640-645](src/decode.rs#L640-L645) | Marker size family (2×2…5×5). Encodes node counts, grid dimensions, and expected keypoint/data-cell positions used by the decoder. |

Centroids are computed as first-moment / zeroth-moment (`fom[0]/zom`, `fom[1]/zom`),
[src/segmentation.rs:48-57](src/segmentation.rs#L48-L57).

---

## 7. Output / I-O formats

### 7.1 Detection output (Rust)
- **Return value:** `Some(ImageBuffer<Rgba<u8>>)` — the input frame with red crosshairs on
  detected node positions, green crosshairs on expected positions, an `id = N` text label, and
  RGB coordinate axes drawn at the estimated pose. `None` if thresholding failed (§3).
- **Console:** the LFTag path prints `id = N` only as an image overlay; the TopoTag path also
  prints `tag decoded: <id>` and `pose: <Isometry>` to stdout
  ([src/debug.rs:92,202](src/debug.rs#L92-L202)). Timing prints as `<ms>ms`.

The `lftag` example does **not** save the annotated image — the `.save(...)` call is commented out
([examples/lftag.rs:40](examples/lftag.rs#L40)); it only prints timing. The `webcam` example
displays frames live and saves nothing.

### 7.2 Generated marker output (Python)
Each call to `generate()` writes two files sharing a basename
([LFtag/generate.py](LFtag/generate.py)):

- `tagN_ddddd.png` — the marker image, side `(3 + 3·size)·2` px (e.g. 24×24 for size 3).
- `tagN_ddddd.dat` — human-readable `key=value` metadata. Example for a 3×3 tag, id 13:

```
# LFTag marker metadata
algorithm=LFTag
size=3
id=13
data_blocks=7
bits=14
side_length=24
blocks=1,3,0,0,0,0,0
```

`blocks` lists each cell's 2-bit value in encoding order (LSB block first); for id 13 = `0b1101`
that is `1, 3, 0, …`. The `.dat` lets downstream tooling read the id without re-decoding pixels.

### 7.3 Calibration output (Python)
[calibration/camera_cal.py](calibration/camera_cal.py) prints the 3×3 camera matrix `mtx` to
stdout (nothing is written to disk). Those numbers are what you paste into an example's
`CameraIntrinsics` — see the commented reference matrix in
[examples/webcam.rs:34-37](examples/webcam.rs#L34-L37).

---

## 8. Design decisions & rationale (ADR-style)

| ID | Decision | Rationale / consequence |
|----|----------|-------------------------|
| D1 | Single-pass segmentation that builds a **containment tree**, not just labels | LFTag/TopoTag decoding is topological (nodes-inside-a-background). The `child`/`sibling` links ([segmentation.rs:11-25](src/segmentation.rs#L11-L25)) let the decoder match by nesting instead of re-scanning. Cost: intricate merge bookkeeping ([segmentation.rs:277-338](src/segmentation.rs#L277-L338)). |
| D2 | **Adaptive local thresholding** with a 16×16 block mean + bilinear interpolation | Robust to uneven lighting without a per-pixel window. Consequence: image dimensions must be multiples of 16, else `create_threshold` bails ([lib.rs:42-47](src/lib.rs#L42-L47)). |
| D3 | **OpenCV `solvePnP` (IPPE)** for LFTag pose, but **lambda-twist + ARRSAC** for TopoTag | Two code paths reflect two experiments; IPPE (planar) suits LFTag's coplanar cells ([localize.rs:41](src/localize.rs#L41)), while TopoTag uses a P3P/RANSAC route ([localize.rs:95-99](src/localize.rs#L95-L99)). Consequence: a hard dependency on system OpenCV 4. |
| D4 | **Brute-force search** over candidate keypoint sub-positions during decode (`iproduct!`) | Handles the sub-pixel ambiguity of which corner of a cell is the keypoint by trying all and keeping the min-residual fit ([decode.rs:404-500](src/decode.rs#L404-L500)). Consequence: decode cost grows with the candidate product. |
| D5 | Font **embedded via `include_bytes!`** | Makes the annotated-image feature self-contained (no runtime asset path) ([debug.rs:28-37](src/debug.rs#L28-L37)). |
| D6 | Generator writes a **`.dat` sidecar** beside each PNG | The id was otherwise only recoverable from the filename or by decoding pixels; the sidecar records size/id/bits/blocks explicitly for downstream tools. Added on top of the upstream PNG-only generator. |
| D7 | Detection API lives in a module named **`debug`** | Historical: the annotated-image output was a debugging aid that became the main interface. Deviation from the usual "public API in lib.rs" convention — accepted as-is. |

---

## 9. Dependencies & toolchain

**Toolchain:** Rust **nightly** is required — the crate and the `lftag` example use
`#![feature(test)]` and `extern crate test` ([src/lib.rs:1](src/lib.rs#L1),
[examples/lftag.rs:1,16](examples/lftag.rs#L1)). Edition 2018. No `rust-toolchain` file is
committed, so nightly must be selected manually.

| Dependency | Version (pinned) | Integration | Ref |
|------------|------------------|-------------|-----|
| `opencv` | 0.36.0 (feature `opencv-4`) | Links **system OpenCV 4**; `solvePnP`, `Rodrigues` | [Cargo.toml](Cargo.toml), [localize.rs:41-42](src/localize.rs#L41-L42) |
| `image` | 0.21.3 | Image IO, buffers, resize | [Cargo.toml](Cargo.toml) |
| `imageproc` | 0.18.0 | Gradients, drawing, morphology, connected components | [debug.rs:5-19](src/debug.rs#L5-L19) |
| `nalgebra` | 0.21.0 | Linear algebra for poses | many |
| `cv-core` / `cv-pinhole` | 0.10.0 / 0.1.1 | Camera model, keypoints, `WorldPose` | [localize.rs:2-3](src/localize.rs#L2-L3) |
| `lambda-twist` | 0.2.0 | P3P solver (TopoTag pose) | [localize.rs:4](src/localize.rs#L4) |
| `arrsac` | 0.3.0 | RANSAC consensus (TopoTag pose) | [localize.rs:5](src/localize.rs#L5) |
| `euclid` | 0.20.10 | 2D vector geometry in decode | [decode.rs:2](src/decode.rs#L2) |
| `rusttype` | 0.8.2 | Text rendering for id overlays | [debug.rs:7](src/debug.rs#L7) |
| `itertools` | 0.9 | `iproduct!` candidate search | [decode.rs:14](src/decode.rs#L14) |
| `camera_capture` | git (oli-obk fork) | Webcam frames (Linux V4L) — **dev-dependency**, used by examples | [Cargo.toml](Cargo.toml) |
| `piston_window` / `piston-texture` | 0.89.0 / 0.6.0 | GUI window for `webcam` example — **dev-dependency** | [Cargo.toml](Cargo.toml) |
| `rayon` | 1.3.0 | Parallel iteration in `lftag` example — **dev-dependency** | [examples/lftag.rs:9](examples/lftag.rs#L9) |
| `quickcheck(_macros)` | 0.9 | Property tests for segmentation — **dev-dependency** | [segmentation.rs:447](src/segmentation.rs#L447) |

Python tooling additionally needs **numpy + imageio** (generator) and **opencv-python (`cv2`) +
numpy** (calibration).

---

## 10. Quality attributes / measured performance

No performance numbers are committed. The code is instrumented only with per-frame wall-clock
timing (`Instant::now()` / `elapsed()`), printed to stdout and overlaid on the output frame
([debug.rs:408-418](src/debug.rs#L408-L418), [examples/lftag.rs:39](examples/lftag.rs#L39)). There
is a `#[bench] bench_find_lftags` in [examples/lftag.rs:45-53](examples/lftag.rs#L45-L53) and a
`bench_topo` in [segmentation.rs:495-515](src/segmentation.rs#L495-L515), but the former reads
`examples/test12.png`, which is **not in the repo** — so it will not run as-is (see §11). Any
throughput/accuracy claim must be measured locally; none is asserted here.

---

## 11. Known constraints / risks

- **Nightly-only.** `#![feature(test)]` means a stable toolchain will not compile the crate.
- **System OpenCV 4 required.** The `opencv 0.36` crate links against it; missing headers/libs
  cause build failures (see [USAGE.md](USAGE.md) troubleshooting).
- **Image size must be a multiple of 16**, or detection silently returns `None`
  ([lib.rs:42-47](src/lib.rs#L42-L47)).
- **Directory casing:** the generator/calibration paths are `LFtag/` and `calibration/`; the
  README historically wrote `LFTag/` (capital T) — use the on-disk name.
- **Missing test asset:** `bench_find_lftags` references `examples/test12.png`, absent from the
  repo; the benchmark cannot run without supplying that image.
- **`find_topotags_debug` has no example.** The TopoTag path is only reachable by calling the
  function directly; both shipped examples call the LFTag path.
- **Generator side effect:** running `python LFtag/generate.py` directly regenerates the default
  size-4 tag set (thousands of files) via the `__main__` block
  ([LFtag/generate.py](LFtag/generate.py)); import the module and call `generate()` for a single
  marker instead.
- **Author's own warning:** localization is "somewhat dubious at times"; two divergent pose paths
  (D3) and a hand-tuned residual gate (D4) mean results are sensitive to the constants in §4.

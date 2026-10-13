# Usage & Porting Guide — Visual Fiducial Implementations (LFTag & TopoTag)

> **Audience:** anyone building, running, or deploying this on a new machine.
> Internals & parameters: [ARCHITECTURE.md](ARCHITECTURE.md). Overview: [README.md](README.md).

This project has two independent halves:
- **Rust detector** — finds/decodes/localizes tags (needs Rust nightly + system OpenCV 4).
- **Python tools** — the LFTag marker generator and the camera-calibration scripts (need Python +
  a couple of packages; **no Rust build required**).

You can use the Python generator on its own without building any Rust.

---

## Part 1 — Running on an already-set-up machine

### 1.1 Prerequisites
- **Rust nightly** (`#![feature(test)]` is used, so stable will not compile). Verify:
  `rustc +nightly --version`.
- **System OpenCV 4** with development headers and `pkg-config`/`clang` available (the `opencv`
  crate links against it).
- For the **webcam** example: a Linux V4L camera at index 0 and a display server (the demo opens a
  window).
- **Python 3** with `numpy` + `imageio` (generator) and `opencv-python` + `numpy` (calibration).

### 1.2 Build
```bash
# from the repo root
cargo +nightly build --release            # library only
cargo +nightly build --release --examples # library + webcam + lftag examples
```
The first build compiles the `opencv` crate bindings, which is the slow step; it is cached
afterwards. If OpenCV is installed in a non-standard prefix, point the build at it (see §2.2).

### 1.3 Run
```bash
# Batch: decode one or more image files, print per-file timing.
cargo +nightly run --release --example lftag -- path/to/img1.jpg path/to/img2.png

# Live: open the webcam and detect LFTags in a window (Esc to quit).
cargo +nightly run --release --example webcam
```
- `lftag` resizes every input to 640×480 and prints `done in <us> <file>`; it does **not** save
  the annotated image (the save line is commented out — [examples/lftag.rs:40](examples/lftag.rs#L40)).
- `webcam` reads camera 0 at 640×480/30 fps and renders annotated frames live.

**Generate a marker (no Rust needed):**
```bash
# Regenerate the default size-4 tag set into ./generated/ :
python LFtag/generate.py

# Or generate a single tag (e.g. 3x3, id 13) without the big batch:
python - <<'PY'
from LFtag.generate import generate
generate(3, "generated/tag3/tag3_00013.png", 13)   # writes .png + .dat
PY
```
Output directories are created automatically. Each tag produces `tagN_ddddd.png` and a
`tagN_ddddd.dat` sidecar (format in [ARCHITECTURE.md §7.2](ARCHITECTURE.md#72-generated-marker-output-python)).

### 1.4 Runtime controls & output
- **Stop:** `webcam` — press **Esc** (or close the window); `lftag` — exits when all args are done.
- **Console:** decoded ids / poses / timing (LFTag draws `id = N` on the image; TopoTag also prints
  `tag decoded:` and `pose:` to stdout).
- **Files written:** none by the Rust side. The generator writes PNG + `.dat`; calibration writes
  nothing (prints the camera matrix).

### 1.5 Configuration recipes (no rebuild)
There are **no runtime config files or env vars** — detection parameters are compile-time
constants. The only no-rebuild lever on the Rust side is the **input** to `lftag` (which files you
pass). Everything else requires editing source and rebuilding:

| Change | Edit | Then |
|--------|------|------|
| Camera intrinsics | [examples/webcam.rs:39-43](examples/webcam.rs#L39-L43) / [examples/lftag.rs:34-38](examples/lftag.rs#L34-L38) | `cargo +nightly build --release --examples` |
| Which tag sizes are detected | [src/debug.rs:291](src/debug.rs#L291) (LFTag) | rebuild |
| Threshold / sharpen / residual gates | see [ARCHITECTURE.md §4 Tier C](ARCHITECTURE.md#tier-c--source-constants-require-rebuild) | rebuild |

On the generator side, the tag id and size are just arguments to `generate(size, name, id)` — no
rebuild, it's Python.

### 1.6 Exit codes / status signals
| Code | Meaning | Ref |
|------|---------|-----|
| 0 | Normal completion | default |
| 1 | `webcam`: could not open camera 0 | [examples/webcam.rs:24-27](examples/webcam.rs#L24-L27) |
| 2 | `webcam`: opened camera but could not read frames | [examples/webcam.rs:29-32](examples/webcam.rs#L29-L32) |
| (panic) | e.g. OpenCV call `.unwrap()` failure, missing input file | various |

---

## Part 2 — Porting to a new machine

Ordered steps; ⚠ marks the ones that most commonly trip people up.

### 2.1 OS & base tools
Linux is assumed (webcam capture uses V4L; the `camera_capture` fork targets Linux). Install a C
toolchain, `pkg-config`, and `git`:
```bash
sudo apt-get update
sudo apt-get install -y build-essential pkg-config clang git python3 python3-pip
```

### 2.2 ⚠ From-source / version-pinned dependencies — OpenCV 4
The `opencv 0.36` crate needs **system OpenCV 4** with headers. Easiest path on Debian/Ubuntu:
```bash
sudo apt-get install -y libopencv-dev clang libclang-dev
pkg-config --modversion opencv4     # should print a 4.x version
```
If OpenCV lives in a custom prefix, export locations so the crate's build script and `bindgen` find
it, e.g.:
```bash
export OPENCV_LINK_PATHS=/opt/opencv/lib
export OPENCV_INCLUDE_PATHS=/opt/opencv/include/opencv4
export PKG_CONFIG_PATH=/opt/opencv/lib/pkgconfig:$PKG_CONFIG_PATH
```
**Why pinned:** the crate version (0.36) targets a specific OpenCV major (4). A very different
OpenCV can break the generated bindings.

### 2.3 ⚠ Rust nightly
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh   # if rustup not present
rustup toolchain install nightly
rustc +nightly --version
```
The crate will **not** build on stable — `#![feature(test)]` is nightly-only
([src/lib.rs:1](src/lib.rs#L1)).

### 2.4 Get the code
```bash
git clone https://github.com/kingoflolz/fiducial
cd fiducial
```
⚠ **Paths with spaces:** if you place the checkout under a directory whose name contains spaces
(this working copy sits under `Fiducial amrker/`), some tooling and shell one-liners need the path
quoted. Prefer a space-free path if you can.

### 2.5 Configure, build, run
```bash
cargo +nightly build --release --examples
cargo +nightly run --release --example lftag -- some_image.jpg
```
If linking fails, re-check the `OPENCV_*` / `PKG_CONFIG_PATH` from §2.2 point at *this* machine's
install.

### 2.6 Hardware / display / peripherals
- **Camera** (webcam example): confirm a device exists — `ls /dev/video*` — and that your user can
  read it (add to the `video` group if needed). The code uses index **0**
  ([examples/webcam.rs:23](examples/webcam.rs#L23)).
- **Display:** the webcam demo opens a Piston/OpenGL window, so it needs a graphical session
  (X11/Wayland). On a headless box, use the `lftag` example with image files instead.

### 2.7 Adapt to the deployment — camera calibration
Pose accuracy depends on the intrinsics matching your camera. Regenerate them:
```bash
pip install opencv-python numpy
# 1) capture frames of an 8x6 (inner-corner) chessboard:
python calibration/collect_img.py          # writes 60.jpg, 120.jpg, ... in the cwd
# 2) compute the camera matrix from those frames:
python calibration/camera_cal.py           # prints the 3x3 matrix `mtx`
```
Paste the printed focals and principal point into the example you run
([examples/webcam.rs:34-43](examples/webcam.rs#L34-L43) shows the reference matrix and where the
values go), then rebuild. Also confirm the **capture resolution** stays a multiple of 16
([ARCHITECTURE.md §4](ARCHITECTURE.md#4-configurable-parameters)).

### 2.8 Verify the install
End-to-end smoke test that exercises generate → detect:
```bash
# 1) generate a known marker (Python half):
python - <<'PY'
from LFtag.generate import generate
generate(3, "generated/tag3/tag3_00013.png", 13)
PY
cat generated/tag3/tag3_00013.dat     # expect: id=13, size=3, blocks=1,3,0,0,0,0,0

# 2) run the detector on an image of that marker (Rust half):
cargo +nightly run --release --example lftag -- generated/tag3/tag3_00013.png
```
The generator step proves the Python tooling works (correct `.dat`). For the detector step, note
that the raw 24×24 marker is small and printed/photographed markers detect far more reliably than
the bare generated PNG — the meaningful check is that the build runs and prints per-file timing
without panicking.

---

## Part 3 — Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `error[E0554]: #![feature] may not be used on the stable release channel` | Building with stable Rust | Use `cargo +nightly …`; install nightly (§2.3). |
| Build fails linking OpenCV / `could not find opencv4` in pkg-config | System OpenCV 4 dev files missing or in a custom prefix | Install `libopencv-dev` + `libclang-dev`; set `OPENCV_*` / `PKG_CONFIG_PATH` (§2.2). |
| Detector returns nothing / `None` for every frame | Image dimension not a multiple of 16 → `create_threshold` bails | Use a 640×480 (or other ÷16) size; examples already resize to 640×480 ([src/lib.rs:42-47](src/lib.rs#L42-L47)). |
| `webcam` exits with code 1 or 2 | Camera 0 absent or unreadable | Check `ls /dev/video*`, permissions (`video` group); change the index in [examples/webcam.rs:23](examples/webcam.rs#L23). |
| `webcam` panics on window creation | No display / OpenGL context | Run on a graphical session, or use the `lftag` example instead. |
| `python LFtag/generate.py` writes thousands of files | The `__main__` block calls `generate_class(4)` | Import `generate` and call it for one id (§1.3) instead of running the script directly. |
| `FileNotFoundError: The directory 'generated/tag4' does not exist` (older checkouts) | Upstream generator did not create output dirs | This copy auto-creates them ([LFtag/generate.py](LFtag/generate.py)); ensure you're on the updated file. |
| `ModuleNotFoundError: No module named 'imageio'` / `cv2` | Python deps missing | `pip install numpy imageio` (generator) or `pip install opencv-python numpy` (calibration). |
| `bench_find_lftags` fails to open `examples/test12.png` | That test image is not committed | Supply your own image at that path, or edit the bench ([examples/lftag.rs:47](examples/lftag.rs#L47)). |
| Shell can't find files / weird arg splitting | Repo path contains a space (`Fiducial amrker/`) | Quote paths, or move the checkout to a space-free directory (§2.4). |

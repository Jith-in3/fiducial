# Visual Fiducial Implementations (LFTag & TopoTag)

A Rust implementation of two visual fiducial marker algorithms —
[LFTag](https://arxiv.org/abs/2006.00842) and [TopoTag](https://arxiv.org/abs/1908.01450) — that
**finds, decodes, and localizes** markers in camera images, plus Python tools to **generate** LFTag
markers and **calibrate** a camera.

**Status:** experimental. In the author's words: "a quick and dirty implementation, not very well
tested, and the localization is somewhat dubious at times. This shouldn't be used for anything
meaningful. Or do, I'm not the boss of you 🤷". No CI or committed benchmarks.

Pipeline: `threshold → segment (containment tree) → detect → geometric decode → pose`.

## Features
- LFTag detection, decoding, and pose estimation (3×3 and 4×4).
- TopoTag detection/decode/localization (reachable via the library API; no shipped example).
- Two example programs: live **webcam** demo and **batch** image decoding.
- Python **LFTag generator** — PNG marker + `.dat` metadata sidecar.
- Python **camera-calibration** scripts (OpenCV chessboard).

## Quick start
```bash
# Detector (Rust nightly + system OpenCV 4 required):
cargo +nightly build --release --examples
cargo +nightly run --release --example lftag -- path/to/image.jpg   # batch decode
cargo +nightly run --release --example webcam                       # live camera (Esc quits)

# Generate a marker (Python; no Rust build needed):
pip install numpy imageio
python - <<'PY'
from LFtag.generate import generate
generate(3, "generated/tag3/tag3_00013.png", 13)   # -> tag3_00013.png + tag3_00013.dat
PY
```
> Detection silently returns nothing unless the image dimensions are a multiple of 16 (the examples
> resize to 640×480). Running `python LFtag/generate.py` **directly** regenerates the full size-4
> tag set — import `generate()` for a single marker instead.
> Full instructions: **[USAGE.md](USAGE.md)**.

## Repository layout
```
src/           Rust detection library (segmentation, decode, localize, debug pipelines)
examples/      webcam.rs (live) and lftag.rs (batch) demos
LFtag/         generate.py — LFTag marker generator (PNG + .dat)
calibration/   collect_img.py + camera_cal.py — OpenCV camera calibration
```
Full annotated tree: [ARCHITECTURE.md](ARCHITECTURE.md#5-file-placement).

## Configuration (at a glance)
| Change | Where | Rebuild? |
|--------|-------|----------|
| Marker id / size to generate | `generate(size, name, id)` args | No (Python) |
| Camera intrinsics | [examples/webcam.rs](examples/webcam.rs) / [examples/lftag.rs](examples/lftag.rs) | Yes |
| Tag sizes detected | [src/debug.rs:291](src/debug.rs#L291) | Yes |
| Threshold / decode gates | source constants | Yes |

Details: [ARCHITECTURE.md §4](ARCHITECTURE.md#4-configurable-parameters).

## Requirements
- **Rust nightly** (uses `#![feature(test)]`) · edition 2018.
- **System OpenCV 4** + headers (the `opencv 0.36` crate links against it).
- Linux V4L camera + display for the webcam demo.
- **Python 3** with `numpy`/`imageio` (generator) and `opencv-python`/`numpy` (calibration).

Exact-version traps and full setup: [USAGE.md](USAGE.md).

## Documentation
| Doc | Purpose |
|-----|---------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Structure, data flow, configurable parameters, data structures, design decisions |
| [USAGE.md](USAGE.md) | Build, run, port to a new machine, troubleshooting |
| [LFTag paper](https://arxiv.org/abs/2006.00842) · [TopoTag paper](https://arxiv.org/abs/1908.01450) | Algorithm references |

## Notable design decisions
- Segmentation builds a **containment tree**, not just labels — decoding is topological.
- **Adaptive local thresholding** in 16×16 blocks (hence the ÷16 image-size constraint).
- Two pose paths: OpenCV `solvePnP`/IPPE for LFTag, `lambda-twist` + ARRSAC for TopoTag.
- The detection API lives in the module named `debug` (the annotated-image output became the main interface).

See [ARCHITECTURE.md §8](ARCHITECTURE.md#8-design-decisions--rationale-adr-style) for the full list.

## License / acknowledgements
MIT — see [LICENSE](LICENSE). Author: Ben Wang. Builds on `opencv`, `image`, `imageproc`,
`nalgebra`, `cv-core`/`cv-pinhole`, `lambda-twist`, and `arrsac`.

> [!NOTE]
> This is a [suede](https://github.com/pmalacho-mit/suede) dependency.

# canvas_placement — stateless drawing placement for doodlebot canvases

Given a canvas, the drawings already on it, the robots drawing on it right now,
and a new drawing, find where the new drawing goes: a rotation and a position
in one robot's region where its ink clears every existing drawing by the
canvas's buffer, and where the robot's chassis clears every neighbouring robot
that is mid-drawing. If it doesn't fit at its target size, it is shrunk as
little as possible; if it doesn't fit even at the minimum size, the answer is
"nowhere".

This folder is the complete source: the Rust crate (`Cargo.toml`, `src/`), the
`placement` CLI, and the Python extension (`pyproject.toml`). All commands
below run from this folder.

## Stateless

Nothing is kept between calls. Every call rebuilds the region's occupancy from
the `drawings` you pass, so **your list of placed drawings is the only record**
of what space is taken. To reserve a drawing, add it to the list; to clear a
canvas, empty it. Same request plus same `seed` gives the same answer, on any
platform, so a request saved as JSON can be replayed anywhere (see the CLI).

Rebuilding costs about 0.05–0.1 ms per drawing already in the region, which is
small next to the search itself.

## Using it from Python

```python
import canvas_placement as cp

canvas = {
    "width": 1000, "height": 1000,
    "general_buffer": 25,                      # mm between separate drawings
    "regions": [{
        "id": "r", "x": 0, "y": 0, "width": 1000, "height": 1000,
        "robot": "bot1",
        "config": {"cell_mm": 5, "strategy": "scatter", "target_footprint_mm": 300},
    }],
}

result = cp.place({
    "canvas": canvas,
    "drawings": [{"robot_name": "bot1", "strokes": [[(0, 0), (100, 0)]]}],  # world mm
    "active": {"bot2": None},                  # robot -> strokes it is drawing now, or None
    "region_id": "r",
    "job": {
        "commands": [{"kind": "line", "distance": 40, "penDown": True},
                     {"kind": "spin", "degrees": 90},
                     {"kind": "arc", "radius": 20, "degrees": 180}],
        "heading0": 0.0,                       # start heading of the (lead-in-stripped) drawing
        "native_span": 40.0,                   # longest side of the unscaled ink, mm
    },
    "seed": 1234,                              # drives the scatter strategy
    "budget_ms": 1000,                         # caps the shrink-to-fit search (optional)
})
# -> None, or {"angle_deg", "anchor_x", "anchor_y", "scale", "scale_fraction",
#              "attempts", "free_fraction_after"}

cp.free_fractions({"canvas": canvas, "drawings": [...]})   # -> {"r": 0.97}
cp.occupancy({"canvas": canvas, "drawings": [...], "active": {...}, "region_id": "r"})
# -> {"rows", "cols", "ink", "buffered", "active", "bodies", "keepout"}  (each rows*cols bytes, 0/1)
```

All three release the GIL while they run.

### The request

| Field | Meaning |
|---|---|
| `canvas.regions[].config` | `cell_mm` (5), `pen_mm` (5), `clearance_mm` (8), `search_step_cells` (2), `angles_deg` (0, 15, … 345), `strategy` (`"scatter"` or `"origin"`), `target_footprint_mm` (200), `min_footprint_scale` (0.4), `scatter_candidates` (1). Defaults in brackets; omit any to use them. |
| `drawings` | Every drawing on the canvas, with its ink in world mm. Each is reserved in the region of its `robot_name`. |
| `active` | Robots drawing right now. A robot's swept chassis is kept out of every *adjoining* region of the same canvas (sharing an edge, or overlapping; a shared corner doesn't count). |
| `robot_body` | `length_mm` (226), `width_mm` (149), `pen_from_nose_mm` (76). |
| `job.commands` | Drawing commands, `{kind: line, distance, penDown}`, `{kind: spin, degrees}`, `{kind: arc, radius, degrees}`, starting at the first ink (strip any pen-up lead-in first). |
| `scale_tol`, `max_iters` | Shrink-to-fit bisection stop conditions (0.02, 12). |

### The result

`anchor_x`/`anchor_y` is where the first ink point lands, in world mm. Send the
robot there facing `heading0 + angle_deg`, then run the commands scaled by
`scale` (multiply every `distance` and `radius`). `scale_fraction` is `scale`
relative to the region's target size, and `attempts` counts the searches run
(1 means it fitted at full size).

## Prebuilt wheels

Every [GitHub release](https://github.com/mitmedialab/suede.canvas-placement/releases)
carries abi3 wheels for CPython ≥ 3.11 (Linux x86_64/aarch64, macOS universal2,
Windows x64) and an sdist:

```
pip install canvas-placement --find-links https://github.com/mitmedialab/suede.canvas-placement/releases/expanded_assets/v<ver>
```

## Building from source

```
pip install maturin
maturin build --release --out dist        # or: maturin develop --release
cargo test --release                      # Rust unit tests
```

## The CLI

Replays a request as JSON, the same shapes as above:

```
cargo build --release --features cli
./target/release/placement place          < request.json
./target/release/placement free-fractions < request.json
./target/release/placement occupancy      < request.json
```

## How it works

1. **Occupancy.** Each drawing in the region is rasterized at `cell_mm` (a cell
   is ink if its centre is within `pen/2 + clearance` of a stroke), then grown
   by `general_buffer` with an elliptical dilation. Robots drawing in adjoining
   regions add the rectangle their chassis sweeps along each stroke, grown by
   the body's inscribed radius (where a collision is certain at any heading).
2. **Search.** For each rotation, every collision-free offset comes from one FFT
   cross-correlation of the drawing's footprint against the occupancy. `origin`
   takes the free offset nearest the region's top-left corner; `scatter` takes a
   random one. Near a live robot, a candidate must also pass an exact check of
   our own chassis swept along our ink.

   A plain random pick often puts a drawing right beside another while open
   space sits elsewhere: any offset that clears the buffer is as likely as any
   other. With `scatter_candidates` = k > 1, scatter draws one random pick from
   each of k rotations and keeps the one with the most room: the largest
   distance from its footprint to existing ink. (The region's edges don't count;
   lay out regions to keep drawings where they belong.) On a 1 m canvas with
   real drawings, while there was still open space, k = 8 raised the median gap
   to the nearest drawing from 103 mm to 141 mm and cut placements within 80 mm
   of one from 36% to 9%, with the same capacity and ink coverage when filled
   (about 17 drawings), for ~3 ms more per placement on an open canvas and
   ~6 ms once it fills. k = 16 did no better.
3. **Sizing.** Full size first; otherwise a bisection on the scale for the
   largest size that still places.

Module map: `engine.rs` (occupancy, search, sizing), `raster.rs` (rasterizers,
dilation, distance transform), `fftcorr.rs` (FFT correlation), `geometry.rs`
(turtle commands to strokes, body sweep), `rng.rs` (seeded SplitMix64),
`types.rs` (request/response), `python.rs` (PyO3 bindings).

//! The placement search. Everything is rebuilt from the request on each call.

use crate::clock::Deadline;
use crate::fftcorr::Correlator;
use crate::geometry::{body_sweep, bounds, local_strokes, rotate_strokes};
use crate::raster::{
    body_sweep_mask, dilate_ellipse, rasterize, squared_distance_map, within_distance, BodySweep, Footprint,
    Mask,
};
use crate::rng::Rng;
use crate::types::*;
use std::collections::HashMap;

// --------------------------------------------------------------------------- //
// Canvas lookups
// --------------------------------------------------------------------------- //

/// The region a robot draws in: the first one naming it (canvas.py
/// `Canvas.region_for_robot`).
fn region_index_for_robot(canvas: &CanvasSpec, robot: &str) -> Option<usize> {
    canvas.regions.iter().position(|r| r.robot.as_deref() == Some(robot))
}

/// Could a robot working `b` reach into `a`? Overlapping or edge-sharing, but
/// not corner-only (canvas.py `Region.adjoins`).
fn adjoins(a: &RegionSpec, b: &RegionSpec) -> bool {
    const TOL: f64 = 1e-6;
    let x_overlap = (a.x + a.width).min(b.x + b.width) - a.x.max(b.x);
    let y_overlap = (a.y + a.height).min(b.y + b.height) - a.y.max(b.y);
    if x_overlap < -TOL || y_overlap < -TOL {
        return false;
    }
    !(x_overlap.abs() <= TOL && y_overlap.abs() <= TOL)
}

fn grid_shape(region: &RegionSpec) -> (usize, usize) {
    let cell = region.config.cell_mm;
    let rows = ((region.height / cell).ceil() as i64).max(1) as usize;
    let cols = ((region.width / cell).ceil() as i64).max(1) as usize;
    (rows, cols)
}

// --------------------------------------------------------------------------- //
// Committed ink
// --------------------------------------------------------------------------- //

/// Region `ri`'s occupancy from the drawings: each drawing is stamped into its
/// robot's region, at the cells its ink actually covers.
fn build_grid(canvas: &CanvasSpec, ri: usize, drawings: &[DrawingSpec]) -> Mask {
    let region = &canvas.regions[ri];
    let (rows, cols) = grid_shape(region);
    let cell = region.config.cell_mm;
    let half = region.config.half_width_cells();
    let mut grid = Mask::new(rows, cols);
    for d in drawings {
        if region_index_for_robot(canvas, &d.robot_name) != Some(ri) {
            continue;
        }
        let Some(fp) = rasterize(&d.strokes, cell, half) else { continue };
        // The ink's minimum sits at mask cell (pad, pad), so the mask's own
        // origin is pad cells up and left of it. (Python's add_drawings left out
        // the `- pad`, reserving every restored drawing pad cells off its ink.)
        let top = ((fp.min_y - region.y) / cell).round_ties_even() as i64 - fp.pad;
        let left = ((fp.min_x - region.x) / cell).round_ties_even() as i64 - fp.pad;
        grid.stamp(&fp.mask, top, left);
    }
    grid
}

fn free_fraction(grid: &Mask) -> f64 {
    1.0 - grid.count() as f64 / grid.data.len() as f64
}

// --------------------------------------------------------------------------- //
// Neighbouring robots mid-drawing
// --------------------------------------------------------------------------- //

/// Cells region `ri` must not draw in because an adjoining robot's body sweeps
/// them, grown by `clearance` mm (canvas.py `_active_robot_keepout`).
fn active_keepout(
    canvas: &CanvasSpec,
    ri: usize,
    active: &HashMap<String, Option<Vec<Stroke>>>,
    body: &RobotBody,
    clearance: f64,
) -> Mask {
    let region = &canvas.regions[ri];
    let (rows, cols) = grid_shape(region);
    let cell = region.config.cell_mm;
    let blocked = Mask::new(rows, cols);

    // One cell of slop so centre-sampling can't leave a sliver of body free.
    let margin = cell;
    let clearance = clearance.max(0.0);
    let reach = body.reach_mm() + margin + clearance;
    if active.is_empty() || reach <= 0.0 {
        return blocked;
    }
    let (zx0, zy0) = (region.x - reach, region.y - reach);
    let (zx1, zy1) = (region.x + region.width + reach, region.y + region.height + reach);

    let mut relevant: Vec<&Stroke> = Vec::new();
    for (name, strokes) in active {
        let Some(strokes) = strokes else { continue };
        let Some(oi) = region_index_for_robot(canvas, name) else { continue };
        if oi == ri || !adjoins(region, &canvas.regions[oi]) {
            continue;
        }
        for st in strokes {
            let Some((x0, y0, x1, y1)) = bounds(std::slice::from_ref(st)) else { continue };
            if x0 > zx1 || x1 < zx0 || y0 > zy1 || y1 < zy0 {
                continue;
            }
            relevant.push(st);
        }
    }
    if relevant.is_empty() {
        return blocked;
    }

    // Room around the region so a body just outside can still grow into it.
    let pad = (clearance / cell).ceil() as usize + usize::from(clearance > 0.0);
    let mut boxm = Mask::new(rows + 2 * pad, cols + 2 * pad);
    let (ox, oy) = (region.x - pad as f64 * cell, region.y - pad as f64 * cell);
    let to_cell = |p: Point| ((p.0 - ox) / cell, (p.1 - oy) / cell);
    for st in relevant {
        for w in st.windows(2) {
            if let Some(q) = body_sweep(w[0], w[1], body, margin) {
                boxm.fill_convex(&q.map(to_cell));
            }
        }
        if st.len() == 1 {
            // Heading unknown: worst case over all headings.
            let (cx, cy) = to_cell(st[0]);
            boxm.fill_disc(cx, cy, (body.reach_mm() + margin) / cell);
        }
    }
    if clearance > 0.0 && boxm.any() {
        boxm = within_distance(&boxm, clearance / cell);
    }
    boxm.crop(pad, pad, rows, cols)
}

// --------------------------------------------------------------------------- //
// Region snapshot
// --------------------------------------------------------------------------- //

/// Every grid the search consults for one region, built from the request.
pub struct Layers {
    /// Committed ink.
    pub ink: Mask,
    /// Ink grown by the canvas's general buffer.
    pub buffered: Mask,
    /// Live neighbours' swept bodies grown by `max(general_buffer,
    /// min_overhang)`: poses that are certain to collide (broad phase).
    pub active: Mask,
    /// Live neighbours' swept bodies, ungrown (narrow phase ground truth).
    pub bodies: Mask,
    /// What candidate footprints are tested against: `buffered | active`.
    pub keepout: Mask,
}

fn build_layers(
    canvas: &CanvasSpec,
    ri: usize,
    drawings: &[DrawingSpec],
    active: &HashMap<String, Option<Vec<Stroke>>>,
    body: &RobotBody,
) -> Layers {
    let cell = canvas.regions[ri].config.cell_mm;
    let ink = build_grid(canvas, ri, drawings);
    // Our ink must clear committed ink by the general buffer...
    let buffer_cells = (canvas.general_buffer / cell).ceil().max(0.0) as usize;
    let buffered = if buffer_cells > 0 && ink.any() { dilate_ellipse(&ink, buffer_cells) } else { ink.clone() };
    // ...and our pen must stay min_overhang clear of any live body, inside which
    // a collision is certain at every heading. Two constraints over the same
    // set, so the wider wins rather than adding up.
    let clearance = canvas.general_buffer.max(body.min_overhang_mm());
    let active_grown = active_keepout(canvas, ri, active, body, clearance);
    let bodies = active_keepout(canvas, ri, active, body, 0.0);
    let mut keepout = buffered.clone();
    keepout.or_assign(&active_grown);
    Layers { ink, buffered, active: active_grown, bodies, keepout }
}

struct Context<'a> {
    region: &'a RegionSpec,
    rows: usize,
    cols: usize,
    /// Committed ink only (for the free fraction).
    grid: Mask,
    /// `None` when nothing is blocked: every offset is free.
    corr: Option<Correlator>,
    /// Neighbours' swept bodies, ungrown: the narrow phase's ground truth.
    active_bodies: Mask,
    /// `active_bodies`' bbox grown by our reach: (top, left, bottom, right).
    reach_bbox: Option<(i64, i64, i64, i64)>,
    body: RobotBody,
    /// Per cell, how far it is from committed ink (and from the region's edges,
    /// per `edge_weight`), in cells. Only built when scatter compares several
    /// candidates.
    clearance: Option<Vec<f64>>,
}

/// Each cell's distance to the nearest committed ink, or to the region's edge
/// divided by `edge_weight` if that is nearer — the quantity best-candidate
/// scatter maximizes. Cells, centre to centre.
fn clearance_map(ink: &Mask, edge_weight: f64) -> Vec<f64> {
    let (rows, cols) = (ink.rows, ink.cols);
    let mut map: Vec<f64> = squared_distance_map(ink).into_iter().map(f64::sqrt).collect();
    if edge_weight > 0.0 {
        for r in 0..rows {
            let to_row_edge = (r as f64 + 0.5).min(rows as f64 - r as f64 - 0.5);
            for c in 0..cols {
                let to_edge = to_row_edge.min(c as f64 + 0.5).min(cols as f64 - c as f64 - 0.5);
                let v = &mut map[r * cols + c];
                *v = v.min(to_edge / edge_weight);
            }
        }
    }
    map
}

fn prepare<'a>(
    canvas: &'a CanvasSpec,
    ri: usize,
    drawings: &[DrawingSpec],
    active: &HashMap<String, Option<Vec<Stroke>>>,
    body: &RobotBody,
) -> Context<'a> {
    let region = &canvas.regions[ri];
    let (rows, cols) = grid_shape(region);
    let cell = region.config.cell_mm;
    let Layers { ink, keepout, bodies, .. } = build_layers(canvas, ri, drawings, active, body);

    let reach_bbox = bodies.bbox().map(|(r0, c0, r1, c1)| {
        let pad = (body.reach_mm() / cell).ceil() as i64 + 1;
        (
            (r0 as i64 - pad).max(0),
            (c0 as i64 - pad).max(0),
            (r1 as i64 + pad).min(rows as i64 - 1),
            (c1 as i64 + pad).min(cols as i64 - 1),
        )
    });

    let cfg = &region.config;
    let clearance = (cfg.strategy == Strategy::Scatter && cfg.scatter_candidates > 1)
        .then(|| clearance_map(&ink, cfg.edge_weight));

    Context {
        region,
        rows,
        cols,
        grid: ink,
        corr: keepout.any().then(|| Correlator::new(&keepout)),
        active_bodies: bodies,
        reach_bbox,
        body: *body,
        clearance,
    }
}

// --------------------------------------------------------------------------- //
// Search
// --------------------------------------------------------------------------- //

struct Found {
    angle_deg: f64,
    top: usize,
    left: usize,
    anchor_x: f64,
    anchor_y: f64,
    mask: Mask,
}

/// One rotation's footprint, plus what the narrow phase lazily needs.
struct AngleState {
    angle: f64,
    rotated: Vec<Stroke>,
    fp: Footprint,
    sweep: Option<BodySweep>,
}

impl Context<'_> {
    /// Would our chassis, drawing at (top, left), touch a live neighbour's?
    fn body_collides(&self, st: &mut AngleState, top: usize, left: usize) -> bool {
        let Some((btop, bleft, bbottom, bright)) = self.reach_bbox else { return false };
        let (t, l) = (top as i64, left as i64);
        let (mh, mw) = (st.fp.mask.rows as i64, st.fp.mask.cols as i64);
        if t > bbottom || t + mh - 1 < btop || l > bright || l + mw - 1 < bleft {
            return false; // our ink lands nowhere near a live robot
        }
        let cell = self.region.config.cell_mm;
        let sweep = st
            .sweep
            .get_or_insert_with(|| body_sweep_mask(&st.rotated, &st.fp, &self.body, cell));
        let (r0, c0) = (t - sweep.delta, l - sweep.delta);
        let (bh, bw) = (sweep.mask.rows as i64, sweep.mask.cols as i64);
        let (lo_r, lo_c) = (r0.max(0), c0.max(0));
        let (hi_r, hi_c) = ((r0 + bh).min(self.rows as i64), (c0 + bw).min(self.cols as i64));
        for r in lo_r..hi_r {
            let ours = &sweep.mask.row((r - r0) as usize)[(lo_c - c0) as usize..(hi_c - c0) as usize];
            let theirs = &self.active_bodies.row(r as usize)[lo_c as usize..hi_c as usize];
            if ours.iter().zip(theirs).any(|(&a, &b)| a & b != 0) {
                return true;
            }
        }
        false
    }

    fn found(&self, st: &AngleState, anchor_local: Point, top: usize, left: usize) -> Found {
        let cell = self.region.config.cell_mm;
        let (ac, ar) = st.fp.to_px(anchor_local);
        Found {
            angle_deg: st.angle,
            top,
            left,
            anchor_x: self.region.x + (left as f64 + ac) * cell,
            anchor_y: self.region.y + (top as f64 + ar) * cell,
            mask: st.fp.mask.clone(),
        }
    }

    /// How much room a footprint placed at (top, left) has: the least clearance
    /// over the cells it covers.
    fn footprint_clearance(&self, map: &[f64], mask: &Mask, top: usize, left: usize) -> f64 {
        let mut least = f64::INFINITY;
        for r in 0..mask.rows {
            let row = &map[(top + r) * self.cols + left..][..mask.cols];
            for (&m, &c) in mask.row(r).iter().zip(row) {
                if m != 0 && c < least {
                    least = c;
                }
            }
        }
        least
    }

    /// A collision-free pose for `strokes` (rotation + offset), or None
    /// (canvas.py `Region.try_place`).
    ///
    /// Scatter with `scatter_candidates` = k > 1 draws one candidate from each of
    /// the first k rotations (in shuffled order) that have room, each picked
    /// exactly as k = 1 picks its single pose, and keeps the one whose footprint
    /// has the most clearance (ties: the earlier). k = 1 is a plain uniform pick.
    fn try_place(&self, strokes: &[Stroke], rng: &mut Rng) -> Option<Found> {
        if strokes.is_empty() {
            return None;
        }
        let cfg = &self.region.config;
        let half = cfg.half_width_cells();
        let step = cfg.search_step_cells.max(1) as usize;
        let scatter = cfg.strategy == Strategy::Scatter;
        let mut angles = cfg.angles_deg.clone();
        if scatter {
            rng.shuffle(&mut angles);
        }
        let narrow = self.reach_bbox.is_some();
        // Best-candidate scatter: (clearance, pose) of the roomiest so far.
        let candidates = cfg.scatter_candidates.max(1);
        let mut drawn = 0u32;
        let mut roomiest: Option<(f64, Found)> = None;

        let mut best: Option<Found> = None;
        let mut best_key: Option<(usize, usize)> = None;

        for &angle in &angles {
            let rotated = rotate_strokes(strokes, angle);
            let fp = rasterize(&rotated, cfg.cell_mm, half)?; // no ink at all
            let anchor_local = rotated[0][0];
            let (mh, mw) = (fp.mask.rows, fp.mask.cols);
            if mh > self.rows || mw > self.cols {
                continue; // this rotation can't fit at all
            }
            let mut st = AngleState { angle, rotated, fp, sweep: None };

            // Free offsets, subsampled by `step`, row-major (corner first).
            let (vr, vc) = (self.rows - mh + 1, self.cols - mw + 1);
            let coords: Vec<(usize, usize)> = match &self.corr {
                None => (0..vr).step_by(step).flat_map(|r| (0..vc).step_by(step).map(move |c| (r, c))).collect(),
                Some(corr) => {
                    let free = corr.free_offsets(&st.fp.mask);
                    (0..vr)
                        .step_by(step)
                        .flat_map(|r| (0..vc).step_by(step).map(move |c| (r, c)))
                        .filter(|&(r, c)| free.get(r, c))
                        .collect()
                }
            };
            if coords.is_empty() {
                continue;
            }

            if scatter {
                // This rotation's pick: a uniform free offset, or with a live
                // neighbour, the first in a shuffled order whose chassis clears it.
                let pick = if !narrow {
                    Some(coords[rng.below(coords.len())])
                } else {
                    let mut order: Vec<usize> = (0..coords.len()).collect();
                    rng.shuffle(&mut order);
                    order.into_iter().map(|i| coords[i]).find(|&(t, l)| !self.body_collides(&mut st, t, l))
                };
                let Some((t, l)) = pick else {
                    continue; // every free offset here would clip a live robot
                };
                let Some(map) = self.clearance.as_deref().filter(|_| candidates > 1) else {
                    return Some(self.found(&st, anchor_local, t, l)); // k = 1
                };
                let room = self.footprint_clearance(map, &st.fp.mask, t, l);
                if roomiest.as_ref().map_or(true, |(best_room, _)| room > *best_room) {
                    roomiest = Some((room, self.found(&st, anchor_local, t, l)));
                }
                drawn += 1;
                if drawn == candidates {
                    break;
                }
                continue;
            }

            // Origin: first survivor is this angle's best; stop once we can't
            // beat the incumbent (later coords are further from the corner).
            for &(t, l) in &coords {
                if best_key.is_some_and(|k| (t, l) >= k) {
                    break;
                }
                if narrow && self.body_collides(&mut st, t, l) {
                    continue;
                }
                best_key = Some((t, l));
                best = Some(self.found(&st, anchor_local, t, l));
                break;
            }
            if best_key == Some((0, 0)) {
                break; // nothing beats the corner
            }
        }
        if scatter {
            return roomiest.map(|(_, found)| found);
        }
        best
    }
}

// --------------------------------------------------------------------------- //
// Public API
// --------------------------------------------------------------------------- //

fn region_index(canvas: &CanvasSpec, id: &str) -> Result<usize, String> {
    canvas
        .regions
        .iter()
        .position(|r| r.id == id)
        .ok_or_else(|| format!("no region with id {id:?} on this canvas"))
}

/// Place `req.job` in region `req.region_id` at its target size, shrinking only
/// as far as needed to fit (robots.py `_place_scaled` + canvas.py `try_place`).
/// `Ok(None)` if it won't fit even at `min_footprint_scale`.
pub fn place(req: &PlaceRequest) -> Result<Option<PlaceResult>, String> {
    let ri = region_index(&req.canvas, &req.region_id)?;
    if req.job.native_span <= 0.0 {
        return Ok(None);
    }
    let ctx = prepare(&req.canvas, ri, &req.drawings, &req.active, &req.robot_body);
    let cfg = &ctx.region.config;
    let target = cfg.target_footprint_mm / req.job.native_span;
    let deadline = Deadline::after_ms(req.budget_ms);
    let mut rng = Rng::new(req.seed);
    let mut attempts = 0u32;

    let mut attempt = |s: f64, rng: &mut Rng| {
        attempts += 1;
        let strokes = local_strokes(&req.job.commands, target * s, req.job.heading0);
        ctx.try_place(&strokes, rng).map(|f| (f, s))
    };

    // Full size first: the common case on a canvas with room.
    let mut best = attempt(1.0, &mut rng);
    if best.is_none() {
        // Fit is monotonic in size: bisect for the largest scale that places.
        let (mut lo, mut hi) = (cfg.min_footprint_scale, 1.0);
        let mut iters = 0;
        while hi - lo > req.scale_tol && iters < req.max_iters {
            if deadline.passed() {
                break; // keep the best fit already proved
            }
            iters += 1;
            let mid = (lo + hi) / 2.0;
            match attempt(mid, &mut rng) {
                Some(hit) => {
                    best = Some(hit);
                    lo = mid;
                }
                None => hi = mid,
            }
        }
    }

    Ok(best.map(|(f, s)| {
        let mut after = ctx.grid.clone();
        after.stamp(&f.mask, f.top as i64, f.left as i64);
        PlaceResult {
            angle_deg: f.angle_deg,
            anchor_x: f.anchor_x,
            anchor_y: f.anchor_y,
            scale: target * s,
            scale_fraction: s,
            attempts,
            free_fraction_after: free_fraction(&after),
        }
    }))
}

/// Free fraction of every region, keyed by region id.
pub fn free_fractions(req: &FreeFractionsRequest) -> HashMap<String, f64> {
    (0..req.canvas.regions.len())
        .map(|ri| (req.canvas.regions[ri].id.clone(), free_fraction(&build_grid(&req.canvas, ri, &req.drawings))))
        .collect()
}

/// The grids a placement in `req.region_id` would be searched against — for
/// tests, debugging and visualization. Same code path as [`place`].
pub fn occupancy(req: &OccupancyRequest) -> Result<OccupancyLayers, String> {
    let ri = region_index(&req.canvas, &req.region_id)?;
    let l = build_layers(&req.canvas, ri, &req.drawings, &req.active, &req.robot_body);
    Ok(OccupancyLayers {
        rows: l.ink.rows,
        cols: l.ink.cols,
        ink: l.ink.data,
        buffered: l.buffered.data,
        active: l.active.data,
        bodies: l.bodies.data,
        keepout: l.keepout.data,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(side: f64) -> Vec<Command> {
        let mut c = Vec::new();
        for _ in 0..4 {
            c.push(Command::Line { distance: side, pen_down: true });
            c.push(Command::Spin { degrees: 90.0 });
        }
        c
    }

    fn request(strategy: Strategy, drawings: Vec<DrawingSpec>) -> PlaceRequest {
        PlaceRequest {
            canvas: CanvasSpec {
                width: 1000.0,
                height: 1000.0,
                general_buffer: 25.0,
                regions: vec![RegionSpec {
                    id: "r".into(),
                    x: 0.0,
                    y: 0.0,
                    width: 1000.0,
                    height: 1000.0,
                    robot: Some("bot".into()),
                    config: PlacementConfig { strategy, target_footprint_mm: 300.0, ..Default::default() },
                }],
            },
            drawings,
            active: HashMap::new(),
            robot_body: RobotBody::default(),
            region_id: "r".into(),
            job: Job { commands: square(100.0), heading0: 0.0, native_span: 100.0 },
            seed: 1,
            budget_ms: None,
            scale_tol: 0.02,
            max_iters: 12,
        }
    }

    /// Place repeatedly, feeding each result back as a drawing (as robots.py
    /// will), and return the drawings.
    fn fill(strategy: Strategy, n: usize) -> Vec<DrawingSpec> {
        let mut drawings = Vec::new();
        for i in 0..n {
            let mut req = request(strategy, drawings.clone());
            req.seed = i as u64;
            let Some(p) = place(&req).unwrap() else { break };
            let world = local_strokes(&req.job.commands, p.scale, p.angle_deg)
                .into_iter()
                .map(|s| s.into_iter().map(|(x, y)| (x + p.anchor_x, y + p.anchor_y)).collect())
                .collect();
            drawings.push(DrawingSpec { robot_name: "bot".into(), strokes: world });
        }
        drawings
    }

    #[test]
    fn origin_on_empty_region_packs_into_the_corner() {
        let p = place(&request(Strategy::Origin, vec![])).unwrap().unwrap();
        assert_eq!(p.angle_deg, 0.0);
        assert_eq!(p.attempts, 1);
        // Mask's top-left at cell (0, 0): first ink is `pad` cells in.
        let pad = PlacementConfig::default().half_width_cells() + 1;
        assert!((p.anchor_x - pad as f64 * 5.0).abs() < 1e-9);
        assert!((p.anchor_y - pad as f64 * 5.0).abs() < 1e-9);
    }

    #[test]
    fn same_seed_same_answer() {
        let a = place(&request(Strategy::Scatter, vec![])).unwrap();
        let b = place(&request(Strategy::Scatter, vec![])).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn placed_drawings_never_come_within_the_buffer() {
        for strategy in [Strategy::Origin, Strategy::Scatter] {
            let ds = fill(strategy, 12);
            assert!(ds.len() >= 4, "{strategy:?} placed only {}", ds.len());
            for (i, a) in ds.iter().enumerate() {
                for b in &ds[i + 1..] {
                    let gap = min_distance(&a.strokes, &b.strokes);
                    assert!(gap >= 25.0, "{strategy:?}: gap {gap:.1}mm < buffer");
                }
                let (x0, y0, x1, y1) = bounds(&a.strokes).unwrap();
                assert!(x0 >= 0.0 && y0 >= 0.0 && x1 <= 1000.0 && y1 <= 1000.0);
            }
        }
    }

    #[test]
    fn shrinks_when_full_size_does_not_fit() {
        let ds = fill(Strategy::Origin, 30);
        let mut req = request(Strategy::Origin, ds);
        req.job.commands = square(100.0);
        if let Some(p) = place(&req).unwrap() {
            assert!(p.scale_fraction < 1.0 && p.attempts > 1);
        }
    }

    #[test]
    fn rebuilt_grid_lines_up_with_the_search_grid() {
        // Place once, rebuild from world strokes, and check the stamped mask
        // matches what the search reserved (the Python add_drawings bug).
        let ds = fill(Strategy::Origin, 1);
        let canvas = request(Strategy::Origin, vec![]).canvas;
        let rebuilt = build_grid(&canvas, 0, &ds);
        let (r0, c0, _, _) = rebuilt.bbox().unwrap();
        // Origin placement puts the mask at (0, 0); the ink mask's first set
        // cell sits at row/col 1 (the outer pad ring is empty).
        assert!(r0 <= 1 && c0 <= 1, "rebuilt mask starts at ({r0}, {c0})");
    }

    #[test]
    fn live_neighbour_blocks_its_side() {
        let mut req = request(Strategy::Origin, vec![]);
        req.canvas.regions.push(RegionSpec {
            id: "r2".into(),
            x: 0.0,
            y: 0.0,
            width: 1000.0,
            height: 1000.0,
            robot: Some("other".into()),
            config: PlacementConfig::default(),
        });
        // The other robot is drawing across the top-left corner.
        req.active.insert("other".into(), Some(vec![vec![(0.0, 100.0), (300.0, 100.0)]]));
        let p = place(&req).unwrap().unwrap();
        let ink = local_strokes(&req.job.commands, p.scale, p.angle_deg);
        let world: Vec<Stroke> =
            ink.into_iter().map(|s| s.into_iter().map(|(x, y)| (x + p.anchor_x, y + p.anchor_y)).collect()).collect();
        let gap = min_distance(&world, &[vec![(0.0, 100.0), (300.0, 100.0)]]);
        assert!(gap > RobotBody::default().min_overhang_mm(), "pen within {gap:.1}mm of a live robot");
    }

    /// Scatter `n` drawings (100mm squares at 150mm) with `k` candidates and the
    /// given edge weight; returns each drawing's gap to the ones before it.
    fn scatter_gaps(k: u32, edge_weight: f64, seed: u64, n: usize) -> (Vec<DrawingSpec>, Vec<f64>) {
        let mut drawings: Vec<DrawingSpec> = Vec::new();
        let mut gaps = Vec::new();
        for i in 0..n {
            let mut req = request(Strategy::Scatter, drawings.clone());
            let cfg = &mut req.canvas.regions[0].config;
            cfg.scatter_candidates = k;
            cfg.edge_weight = edge_weight;
            cfg.target_footprint_mm = 150.0;
            req.seed = seed * 1000 + i as u64;
            let Some(p) = place(&req).unwrap() else { break };
            let world: Vec<Stroke> = local_strokes(&req.job.commands, p.scale, p.angle_deg)
                .into_iter()
                .map(|s| s.into_iter().map(|(x, y)| (x + p.anchor_x, y + p.anchor_y)).collect())
                .collect();
            if let Some(gap) = drawings.iter().map(|d| min_distance(&d.strokes, &world)).reduce(f64::min) {
                gaps.push(gap);
            }
            drawings.push(DrawingSpec { robot_name: "bot".into(), strokes: world });
        }
        (drawings, gaps)
    }

    fn median(mut xs: Vec<f64>) -> f64 {
        xs.sort_by(f64::total_cmp);
        xs[xs.len() / 2]
    }

    #[test]
    fn best_candidate_scatter_leaves_more_room() {
        // At the default edge weight. (Measured: median gap ~186mm vs ~128mm.)
        let w = PlacementConfig::default().edge_weight;
        let (mut one, mut eight) = (Vec::new(), Vec::new());
        for seed in 0..12 {
            one.extend(scatter_gaps(1, w, seed, 6).1);
            eight.extend(scatter_gaps(8, w, seed, 6).1);
        }
        let (m1, m8) = (median(one), median(eight));
        assert!(m8 > m1 * 1.3, "median gap {m8:.0}mm with 8 candidates vs {m1:.0}mm with 1");
    }

    #[test]
    fn default_edge_weight_keeps_drawings_off_the_edges() {
        // How many drawings land within 40mm of an edge. (Measured: ~15% with a
        // plain pick, ~3% with 8 candidates at the default weight.)
        let near_edge = |k: u32| {
            let (mut near, mut total) = (0, 0);
            for seed in 0..12 {
                for d in scatter_gaps(k, PlacementConfig::default().edge_weight, seed, 6).0 {
                    let (x0, y0, x1, y1) = bounds(&d.strokes).unwrap();
                    near += (x0.min(y0).min(1000.0 - x1).min(1000.0 - y1) < 40.0) as usize;
                    total += 1;
                }
            }
            near as f64 / total as f64
        };
        let (one, eight) = (near_edge(1), near_edge(8));
        assert!(eight < one * 0.5, "{:.0}% near an edge with 8 candidates vs {:.0}% with 1", eight * 100.0, one * 100.0);
    }

    #[test]
    fn best_candidate_scatter_still_never_crowds() {
        for seed in 0..4 {
            let (drawings, gaps) = scatter_gaps(8, 0.25, seed, 12);
            assert!(drawings.len() >= 6, "placed only {}", drawings.len());
            assert!(gaps.iter().all(|&g| g >= 25.0), "a gap under the 25mm buffer: {gaps:?}");
        }
    }

    #[test]
    fn best_candidate_scatter_is_deterministic() {
        let a = scatter_gaps(8, 0.5, 3, 5).0;
        let b = scatter_gaps(8, 0.5, 3, 5).0;
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(x.strokes, y.strokes);
        }
    }

    #[test]
    fn edge_weight_decides_whether_edges_repel() {
        // On an empty region the only obstacles are the edges: counting them
        // pulls the first drawing toward the middle; ignoring them doesn't.
        let centre_dist = |w: f64| {
            let mut total = 0.0;
            for seed in 0..10 {
                let d = &scatter_gaps(8, w, seed, 1).0[0];
                let (x0, y0, x1, y1) = bounds(&d.strokes).unwrap();
                total += ((x0 + x1) / 2.0 - 500.0).hypot((y0 + y1) / 2.0 - 500.0);
            }
            total / 10.0
        };
        let (counted, ignored) = (centre_dist(1.0), centre_dist(0.0));
        assert!(counted < ignored * 0.6, "{counted:.0}mm from centre with edges counted vs {ignored:.0}mm ignored");
    }

    #[test]
    fn clearance_map_measures_ink_and_edges() {
        let mut ink = Mask::new(10, 20);
        ink.set(5, 15);
        let m = clearance_map(&ink, 0.0); // ink only
        assert_eq!(m[5 * 20 + 15], 0.0);
        assert!((m[5 * 20 + 12] - 3.0).abs() < 1e-12);
        assert!((m[1 * 20 + 12] - 5.0).abs() < 1e-12); // 4 rows, 3 cols: 3-4-5
        let m = clearance_map(&ink, 1.0); // edges count too
        assert!((m[5 * 20] - 0.5).abs() < 1e-12); // left edge, half a cell away
        let m = clearance_map(&ink, 0.5); // edges count for half
        assert!((m[5 * 20] - 1.0).abs() < 1e-12);
        let m = clearance_map(&Mask::new(4, 4), 0.0); // no ink, no edges: unbounded
        assert!(m.iter().all(|&v| v > 1e9));
    }

    fn min_distance(a: &[Stroke], b: &[Stroke]) -> f64 {
        let seg = |p: Point, a: Point, b: Point| {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = dx * dx + dy * dy;
            let t = if l2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
            (a.0 + t * dx - p.0).hypot(a.1 + t * dy - p.1)
        };
        let mut best = f64::INFINITY;
        for (x, y) in [(a, b), (b, a)] {
            for s in x {
                for &p in s {
                    for t in y {
                        for w in t.windows(2) {
                            best = best.min(seg(p, w[0], w[1]));
                        }
                    }
                }
            }
        }
        best
    }
}

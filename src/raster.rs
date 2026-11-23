//! Boolean grids and the rasterizers that fill them.
//!
//! Pixel convention: cell (row i, col j) covers `[j, j+1) x [i, i+1)` in cell
//! units, so its centre is `(j + 0.5, i + 0.5)`. Every fill here samples centres.

use crate::geometry::{body_sweep, bounds};
use crate::types::{Point, RobotBody, Stroke};

/// Ink reaches this far past `half_width_cells` from the centreline. Tuned so
/// footprints cover essentially everything PIL's `width=2*half+1` round-joined
/// line did (about 3% fewer cells missed than any thinner radius), without the
/// 4x supersampling the OpenCV approximation needed.
pub const INK_RADIUS_EXTRA: f64 = 0.25;

/// Row-major 0/1 grid.
#[derive(Clone, Debug, PartialEq)]
pub struct Mask {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<u8>,
}

impl Mask {
    pub fn new(rows: usize, cols: usize) -> Self {
        Mask { rows, cols, data: vec![0; rows * cols] }
    }
    #[inline]
    pub fn get(&self, r: usize, c: usize) -> bool {
        self.data[r * self.cols + c] != 0
    }
    #[inline]
    pub fn set(&mut self, r: usize, c: usize) {
        self.data[r * self.cols + c] = 1;
    }
    pub fn row(&self, r: usize) -> &[u8] {
        &self.data[r * self.cols..(r + 1) * self.cols]
    }
    pub fn any(&self) -> bool {
        self.data.iter().any(|&v| v != 0)
    }
    pub fn count(&self) -> usize {
        self.data.iter().filter(|&&v| v != 0).count()
    }
    pub fn or_assign(&mut self, other: &Mask) {
        debug_assert_eq!((self.rows, self.cols), (other.rows, other.cols));
        for (a, &b) in self.data.iter_mut().zip(&other.data) {
            *a |= b;
        }
    }
    /// OR `src` into self with src's (0, 0) at (top, left); clipped to bounds.
    pub fn stamp(&mut self, src: &Mask, top: i64, left: i64) {
        let r0 = top.max(0);
        let c0 = left.max(0);
        let r1 = (top + src.rows as i64).min(self.rows as i64);
        let c1 = (left + src.cols as i64).min(self.cols as i64);
        if r1 <= r0 || c1 <= c0 {
            return;
        }
        for r in r0..r1 {
            let sr = (r - top) as usize;
            let dst = &mut self.data[r as usize * self.cols..][c0 as usize..c1 as usize];
            let s = &src.row(sr)[(c0 - left) as usize..(c1 - left) as usize];
            for (a, &b) in dst.iter_mut().zip(s) {
                *a |= b;
            }
        }
    }
    /// Sub-grid `[r0, r0+rows) x [c0, c0+cols)`; must be in bounds.
    pub fn crop(&self, r0: usize, c0: usize, rows: usize, cols: usize) -> Mask {
        let mut out = Mask::new(rows, cols);
        for r in 0..rows {
            out.data[r * cols..(r + 1) * cols]
                .copy_from_slice(&self.data[(r0 + r) * self.cols + c0..][..cols]);
        }
        out
    }
    /// (first row, first col, last row, last col) of set cells.
    pub fn bbox(&self) -> Option<(usize, usize, usize, usize)> {
        let (mut r0, mut c0, mut r1, mut c1) = (usize::MAX, usize::MAX, 0, 0);
        for r in 0..self.rows {
            for (c, &v) in self.row(r).iter().enumerate() {
                if v != 0 {
                    r0 = r0.min(r);
                    r1 = r1.max(r);
                    c0 = c0.min(c);
                    c1 = c1.max(c);
                }
            }
        }
        (r0 != usize::MAX).then_some((r0, c0, r1, c1))
    }

    /// Fill every cell whose centre is within `r` of segment a-b (cell units).
    pub fn fill_capsule(&mut self, a: Point, b: Point, r: f64) {
        let r2 = r * r;
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let Some((i0, i1)) = self.span(a.1.min(b.1) - r, a.1.max(b.1) + r, self.rows) else { return };
        let Some((j0, j1)) = self.span(a.0.min(b.0) - r, a.0.max(b.0) + r, self.cols) else { return };
        for i in i0..=i1 {
            let py = i as f64 + 0.5;
            let row = &mut self.data[i * self.cols..(i + 1) * self.cols];
            for j in j0..=j1 {
                if row[j] != 0 {
                    continue;
                }
                let px = j as f64 + 0.5;
                let t = if len2 > 0.0 { (((px - a.0) * dx + (py - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
                let (ex, ey) = (a.0 + t * dx - px, a.1 + t * dy - py);
                if ex * ex + ey * ey <= r2 {
                    row[j] = 1;
                }
            }
        }
    }

    /// Fill every cell whose centre lies inside the convex polygon `pts`.
    pub fn fill_convex(&mut self, pts: &[Point]) {
        let ymin = pts.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let ymax = pts.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        let Some((i0, i1)) = self.span(ymin, ymax, self.rows) else { return };
        for i in i0..=i1 {
            let yc = i as f64 + 0.5;
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for k in 0..pts.len() {
                let (p, q) = (pts[k], pts[(k + 1) % pts.len()]);
                let (ya, yb) = (p.1.min(q.1), p.1.max(q.1));
                if yc < ya || yc > yb {
                    continue;
                }
                if p.1 == q.1 {
                    lo = lo.min(p.0.min(q.0));
                    hi = hi.max(p.0.max(q.0));
                } else {
                    let x = p.0 + (yc - p.1) * (q.0 - p.0) / (q.1 - p.1);
                    lo = lo.min(x);
                    hi = hi.max(x);
                }
            }
            self.fill_row_span(i, lo, hi);
        }
    }

    /// Fill every cell whose centre is within `r` of (cx, cy).
    pub fn fill_disc(&mut self, cx: f64, cy: f64, r: f64) {
        let Some((i0, i1)) = self.span(cy - r, cy + r, self.rows) else { return };
        for i in i0..=i1 {
            let dy = i as f64 + 0.5 - cy;
            let h = r * r - dy * dy;
            if h < 0.0 {
                continue;
            }
            let dx = h.sqrt();
            self.fill_row_span(i, cx - dx, cx + dx);
        }
    }

    /// Cells j with centre in [lo, hi] on row i.
    fn fill_row_span(&mut self, i: usize, lo: f64, hi: f64) {
        if !(lo <= hi) {
            return;
        }
        let Some((j0, j1)) = self.span(lo, hi, self.cols) else { return };
        let row = &mut self.data[i * self.cols..(i + 1) * self.cols];
        for v in &mut row[j0..=j1] {
            *v = 1;
        }
    }

    /// Index range of cells whose centres fall in [lo, hi], clipped to [0, n).
    fn span(&self, lo: f64, hi: f64, n: usize) -> Option<(usize, usize)> {
        let a = (lo - 0.5).ceil().max(0.0);
        let b = (hi - 0.5).floor().min(n as f64 - 1.0);
        (a <= b).then(|| (a as usize, b as usize))
    }
}

/// A drawing's ink rasterized in its own frame.
#[derive(Clone, Debug)]
pub struct Footprint {
    pub mask: Mask,
    /// Local mm of the ink's bounding-box minimum; mask cell (pad, pad) holds it.
    pub min_x: f64,
    pub min_y: f64,
    pub pad: i64,
    pub cell_mm: f64,
}

impl Footprint {
    /// Local mm -> (col, row) within the mask.
    pub fn to_px(&self, p: Point) -> (f64, f64) {
        (
            (p.0 - self.min_x) / self.cell_mm + self.pad as f64,
            (p.1 - self.min_y) / self.cell_mm + self.pad as f64,
        )
    }
}

/// Rasterize strokes into a footprint whose ink is `half_width_cells +
/// INK_RADIUS_EXTRA` cells thick on each side of the centreline (canvas.py
/// `rasterize`, same frame and mask size). `None` if there are no points.
pub fn rasterize(strokes: &[Stroke], cell_mm: f64, half_width_cells: i64) -> Option<Footprint> {
    let (min_x, min_y, max_x, max_y) = bounds(strokes)?;
    let pad = half_width_cells + 1;
    let cols = (((max_x - min_x) / cell_mm).ceil() as i64 + 2 * pad + 1) as usize;
    let rows = (((max_y - min_y) / cell_mm).ceil() as i64 + 2 * pad + 1) as usize;
    let mut mask = Mask::new(rows, cols);
    let fp = Footprint { mask: Mask::new(0, 0), min_x, min_y, pad, cell_mm };
    let r = half_width_cells as f64 + INK_RADIUS_EXTRA;
    for st in strokes {
        match st.len() {
            0 => {}
            1 => {
                let p = fp.to_px(st[0]);
                mask.fill_capsule(p, p, r);
            }
            _ => {
                for w in st.windows(2) {
                    mask.fill_capsule(fp.to_px(w[0]), fp.to_px(w[1]), r);
                }
            }
        }
    }
    Some(Footprint { mask, ..fp })
}

/// The chassis swept along a footprint's ink, in the same frame but with a
/// wider pad (canvas.py `FootprintCache.body_at`).
#[derive(Clone, Debug)]
pub struct BodySweep {
    pub mask: Mask,
    /// The sweep's (0, 0) sits this many cells above/left of the ink mask's.
    pub delta: i64,
}

pub fn body_sweep_mask(rotated: &[Stroke], fp: &Footprint, body: &RobotBody, margin: f64) -> BodySweep {
    let cell = fp.cell_mm;
    let (min_x, min_y, max_x, max_y) = bounds(rotated).expect("footprint has points");
    let pad = ((body.reach_mm() + margin) / cell).ceil() as i64 + 1;
    let cols = (((max_x - min_x) / cell).ceil() as i64 + 2 * pad + 1) as usize;
    let rows = (((max_y - min_y) / cell).ceil() as i64 + 2 * pad + 1) as usize;
    let mut mask = Mask::new(rows, cols);
    let to_px = |p: Point| ((p.0 - min_x) / cell + pad as f64, (p.1 - min_y) / cell + pad as f64);
    for st in rotated {
        for w in st.windows(2) {
            if let Some(q) = body_sweep(w[0], w[1], body, margin) {
                mask.fill_convex(&q.map(to_px));
            }
        }
        if st.len() == 1 {
            let (cx, cy) = to_px(st[0]);
            mask.fill_disc(cx, cy, (body.reach_mm() + margin) / cell);
        }
    }
    BodySweep { mask, delta: pad - fp.pad }
}

/// Half-widths of OpenCV's `MORPH_ELLIPSE` structuring element of size
/// (2b+1)x(2b+1), one per row from dy = -b to b.
pub fn ellipse_kernel(b: usize) -> Vec<usize> {
    let r = b as f64;
    (0..=2 * b)
        .map(|i| {
            let dy = i as f64 - r;
            if b == 0 {
                return 0;
            }
            // cv::saturate_cast<int>(double) rounds half to even.
            (r * ((r * r - dy * dy) / (r * r)).sqrt()).round_ties_even() as usize
        })
        .collect()
}

/// Dilate by the `MORPH_ELLIPSE` kernel of radius `b`, treating everything
/// outside the grid as empty (canvas.py `compute_occupancy`'s pad-dilate-crop).
pub fn dilate_ellipse(src: &Mask, b: usize) -> Mask {
    if b == 0 {
        return src.clone();
    }
    let k = ellipse_kernel(b);
    let mut out = Mask::new(src.rows, src.cols);
    let (rows, cols) = (src.rows as i64, src.cols as i64);
    for y in 0..src.rows {
        let row = src.row(y);
        let mut j = 0;
        while j < src.cols {
            if row[j] == 0 {
                j += 1;
                continue;
            }
            let start = j;
            while j < src.cols && row[j] != 0 {
                j += 1;
            }
            let end = j - 1;
            for (ki, &dx) in k.iter().enumerate() {
                let ty = y as i64 + ki as i64 - b as i64;
                if ty < 0 || ty >= rows {
                    continue;
                }
                let a = (start as i64 - dx as i64).max(0) as usize;
                let e = (end as i64 + dx as i64).min(cols - 1) as usize;
                let base = ty as usize * src.cols;
                out.data[base + a..=base + e].fill(1);
            }
        }
    }
    out
}

/// Squared value of [`squared_distance_map`] where no cell is set.
pub const FAR: f64 = 1e20;

/// Squared Euclidean distance (cells, centre to centre) from every cell to the
/// nearest set cell, row-major; [`FAR`] everywhere if none is set. Exact, via
/// Felzenszwalb–Huttenlocher.
pub fn squared_distance_map(src: &Mask) -> Vec<f64> {
    let (rows, cols) = (src.rows, src.cols);
    // Column pass: squared vertical distance to the nearest set cell.
    let mut g = vec![FAR; rows * cols];
    let mut f = vec![0.0; rows.max(cols)];
    let mut d = vec![0.0; rows.max(cols)];
    let mut v = vec![0usize; rows.max(cols)];
    let mut z = vec![0.0; rows.max(cols) + 1];
    for c in 0..cols {
        for r in 0..rows {
            f[r] = if src.get(r, c) { 0.0 } else { FAR };
        }
        edt_1d(&f[..rows], &mut d[..rows], &mut v, &mut z);
        for r in 0..rows {
            g[r * cols + c] = d[r];
        }
    }
    // Row pass over the column result.
    for r in 0..rows {
        f[..cols].copy_from_slice(&g[r * cols..(r + 1) * cols]);
        edt_1d(&f[..cols], &mut d[..cols], &mut v, &mut z);
        g[r * cols..(r + 1) * cols].copy_from_slice(&d[..cols]);
    }
    g
}

/// Cells within Euclidean distance `radius` (cells, centre to centre) of any set
/// cell — what canvas.py gets from `cv2.distanceTransform(..., DIST_L2,
/// DIST_MASK_PRECISE) <= radius`.
pub fn within_distance(src: &Mask, radius: f64) -> Mask {
    let lim = radius * radius;
    let mut out = Mask::new(src.rows, src.cols);
    for (o, &d) in out.data.iter_mut().zip(&squared_distance_map(src)) {
        *o = (d <= lim) as u8;
    }
    out
}

/// 1-D squared distance transform of sampled function `f` (lower envelope of
/// parabolas).
fn edt_1d(f: &[f64], d: &mut [f64], v: &mut [usize], z: &mut [f64]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let mut k = 0usize;
    v[0] = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    for q in 1..n {
        loop {
            let p = v[k];
            let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * (q as f64 - p as f64));
            if s <= z[k] && k > 0 {
                k -= 1;
                continue;
            }
            if s <= z[k] {
                // k == 0: q replaces the only parabola.
                v[0] = q;
                z[0] = f64::NEG_INFINITY;
                z[1] = f64::INFINITY;
                break;
            }
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = f64::INFINITY;
            break;
        }
    }
    k = 0;
    for q in 0..n {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let p = v[k];
        let dq = q as f64 - p as f64;
        d[q] = dq * dq + f[p];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ellipse_kernel_matches_opencv() {
        // cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) rows:
        // 00100 / 11111 / 11111 / 11111 / 00100
        assert_eq!(ellipse_kernel(2), vec![0, 2, 2, 2, 0]);
        assert_eq!(ellipse_kernel(0), vec![0]);
    }

    #[test]
    fn dilate_single_cell() {
        let mut m = Mask::new(7, 7);
        m.set(3, 3);
        let d = dilate_ellipse(&m, 2);
        assert_eq!(d.count(), 1 + 5 + 5 + 5 + 1);
        assert!(d.get(1, 3) && !d.get(1, 2) && d.get(3, 1));
    }

    #[test]
    fn distance_transform_matches_brute_force() {
        let mut m = Mask::new(23, 31);
        for &(r, c) in &[(2, 3), (10, 20), (22, 0), (15, 15)] {
            m.set(r, c);
        }
        for radius in [0.0, 1.0, 2.5, 4.2, 9.9] {
            let fast = within_distance(&m, radius);
            for r in 0..m.rows {
                for c in 0..m.cols {
                    let near = (0..m.rows).any(|a| {
                        (0..m.cols).any(|b| {
                            m.get(a, b) && {
                                let (dr, dc) = (a as f64 - r as f64, b as f64 - c as f64);
                                dr * dr + dc * dc <= radius * radius
                            }
                        })
                    });
                    assert_eq!(fast.get(r, c), near, "r={r} c={c} radius={radius}");
                }
            }
        }
    }

    #[test]
    fn capsule_covers_centreline_cells() {
        let mut m = Mask::new(5, 10);
        m.fill_capsule((1.0, 2.5), (9.0, 2.5), 0.6);
        assert!((1..9).all(|j| m.get(2, j)));
        assert!(!m.get(0, 5) && !m.get(4, 5));
    }

    #[test]
    fn convex_fill_square() {
        let mut m = Mask::new(6, 6);
        m.fill_convex(&[(1.0, 1.0), (4.0, 1.0), (4.0, 4.0), (1.0, 4.0)]);
        assert_eq!(m.count(), 9);
    }
}

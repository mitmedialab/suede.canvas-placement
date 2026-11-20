//! Free offsets via FFT cross-correlation (canvas.py `_free_offsets`).
//!
//! `corr[t, l]` = occupied cells a mask would overlap with its top-left at
//! `(t, l)`; `corr = irfft2(FFT(grid) * conj(FFT(mask)))`. Counts are integers,
//! so `< 0.5` is exact "free". Stays in f64: the DC term reaches ~1e8.

use crate::raster::Mask;
use realfft::{ComplexToReal, RealFftPlanner, RealToComplex};
use rustfft::num_complex::Complex64;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Smallest 2·3·5-smooth length >= n (scipy `next_fast_len(n, real=True)`).
pub fn next_fast_len(n: usize) -> usize {
    let mut m = n.max(1);
    loop {
        let mut k = m;
        for p in [2, 3, 5] {
            while k % p == 0 {
                k /= p;
            }
        }
        if k == 1 {
            return m;
        }
        m += 1;
    }
}

/// A grid's spectrum plus the plans to correlate masks against it.
pub struct Correlator {
    rows: usize,
    cols: usize,
    /// Transform size (5-smooth, >= grid size).
    fr: usize,
    fc: usize,
    /// Half-spectrum width, fc/2 + 1.
    hc: usize,
    r2c: Arc<dyn RealToComplex<f64>>,
    c2r: Arc<dyn ComplexToReal<f64>>,
    col_fwd: Arc<dyn Fft<f64>>,
    col_inv: Arc<dyn Fft<f64>>,
    /// Column-major half-spectrum of the grid: `spec[k * fr + i]`.
    grid_spec: Vec<Complex64>,
}

impl Correlator {
    pub fn new(grid: &Mask) -> Self {
        let (rows, cols) = (grid.rows, grid.cols);
        let (fr, fc) = (next_fast_len(rows), next_fast_len(cols));
        let mut rp = RealFftPlanner::<f64>::new();
        let mut cp = FftPlanner::<f64>::new();
        let mut me = Correlator {
            rows,
            cols,
            fr,
            fc,
            hc: fc / 2 + 1,
            r2c: rp.plan_fft_forward(fc),
            c2r: rp.plan_fft_inverse(fc),
            col_fwd: cp.plan_fft_forward(fr),
            col_inv: cp.plan_fft_inverse(fr),
            grid_spec: Vec::new(),
        };
        me.grid_spec = me.forward(grid);
        me
    }

    /// 2-D real FFT of `src` zero-padded to (fr, fc), column-major half-spectrum.
    fn forward(&self, src: &Mask) -> Vec<Complex64> {
        let (fr, hc) = (self.fr, self.hc);
        let mut spec = vec![Complex64::new(0.0, 0.0); hc * fr];
        let mut inp = self.r2c.make_input_vec();
        let mut out = self.r2c.make_output_vec();
        // Rows past src.rows are zero and transform to zero: skip them.
        for i in 0..src.rows {
            inp.fill(0.0);
            for (j, &v) in src.row(i).iter().enumerate() {
                inp[j] = v as f64;
            }
            self.r2c.process(&mut inp, &mut out).expect("r2c sizes");
            for k in 0..hc {
                spec[k * fr + i] = out[k];
            }
        }
        for k in 0..hc {
            self.col_fwd.process(&mut spec[k * fr..(k + 1) * fr]);
        }
        spec
    }

    /// Collision-free top-left offsets for `mask`, shape
    /// `(rows - mh + 1, cols - mw + 1)`. Caller ensures the mask fits.
    pub fn free_offsets(&self, mask: &Mask) -> Mask {
        let (fr, hc) = (self.fr, self.hc);
        let vr = self.rows - mask.rows + 1;
        let vc = self.cols - mask.cols + 1;
        let mut spec = self.forward(mask);
        for (s, g) in spec.iter_mut().zip(&self.grid_spec) {
            *s = g * s.conj();
        }
        for k in 0..hc {
            self.col_inv.process(&mut spec[k * fr..(k + 1) * fr]);
        }
        let scale = 1.0 / (self.fr * self.fc) as f64;
        let mut row_spec = self.c2r.make_input_vec();
        let mut row_out = self.c2r.make_output_vec();
        let mut free = Mask::new(vr, vc);
        for i in 0..vr {
            for k in 0..hc {
                row_spec[k] = spec[k * fr + i];
            }
            // A real signal's DC (and even-length Nyquist) bins are real; drop
            // the rounding noise realfft would otherwise reject.
            row_spec[0].im = 0.0;
            if self.fc % 2 == 0 {
                row_spec[hc - 1].im = 0.0;
            }
            self.c2r.process(&mut row_spec, &mut row_out).expect("c2r sizes");
            for j in 0..vc {
                if row_out[j] * scale < 0.5 {
                    free.data[i * vc + j] = 1;
                }
            }
        }
        free
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fast_len() {
        assert_eq!(next_fast_len(667), 675);
        assert_eq!(next_fast_len(200), 200);
        assert_eq!(next_fast_len(7), 8);
        assert_eq!(next_fast_len(1), 1);
    }

    #[test]
    fn matches_brute_force() {
        // Pseudo-random grid and mask; odd sizes to exercise padding.
        let mut seed = 12345u64;
        let mut bit = |p: u64| {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 33) % 100 < p
        };
        let mut grid = Mask::new(37, 53);
        for v in grid.data.iter_mut() {
            *v = bit(8) as u8;
        }
        let mut mask = Mask::new(6, 9);
        for v in mask.data.iter_mut() {
            *v = bit(40) as u8;
        }
        let free = Correlator::new(&grid).free_offsets(&mask);
        for t in 0..free.rows {
            for l in 0..free.cols {
                let hit = (0..mask.rows).any(|r| (0..mask.cols).any(|c| mask.get(r, c) && grid.get(t + r, l + c)));
                assert_eq!(free.get(t, l), !hit, "t={t} l={l}");
            }
        }
    }
}

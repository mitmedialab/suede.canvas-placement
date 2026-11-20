//! Stateless drawing placement for doodlebot canvases.
//!
//! One call answers "where does this drawing go?": [`place`] takes the canvas
//! layout, every drawing already on it, what neighbouring robots are drawing
//! right now, and the job to place — and returns a pose, or `None` if it fits
//! nowhere even at the minimum scale. Nothing is kept between calls: the
//! region's occupancy grid is rebuilt from `drawings` each time, so the list of
//! placed drawings is the only source of truth. [`free_fractions`] reports how
//! full each region is, from the same rebuild.
//!
//! Port of the placement half of `release/canvas.py` plus `_place_scaled` from
//! `release/robots.py`. Differences from the Python engine:
//!
//! * Footprints are rasterized geometrically (a cell is ink iff its centre lies
//!   within `half + 0.25` cells of the stroke centreline) instead of through PIL's
//!   wide-line drawing. See [`raster::rasterize`].
//! * Rebuilding a region from saved drawings places each mask where its ink
//!   actually is; the Python `Region.add_drawings` shifted it `pad` cells.
//! * Scatter randomness comes from a seeded SplitMix64 ([`rng::Rng`]), not
//!   CPython's Mersenne Twister.

pub mod engine;
pub mod fftcorr;
pub mod geometry;
pub mod raster;
pub mod rng;
pub mod types;

mod clock;

#[cfg(feature = "python")]
mod python;

pub use engine::{free_fractions, occupancy, place};
pub use types::*;

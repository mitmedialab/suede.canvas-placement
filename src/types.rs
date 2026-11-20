//! Request/response types. Plain serde structs so the same shapes work from
//! Python (pythonize), JSON (the CLI) and, later, wasm.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// (x, y) in mm.
pub type Point = (f64, f64);
/// A contiguous pen-down polyline.
pub type Stroke = Vec<Point>;

/// One drawing command, matching robots.py's `{kind: line|spin|arc}` models.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Command {
    Line {
        distance: f64,
        #[serde(rename = "penDown")]
        pen_down: bool,
    },
    Spin {
        degrees: f64,
    },
    Arc {
        radius: f64,
        degrees: f64,
    },
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Strategy {
    /// Pack toward the region's (0, 0) corner; ties prefer the earlier angle.
    Origin,
    /// Uniformly random free pose.
    Scatter,
}

/// Per-region placement settings (canvas.py `PlacementConfig`).
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct PlacementConfig {
    pub cell_mm: f64,
    pub pen_mm: f64,
    pub clearance_mm: f64,
    pub search_step_cells: i64,
    pub angles_deg: Vec<f64>,
    pub strategy: Strategy,
    pub target_footprint_mm: f64,
    pub min_footprint_scale: f64,
}

impl Default for PlacementConfig {
    fn default() -> Self {
        PlacementConfig {
            cell_mm: 5.0,
            pen_mm: 5.0,
            clearance_mm: 8.0,
            search_step_cells: 2,
            angles_deg: (0..24).map(|i| (i * 15) as f64).collect(),
            strategy: Strategy::Scatter,
            target_footprint_mm: 200.0,
            min_footprint_scale: 0.4,
        }
    }
}

impl PlacementConfig {
    /// Half the inked line width in cells: pen radius plus clearance.
    pub fn half_width_cells(&self) -> i64 {
        ((self.pen_mm / 2.0 + self.clearance_mm) / self.cell_mm).ceil() as i64
    }
}

/// A robot's chassis relative to its pen (canvas.py `RobotBody`).
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct RobotBody {
    pub length_mm: f64,
    pub width_mm: f64,
    pub pen_from_nose_mm: f64,
}

impl Default for RobotBody {
    fn default() -> Self {
        RobotBody { length_mm: 226.0, width_mm: 149.0, pen_from_nose_mm: 76.0 }
    }
}

impl RobotBody {
    pub fn pen_from_tail_mm(&self) -> f64 {
        self.length_mm - self.pen_from_nose_mm
    }
    /// Farthest any part of the body can be from the pen, over all headings.
    pub fn reach_mm(&self) -> f64 {
        let along = self.pen_from_nose_mm.max(self.pen_from_tail_mm());
        along.hypot(self.width_mm / 2.0)
    }
    /// Nearest the body's edge can be to the pen, over all headings.
    pub fn min_overhang_mm(&self) -> f64 {
        self.pen_from_nose_mm.min(self.pen_from_tail_mm()).min(self.width_mm / 2.0)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RegionSpec {
    pub id: String,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    #[serde(default)]
    pub robot: Option<String>,
    #[serde(default)]
    pub config: PlacementConfig,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CanvasSpec {
    #[serde(default)]
    pub width: f64,
    #[serde(default)]
    pub height: f64,
    /// mm of clearance between separate drawings.
    #[serde(default)]
    pub general_buffer: f64,
    pub regions: Vec<RegionSpec>,
}

/// A drawing already on the canvas. Only what placement needs: whose region it
/// belongs to, and its ink in world mm.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DrawingSpec {
    pub robot_name: String,
    pub strokes: Vec<Stroke>,
}

/// The drawing to place, lead-in already stripped (robots.py `_QueuedJob`).
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Job {
    pub commands: Vec<Command>,
    #[serde(default)]
    pub heading0: f64,
    /// Longest side of the unscaled ink's bounding box, mm.
    pub native_span: f64,
}

fn default_scale_tol() -> f64 {
    0.02
}
fn default_max_iters() -> u32 {
    12
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct PlaceRequest {
    pub canvas: CanvasSpec,
    /// Every drawing on this canvas. Each is stamped into its robot's region.
    #[serde(default)]
    pub drawings: Vec<DrawingSpec>,
    /// Robot name -> strokes (world mm) it is drawing right now, or None.
    #[serde(default)]
    pub active: HashMap<String, Option<Vec<Stroke>>>,
    #[serde(default)]
    pub robot_body: RobotBody,
    /// Region to place into.
    pub region_id: String,
    pub job: Job,
    /// Drives the scatter strategy; same seed + same input = same result.
    #[serde(default)]
    pub seed: u64,
    /// Time allowed for the shrink-to-fit search. The full-size attempt always
    /// runs; `None` means no limit.
    #[serde(default)]
    pub budget_ms: Option<f64>,
    #[serde(default = "default_scale_tol")]
    pub scale_tol: f64,
    #[serde(default = "default_max_iters")]
    pub max_iters: u32,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
pub struct PlaceResult {
    /// Rotation applied on top of `heading0`, degrees.
    pub angle_deg: f64,
    /// Where the first ink point lands, world mm.
    pub anchor_x: f64,
    pub anchor_y: f64,
    /// Absolute scale to apply to the job's commands (distance, radius).
    pub scale: f64,
    /// `scale` as a fraction of the region's target size (1.0 = full size).
    pub scale_fraction: f64,
    /// Placement searches run (1 = fitted at full size).
    pub attempts: u32,
    /// The region's free fraction once this drawing is added.
    pub free_fraction_after: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FreeFractionsRequest {
    pub canvas: CanvasSpec,
    #[serde(default)]
    pub drawings: Vec<DrawingSpec>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct OccupancyRequest {
    pub canvas: CanvasSpec,
    #[serde(default)]
    pub drawings: Vec<DrawingSpec>,
    #[serde(default)]
    pub active: HashMap<String, Option<Vec<Stroke>>>,
    #[serde(default)]
    pub robot_body: RobotBody,
    pub region_id: String,
}

/// One region's grids, each `rows * cols` bytes, row-major, 0 or 1.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct OccupancyLayers {
    pub rows: usize,
    pub cols: usize,
    /// Committed ink.
    #[serde(with = "serde_bytes")]
    pub ink: Vec<u8>,
    /// Ink grown by the general buffer.
    #[serde(with = "serde_bytes")]
    pub buffered: Vec<u8>,
    /// Live neighbours' bodies grown by the broad-phase margin.
    #[serde(with = "serde_bytes")]
    pub active: Vec<u8>,
    /// Live neighbours' bodies, ungrown.
    #[serde(with = "serde_bytes")]
    pub bodies: Vec<u8>,
    /// What the search tests against: `buffered | active`.
    #[serde(with = "serde_bytes")]
    pub keepout: Vec<u8>,
}

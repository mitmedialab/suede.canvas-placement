//! WebAssembly bindings (feature `wasm`, built by `build_wasm.sh`).
//!
//! The same three calls as the Python extension, with the same request and
//! result shapes (see `types.rs`), in two forms each: plain JS objects in and
//! out, or JSON strings in and out (`*_json`). Bad requests throw.
//!
//! Objects cross the boundary as JSON (`JSON.stringify` / `JSON.parse`), which
//! keeps the bindings free of extra dependencies; requests are small next to
//! the work a placement does.
//!
//! **Seeds:** a JS number holds integers exactly only up to 2^53 − 1
//! (`Number.MAX_SAFE_INTEGER`), so pass seeds no larger than that. A bigger one
//! is rounded on its way in and gives a different (still valid) placement than
//! the same seed would natively. Within that range, wasm and native builds give
//! the same placements (anchors can differ in the last bit or so: `sin`/`cos`
//! aren't bit-identical across platforms).

use crate::types::{FreeFractionsRequest, OccupancyRequest, PlaceRequest};
use serde::de::DeserializeOwned;
use serde::Serialize;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = "
export function to_json(v) { return JSON.stringify(v); }
export function from_json(s) { return JSON.parse(s); }
")]
extern "C" {
    fn to_json(v: &JsValue) -> String;
    fn from_json(s: &str) -> JsValue;
}

fn parse<T: DeserializeOwned>(json: &str) -> Result<T, JsError> {
    serde_json::from_str(json).map_err(|e| JsError::new(&format!("bad request: {e}")))
}

fn to_string<T: Serialize>(value: &T) -> Result<String, JsError> {
    serde_json::to_string(value).map_err(|e| JsError::new(&e.to_string()))
}

/// Where does this drawing go? Returns `{angle_deg, anchor_x, anchor_y, scale,
/// scale_fraction, attempts, free_fraction_after}`, or `null` if it fits nowhere.
#[wasm_bindgen]
pub fn place(request: JsValue) -> Result<JsValue, JsError> {
    Ok(from_json(&place_json(&to_json(&request))?))
}

/// `place`, with the request and result as JSON strings.
#[wasm_bindgen]
pub fn place_json(request: &str) -> Result<String, JsError> {
    let req: PlaceRequest = parse(request)?;
    to_string(&crate::engine::place(&req).map_err(|e| JsError::new(&e))?)
}

/// How empty is each region? `{canvas, drawings}` → `{region_id: fraction}`.
#[wasm_bindgen]
pub fn free_fractions(request: JsValue) -> Result<JsValue, JsError> {
    Ok(from_json(&free_fractions_json(&to_json(&request))?))
}

/// `free_fractions`, with the request and result as JSON strings.
#[wasm_bindgen]
pub fn free_fractions_json(request: &str) -> Result<String, JsError> {
    let req: FreeFractionsRequest = parse(request)?;
    to_string(&crate::engine::free_fractions(&req))
}

/// The grids a placement in `region_id` is searched against: `{rows, cols,
/// ink, buffered, active, bodies, keepout}`, each grid an array of
/// `rows * cols` 0/1 values, row-major.
#[wasm_bindgen]
pub fn occupancy(request: JsValue) -> Result<JsValue, JsError> {
    Ok(from_json(&occupancy_json(&to_json(&request))?))
}

/// `occupancy`, with the request and result as JSON strings.
#[wasm_bindgen]
pub fn occupancy_json(request: &str) -> Result<String, JsError> {
    let req: OccupancyRequest = parse(request)?;
    to_string(&crate::engine::occupancy(&req).map_err(|e| JsError::new(&e))?)
}

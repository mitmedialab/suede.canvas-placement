//! PyO3 bindings. Requests and results are plain dicts/lists (pythonize), in
//! the shapes of `types.rs`.

use crate::types::{FreeFractionsRequest, OccupancyRequest, PlaceRequest};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pythonize::{depythonize, pythonize};

fn bad<E: std::fmt::Display>(e: E) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// place(request: dict) -> dict | None
///
/// See `PlaceRequest` / `PlaceResult`. Runs without the GIL.
#[pyfunction]
fn place<'py>(py: Python<'py>, request: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let req: PlaceRequest = depythonize(request).map_err(bad)?;
    let res = py.allow_threads(|| crate::engine::place(&req)).map_err(bad)?;
    pythonize(py, &res).map_err(bad)
}

/// free_fractions(request: dict) -> dict[str, float]
///
/// `request` is `{"canvas": ..., "drawings": [...]}`.
#[pyfunction]
fn free_fractions<'py>(py: Python<'py>, request: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let req: FreeFractionsRequest = depythonize(request).map_err(bad)?;
    let res = py.allow_threads(|| crate::engine::free_fractions(&req));
    pythonize(py, &res).map_err(bad)
}

/// occupancy(request: dict) -> dict
///
/// One region's grids as `{"rows", "cols", "ink", "buffered", "active",
/// "bodies", "keepout"}`, each grid `rows * cols` bytes. See `OccupancyLayers`.
#[pyfunction]
fn occupancy<'py>(py: Python<'py>, request: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let req: OccupancyRequest = depythonize(request).map_err(bad)?;
    let res = py.allow_threads(|| crate::engine::occupancy(&req)).map_err(bad)?;
    pythonize(py, &res).map_err(bad)
}

#[pymodule]
fn canvas_placement(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(place, m)?)?;
    m.add_function(wrap_pyfunction!(free_fractions, m)?)?;
    m.add_function(wrap_pyfunction!(occupancy, m)?)?;
    Ok(())
}

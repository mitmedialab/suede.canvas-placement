//! Time budget for the shrink-to-fit search.

#[cfg(not(target_arch = "wasm32"))]
pub struct Deadline(Option<std::time::Instant>);

#[cfg(not(target_arch = "wasm32"))]
impl Deadline {
    pub fn after_ms(ms: Option<f64>) -> Self {
        Deadline(ms.map(|m| std::time::Instant::now() + std::time::Duration::from_secs_f64(m.max(0.0) / 1000.0)))
    }
    pub fn passed(&self) -> bool {
        self.0.is_some_and(|d| std::time::Instant::now() >= d)
    }
}

/// `std::time::Instant` panics on wasm32-unknown-unknown, so the wasm build
/// asks JavaScript for the time instead (`performance.now()`, or `Date.now()`
/// where that's missing).
#[cfg(all(target_arch = "wasm32", feature = "wasm"))]
pub struct Deadline(Option<f64>);

#[cfg(all(target_arch = "wasm32", feature = "wasm"))]
mod js {
    use wasm_bindgen::prelude::*;
    #[wasm_bindgen(inline_js = "export function now_ms() {
        return (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
    }")]
    extern "C" {
        pub fn now_ms() -> f64;
    }
}

#[cfg(all(target_arch = "wasm32", feature = "wasm"))]
impl Deadline {
    pub fn after_ms(ms: Option<f64>) -> Self {
        Deadline(ms.map(|m| js::now_ms() + m.max(0.0)))
    }
    pub fn passed(&self) -> bool {
        self.0.is_some_and(|d| js::now_ms() >= d)
    }
}

/// A wasm32 build without the `wasm` bindings has no clock: the budget is
/// ignored and the shrink-to-fit search runs to completion.
#[cfg(all(target_arch = "wasm32", not(feature = "wasm")))]
pub struct Deadline;

#[cfg(all(target_arch = "wasm32", not(feature = "wasm")))]
impl Deadline {
    pub fn after_ms(_ms: Option<f64>) -> Self {
        Deadline
    }
    pub fn passed(&self) -> bool {
        false
    }
}

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

/// `std::time::Instant` panics on wasm32-unknown-unknown. Until a wasm build
/// wires in a JS clock, the budget is ignored there.
#[cfg(target_arch = "wasm32")]
pub struct Deadline;

#[cfg(target_arch = "wasm32")]
impl Deadline {
    pub fn after_ms(_ms: Option<f64>) -> Self {
        Deadline
    }
    pub fn passed(&self) -> bool {
        false
    }
}

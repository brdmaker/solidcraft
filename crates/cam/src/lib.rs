//! SolidCraft CAM for 3-axis routers machining from the top.
//!
//! Toolpaths come from the exact solid: machining features are read from the B-rep faces
//! (planes, vertical cylinders and cones), so holes keep their true diameters and arcs stay arcs.
//! A finely triangulated copy of the part is used only to check the result by simulation.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

pub mod part;
pub mod region;

/// What went wrong while planning a job.
#[derive(Debug, thiserror::Error)]
pub enum CamError {
    #[error("geometry: {0}")]
    Geometry(String),
    #[error("{0}")]
    Invalid(String),
    #[error("not supported yet: {0}")]
    Unsupported(String),
    #[error(transparent)]
    Kernel(#[from] solidcraft_kernel::KernelError),
}

pub type Result<T> = std::result::Result<T, CamError>;

#[cfg(test)]
mod fixture_tests;
#[cfg(test)]
mod tests;

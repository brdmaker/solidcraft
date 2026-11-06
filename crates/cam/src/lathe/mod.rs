//! Lathe CAM: turning parts from bar on a two-axis lathe (X a diameter, Z along the spindle),
//! first for the Emco Compact 5 with its six-station turret. A part is read as its profile along
//! its axis; it is faced, drilled and bored, roughed and finished, grooved where the turning
//! tools can't reach, threaded and parted off, written as a LinuxCNC lathe program, checked by
//! sweeping the tools through the bar's half section and exported as a scene (models, tool data,
//! the program) for the machine's viewer.

pub mod job;
pub mod plan;
pub mod post;
pub mod profile;
pub mod scene;
pub mod sim;

#[cfg(test)]
mod tests;

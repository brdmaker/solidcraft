//! Lathe CAM: turning parts from bar on a two-axis lathe (X a diameter, Z along the spindle),
//! first for the Emco Compact 5 with its six-station turret and turning tools. A part is read
//! as its profile along its axis; it is faced, roughed and finished, written as a LinuxCNC lathe
//! program and checked by sweeping the tools' inserts through the bar's half section.

pub mod job;
pub mod plan;
pub mod post;
pub mod profile;
pub mod sim;

#[cfg(test)]
mod tests;

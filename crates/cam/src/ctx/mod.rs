//! Turn-mill CAM for a twin-spindle lathe with live tools (the CTX twin in brdmot): bar fed
//! through spindle 1, the part turned and parted off there with spindle 2 gripping it and
//! following, its back finished in spindle 2. Each side gets its own LinuxCNC program; the two
//! meet at the transfer subroutines the machine provides.

pub mod job;
pub mod live;
pub mod plan;
pub mod post;
pub mod sim;

#[cfg(test)]
mod tests;

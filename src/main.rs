//! **robot_simulator_rust**
//!
//! A simulator of an autonomous mobile robot using Extended Kalman Filter and Dynamic Window Approach.

extern crate robot_simulator_rust;

use std::env;
use std::process;

/// The entry point of this binary crate.
fn main() {
  if let Err(e) = robot_simulator_rust::run(env::args()) {
    eprintln!("Problem parsing arguments: {}", e);
    process::exit(1);
  }
}

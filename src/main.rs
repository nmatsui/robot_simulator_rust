extern crate robot_simulator_rust;

use std::env;
use std::process;

fn main() {
  if let Err(e) = robot_simulator_rust::run(env::args()) {
    eprintln!("Problem parsing arguments: {}", e);
    process::exit(1);
  }
}

//! **robot_simulator_rust**
//!
//! A simulator of an autonomous mobile robot using Extended Kalman Filter and Dynamic Window Approach.

pub mod agent;
pub mod filters;
pub mod planners;
pub mod models;
pub mod timers;
pub mod data;
pub mod utils;

use std::error::Error;
use std::f64::consts::PI;

use crate::data::{ Point, Pose};

/// **\[private\]** the initial pose of the robot
const INITIAL_POSE: Pose = Pose { x: 1.0, y: 0.0, theta: PI / 2.0 };

/// **\[private\]** the array of landmark points which will be observed from robot
const LANDMARKS: [Point; 8]  = [
  Point {x: 1.1, y:  1.1}, Point {x: 0.0, y:  1.1}, Point {x: -1.1, y:  1.1},
  Point {x: 1.1, y:  0.0},                          Point {x: -1.1, y:  0.0},
  Point {x: 1.1, y: -1.1}, Point {x: 0.0, y: -1.1}, Point {x: -1.1, y: -1.1},
];

/// The entry point of this library crate.
///
/// ## Arguments
/// * `args` - a command line argument that means the Agent to be used. This crate can receive the following arguments:
///     * circular
///     * square
///     * waypoints
///
/// ## Returns
/// This function returns nothing when the process finishs successfully, but returns Error trait object when the process raises errors.
pub fn run(args: std::env::Args) -> Result<(), Box<dyn Error>> {
  let agt = agent::create_agent(args, LANDMARKS.to_vec())?;
  let ekf = filters::kalman_filter::EKF::new(agt, INITIAL_POSE);
  timers::start(ekf)?;

  Ok(())
}
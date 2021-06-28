//! **\[private\]** The `waypoints_agent` module provides a agent which define a trajectory according to given waypoints.

use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::{Point, Observed};
use crate::models::robot;
use crate::utils;
use agent_derive::AgentDerive;

/// **\[private\]** The squared distance threshold to determine that the simulated robot is close to a waypoint
const DISTANCE_SQUARED_THRESHOLD: f64 = 0.01;
/// **\[private\]** The angular threshold to determine that the simulated robot's direction is the same of the waypoint's direction
const ANGLE_THRESHOLD: f64 = PI / 18.0;

/// **\[private\]** The magnification of the linear and angular accelarations when the simulated robot is close to a waypoint
const NEAR_ACC_MAGNIFICATION: f64 = 0.8;
/// **\[private\]** The magnification of the linear velocity when the simulated robot is close to a waypoint
const NEAR_LINEAR_MAGNIFICATION: f64 = 0.1;
/// **\[private\]** The magnification of the angular velocity when the simulated robot is close to a waypoint
const NEAR_ANGULAR_MAGNIFICATION: f64 = 0.8;

/// **\[private\]** The definision of waypoints (nalgebra::Vector3::new(x, y, theta))
const WAYPOINTS: [na::Vector3<f64>; 5] = [na::Vector3::new( 1.0,  0.5,  PI * 3.0 / 4.0),
                                          na::Vector3::new( 0.5,  1.0, -PI),
                                          na::Vector3::new(-0.5,  1.0, -PI / 2.0),
                                          na::Vector3::new(-0.5, -1.0,  0.0),
                                          na::Vector3::new( 1.0, -1.0,  PI / 2.0)];

/// **\[private\]** The index of [WAYPOINTS] which indicate current target waypoint
static mut CURRENT_IDX: usize = 0;

/// A struct which provides a ideal pose to move to the next waypoint when the simulated robot arrives a waypoint
#[derive(AgentDerive)]
pub struct WaypointsAgent {
  landmarks: Vec<Point>,
  actual: na::Vector3<f64>,
  observed: Vec<Observed>,
}

/// The implementation for Agent trait
impl Agent for WaypointsAgent {

  /// Get the ideal pose that moves on the circumference at a constant angular velocity
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  /// * `_` - elapsed time (sec) from the start of this simulation
  ///     *  Since the elapsed time is not used in this function, the argument name is defind as `_`
  ///
  /// ## Returns
  /// ideal pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  fn get_ideal(&self, current: &na::Vector3<f64>, _: f64) -> na::Vector3<f64> {
    let idx = match WAYPOINTS[current_idx()] {
      target if (target.fixed_rows::<2>(0) - current.fixed_rows::<2>(0)).norm_squared() < DISTANCE_SQUARED_THRESHOLD &&
                utils::normalize_angle(target[2] - current[2]).abs() < ANGLE_THRESHOLD => {
        next_idx()
      },
      _ => {
        current_idx()
      },
    };
    WAYPOINTS[idx]
  }

  /// Get the maximum accelaration values
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///
  /// ## Returns
  /// Tuple of (maximum linear accelaration, maximum angular accelaration)
  fn get_max_accelarations(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_ACC_MAGNIFICATION } else { 1.0 };
    (robot::MAX_LIN_ACC * m, robot::MAX_ANG_ACC * m)
  }

  /// Get the maximum and minimum linear velocity values
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///
  /// ## Returns
  /// Tuple of (maximum linear velocity, minimum linear velocity)
  fn get_linear_velocities(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_LINEAR_MAGNIFICATION } else { 1.0 };
    (robot::MAX_V * m, robot::MIN_V * m)
  }

  /// Get the maximum and minimum angular velocity values
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///
  /// ## Returns
  /// Tuple of (maximum angular velocity, minimum angular velocity)
  fn get_angular_velocities(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_ANGULAR_MAGNIFICATION } else { 1.0 };
    (robot::MAX_OMEGA * m, robot::MIN_OMEGA * m)
  }
}

/// **\[private\]** Get the current index of [WAYPOINTS]
fn current_idx() -> usize {
  unsafe {
    CURRENT_IDX
  }
}

/// **\[private\]** Get the next index of [WAYPOINTS]
fn next_idx() -> usize {
  unsafe {
    CURRENT_IDX = (CURRENT_IDX + 1) % WAYPOINTS.len();
    CURRENT_IDX
  }
}

/// **\[private\]** Returns `true` when the current posision(x, y) is close to the current target waypoint
fn check_dist(current: &na::Vector3<f64>) -> bool {
  (WAYPOINTS[current_idx()].fixed_rows::<2>(0) - current.fixed_rows::<2>(0)).norm_squared() < DISTANCE_SQUARED_THRESHOLD
}

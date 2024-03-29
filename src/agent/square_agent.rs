//! **\[private\]** The `square_agent` module provides a agent which define a square trajectory.

use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::{Point, Observed};
use crate::utils;
use agent_derive::AgentDerive;

/// **\[private\]** The linear velocity to calculate ideal pose
const INPUT_V: f64 = 0.3;
/// **\[private\]** The angular velocity to calculate ideal pose
const INPUT_OMEGA: f64 = 0.5;

/// A struct which provides a ideal pose that moves on the sides of a square at a constant velocity
#[derive(AgentDerive)]
pub struct SquareAgent {
  landmarks: Vec<Point>,
  actual: na::Vector3<f64>,
  observed: Vec<Observed>,
}

/// The implementation for Agent trait
impl Agent for SquareAgent {

  /// Get the ideal pose that moves on the circumference at a constant angular velocity
  ///
  /// ## Arguments
  /// * `_` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///     *  Since the current pose is not used in this function, the argument name is defind as `_`
  /// * `t` - elapsed time (sec) from the start of this simulation
  ///
  /// ## Returns
  /// ideal pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  fn get_ideal(&self, _: &na::Vector3<f64>, t: f64) ->  na::Vector3<f64> {

    let d0 = 0.0;
    let d1 = d0 + 1.0 / INPUT_V;
    let d2 = d1 + PI / 2.0 / INPUT_OMEGA;
    let d3 = d2 + 2.0 / INPUT_V;
    let d4 = d3 + PI / 2.0 / INPUT_OMEGA;
    let d5 = d4 + 2.0 / INPUT_V;
    let d6 = d5 + PI / 2.0 / INPUT_OMEGA;
    let d7 = d6 + 2.0 / INPUT_V;
    let d8 = d7 + PI / 2.0 / INPUT_OMEGA;
    let d9 = d8 + 1.0 / INPUT_V;

    let delta = t % d9;

    let (x, y, theta) = match delta {
      dlt if d0 <= dlt && dlt < d1 => ( 1.0,                         0.0 + INPUT_V * (dlt - d0), PI * 1.0 / 2.0),
      dlt if d1 <= dlt && dlt < d2 => ( 1.0,                         1.0,                        PI * 1.0 / 2.0 + INPUT_OMEGA * (dlt - d1)),
      dlt if d2 <= dlt && dlt < d3 => ( 1.0 - INPUT_V * (dlt - d2),  1.0,                        PI * 2.0 / 2.0),
      dlt if d3 <= dlt && dlt < d4 => (-1.0,                         1.0,                        PI * 2.0 / 2.0 + INPUT_OMEGA * (dlt - d3)),
      dlt if d4 <= dlt && dlt < d5 => (-1.0,                         1.0 - INPUT_V * (dlt - d4), PI * 3.0 / 2.0),
      dlt if d5 <= dlt && dlt < d6 => (-1.0,                        -1.0,                        PI * 3.0 / 2.0 + INPUT_OMEGA * (dlt - d5)),
      dlt if d6 <= dlt && dlt < d7 => (-1.0 + INPUT_V * (dlt - d6), -1.0,                        PI * 0.0 / 2.0),
      dlt if d7 <= dlt && dlt < d8 => ( 1.0,                        -1.0,                        PI * 0.0 / 2.0 + INPUT_OMEGA * (dlt - d7)),
      dlt if d8 <= dlt && dlt < d9 => ( 1.0,                        -1.0 + INPUT_V * (dlt - d8), PI * 1.0 / 2.0),
      _ => panic!("assert error")
    };

    na::Vector3::new(x, y, utils::normalize_angle(theta))
  }
}

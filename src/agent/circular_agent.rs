//! **\[private\]** The `circular_agent` module provides a agent which define a circular trajectory.

use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::{Point, Observed};
use crate::utils;
use agent_derive::AgentDerive;

/// **\[private\]** The angular velocity to calculate ideal pose
const INPUT_OMEGA: f64 = 0.4;

/// A struct which provides a ideal pose that moves on the circumference at a constant angular velocity
#[derive(AgentDerive)]
pub struct CircularAgent {
  landmarks: Vec<Point>,
  actual: na::Vector3<f64>,
  observed: Vec<Observed>,
}

/// The implementation for Agent trait
impl Agent for CircularAgent {

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
    let angle = INPUT_OMEGA * t;
    let x = angle.cos();
    let y = angle.sin();
    let theta = utils::normalize_angle(angle + PI / 2.0);

    na::Vector3::new(x, y, theta)
  }
}

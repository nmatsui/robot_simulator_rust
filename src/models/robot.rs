//! The `robot` module provides an state equation of a simulated robot and the jacobian of the state equation

extern crate nalgebra as na;

use crate::utils;

/// The default value of maximum linear acceleration
pub const MAX_LIN_ACC: f64 = 2.5;
/// The default value of maximum angular acceleration
pub const MAX_ANG_ACC: f64 = 2.5;
/// The default value of maximum linear velocity
pub const MAX_V: f64 = 2.0;
/// The default value of minimum linear velocity
pub const MIN_V: f64 = -0.2;
/// The default value of maximum angular velocity
pub const MAX_OMEGA: f64 = 1.5;
/// The default value of minimum angular velocity
pub const MIN_OMEGA: f64 = -1.5;

/// Calculate the state equation of a simulated robot
///
/// ## Arguments
/// * `current` - the current pose of the simulated robot(x, y, theta)
/// * `input` - the input vector(linear velocity, angular velocity)
/// * `delta` - time delta to next tick
///
/// ## Returns
/// The pose(x, y, theta) of the simulated robot at the next tick
pub fn ideal_move(current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> na::Vector3<f64> {
  let a = current[2] + input[1] * delta / 2.0;
  let m = na::Matrix3x2::new(a.cos() * delta, 0.0,
                             a.sin() * delta, 0.0,
                             0.0,             delta);
  let mut next = current + m * input;
  next[2] = utils::normalize_angle(next[2]);
  next
}

/// Calculate the jacobian of the state equation
///
/// ## Arguments
/// * `current` - the current pose of the simulated robot(x, y, theta)
/// * `input` - the input vector(linear velocity, angular velocity)
/// * `delta` - time delta to next tick
///
/// ## Returns
/// The jacobian of the state equation
pub fn calc_f(current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> na::Matrix3<f64> {
  let a = current[2] + input[1] * delta / 2.0;

  na::Matrix3::new(1.0, 0.0, -1.0 * a.sin() * delta * input[0],
                   0.0, 1.0,  1.0 * a.cos() * delta * input[0],
                   0.0, 0.0,  1.0)
}

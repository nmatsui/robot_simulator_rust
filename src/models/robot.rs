extern crate nalgebra as na;

use crate::utils;

pub const MAX_LIN_ACC: f64 = 1.8;
pub const MAX_ANG_ACC: f64 = 2.0;
pub const MAX_V: f64 = 1.0;
pub const MIN_V: f64 = -0.1;
pub const MAX_OMEGA: f64 = 1.5;
pub const MIN_OMEGA: f64 = -1.5;

pub fn ideal_move(current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> na::Vector3<f64> {
  let a = current[2] + input[1] * delta / 2.0;
  let m = na::Matrix3x2::new(a.cos() * delta, 0.0,
                             a.sin() * delta, 0.0,
                             0.0,             delta);
  let mut next = current + m * input;
  next[2] = utils::normalize_angle(next[2]);
  next
}

pub fn calc_f(current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> na::Matrix3<f64> {
  let a = current[2] + input[1] * delta / 2.0;

  na::Matrix3::new(1.0, 0.0, -1.0 * a.sin() * delta * input[0],
                   0.0, 1.0,  1.0 * a.cos() * delta * input[0],
                   0.0, 0.0,  1.0)
}

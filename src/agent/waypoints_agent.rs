use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::{Point, Observed};
use crate::models::robot;
use crate::utils;

use agent_derive::AgentDerive;


const DISTANCE_SQUARED_THRESHOLD: f64 = 0.01;
const ANGLE_THRESHOLD: f64 = PI / 18.0;

const NEAR_ACC_MAGNIFICATION: f64 = 0.8;
const NEAR_LINEAR_MAGNIFICATION: f64 = 0.1;
const NEAR_ANGULAR_MAGNIFICATION: f64 = 0.8;

const WAYPOINTS: [na::Vector3<f64>; 5] = [na::Vector3::new( 1.0,  0.5,  PI * 3.0 / 4.0),
                                          na::Vector3::new( 0.5,  1.0, -PI),
                                          na::Vector3::new(-0.5,  1.0, -PI / 2.0),
                                          na::Vector3::new(-0.5, -1.0,  0.0),
                                          na::Vector3::new( 1.0, -1.0,  PI / 2.0)];

static mut CURRENT_IDX: usize = 0;

#[derive(AgentDerive)]
pub struct WaypointsAgent {
  landmarks: Vec<Point>,
  actual: na::Vector3<f64>,
  observed: Vec<Observed>,
}

impl Agent for WaypointsAgent {
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
  fn get_max_accelarations(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_ACC_MAGNIFICATION } else { 1.0 };
    (robot::MAX_LIN_ACC * m, robot::MAX_ANG_ACC * m)
  }
  fn get_linear_velocities(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_LINEAR_MAGNIFICATION } else { 1.0 };
    (robot::MAX_V * m, robot::MIN_V * m)
  }
  fn get_angular_velocities(&self, current: &na::Vector3<f64>) -> (f64, f64) {
    let m = if check_dist(current) { NEAR_ANGULAR_MAGNIFICATION } else { 1.0 };
    (robot::MAX_OMEGA * m, robot::MIN_OMEGA * m)
  }
}

fn current_idx() -> usize {
  unsafe {
    CURRENT_IDX
  }
}

fn next_idx() -> usize {
  unsafe {
    CURRENT_IDX = (CURRENT_IDX + 1) % WAYPOINTS.len();
    CURRENT_IDX
  }
}

fn check_dist(current: &na::Vector3<f64>) -> bool {
  (WAYPOINTS[current_idx()].fixed_rows::<2>(0) - current.fixed_rows::<2>(0)).norm_squared() < DISTANCE_SQUARED_THRESHOLD
}

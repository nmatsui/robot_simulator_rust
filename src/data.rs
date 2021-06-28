//! The `data` module provides some data structures which are shared by some modules

use serde::{Deserialize, Serialize};

extern crate nalgebra as na;

/// A struct which defines a point (x, y)
#[derive(Clone)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Point {
  pub x: f64,
  pub y: f64,
}

/// A struct which defines a pose (x, y, theta)
#[derive(Clone)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Pose {
  pub x: f64,
  pub y: f64,
  pub theta: f64,
}

impl Pose {
  /// Create [Pose] object from nalgebra::Vector3
  pub fn from_vector3(v: &na::Vector3<f64>) -> Pose {
    Pose { x: v[0], y: v[1], theta: v[2] }
  }
}

/// A struct which defines an observed values (distance, angle) of a landmark
#[derive(Clone)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Observed {
  pub landmark: Point,
  pub distance: f64,
  pub angle: f64,
}

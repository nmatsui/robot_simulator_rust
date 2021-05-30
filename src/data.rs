use serde::{Deserialize, Serialize};

extern crate nalgebra as na;

#[derive(Clone)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Point {
  pub x: f64,
  pub y: f64,
}

#[derive(Debug)]
#[derive(Serialize, Deserialize)]
pub struct Pose {
  pub x: f64,
  pub y: f64,
  pub theta: f64,
}

impl Pose {
  pub fn from_vector3(v: &na::Vector3<f64>) -> Pose {
    Pose { x: v[0], y: v[1], theta: v[2] }
  }
}
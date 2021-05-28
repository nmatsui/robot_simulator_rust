use serde::{Deserialize, Serialize};

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

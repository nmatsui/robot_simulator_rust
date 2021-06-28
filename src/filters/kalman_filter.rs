//! The `kalman_filters` module estimates a pose of a simulated robot in its state-space model by using Kalman Filter

use std::time::Instant;

extern crate nalgebra as na;

use crate::agent::Agent;
use crate::data::{Pose, Observed};
use crate::planners::dwa_wo_obstacle;
use crate::models::{robot, camera};

/// **\[private\]** The variance of process noise (assuming that each random variable in the state model is independent)
const Q: f64 = 0.01;
/// **\[private\]** The variance of observation noise (assuming that each random variable in the observation model is independent)
const R: f64 = 0.02;

/// A struct to estimate a pose of a simulated robot by using Extended Kalman Filter
#[derive(Debug)]
pub struct EKF {
  /// an agent instance of a robot to be estimated
  pub agent: Box<dyn Agent>,
  /// **\[private\]** the current estimated pose(x, y, theta)
  xhat: na::Vector3<f64>,
  /// **\[private\]** the current covariance matrix
  p: na::Matrix3<f64>,
  /// **\[private\]** the covariance matrix of process noise
  q: na::Matrix3<f64>,
  /// **\[private\]** the covariance matrix of observation noise
  r: na::Matrix2<f64>,
  /// **\[private\]** the current input vector(linear velocity, angular velocity)
  input: na::Vector2<f64>,
  /// **\[private\]** the time started simulating
  start_t: Instant,
  /// **\[private\]** the time started current tick
  t: Instant,
}

impl EKF {
  /// Create an EKF instance
  ///
  /// ## Arguments
  /// * `agent` - an agent instance of a robot to be estimated
  /// * `initial_pose` - the initial pose for the robot
  ///
  /// ## Returns
  /// An instance of EKF
  pub fn new(agent: Box<dyn Agent>, initial_pose: Pose) -> EKF {
    let xhat = na::Vector3::new(initial_pose.x, initial_pose.y, initial_pose.theta);
    let p = na::Matrix3::zeros();
    let q = Q * na::Matrix3::identity();
    let r = R * na::Matrix2::identity();
    let input = na::Vector2::new(0.0, 0.0);
    let start_t = Instant::now();
    let t = start_t.clone();

    EKF { agent, xhat, p, q, r, input, start_t, t }
  }

  /// Estimate a pose of the robot at this tick
  ///
  /// ## Returns
  /// Tuple of (the ideal pose of the robot, the estimated pose of the robot, the covariance matrix, the kalman gain)
  pub fn step(&mut self) -> (Pose, Pose, Vec<f64>, Vec<f64>) {
    let t = Instant::now();
    let delta = (t - self.t).as_secs_f64();

    let ideal = self.agent.get_ideal(&self.xhat, self.start_t.elapsed().as_secs_f64());
    let input = dwa_wo_obstacle::get_input(&self.agent, &self.xhat, &ideal, &self.input, delta);
    self.agent.noisy_move(&self.xhat, &input, delta);
    let (mut xhat, mut p) = EKF::predict(&self.xhat, &self.p, &self.q, &input, delta);
    let mut k: na::Matrix3x2<f64> = na::Matrix3x2::zeros();
    for observed in self.agent.noisy_observe() {
      let (updated_xhat, updated_p, updated_k) = EKF::update(&self.r, &xhat, &p, observed);
      xhat = updated_xhat;
      p = updated_p;
      k = updated_k;
    }

    self.xhat = xhat;
    self.p = p;
    self.t = t;

    (
      Pose::from_vector3(&ideal),
      Pose::from_vector3(&xhat),
      p.transpose().as_slice().to_vec(),
      k.transpose().as_slice().to_vec(),
    )
  }

  /// **\[private\]** Calculate the "predict step"
  ///
  /// ## Arguments
  /// * `xhat` - the current estimated pose(x, y, theta)
  /// * `p` - the current covariance matrix
  /// * `q` - the covariance matrix of process noise
  /// * `input` - the current input vector(linear velocity, angular velocity)
  /// * `delta` - time delta
  ///
  /// ## Returns
  /// Tuple of (predicted pose(x, y, theta), predicted covariance matrix)
  fn predict(xhat: &na::Vector3<f64>, p: &na::Matrix3<f64>, q: &na::Matrix3<f64>, input: &na::Vector2<f64>, delta: f64)
    -> (na::Vector3<f64>, na::Matrix3<f64>) {
    let a_priori_x = robot::ideal_move(xhat, input, delta);
    let f = robot::calc_f(xhat, input, delta);
    let a_priori_p = f * p * f.transpose() + q;
    (a_priori_x, a_priori_p)
  }

  /// **\[private\]** Calculate the "update step"
  ///
  /// ## Arguments
  /// * `r` - the covariance matrix of observation noise
  /// * `a_priori_x` - the predicted pose(x, y, theta)
  /// * `a_priori_p` - the predicted covariance matrix
  /// * `observed` - observed landmark
  ///
  /// ## Returns
  /// * Tuple of (estimated pose(x, y, theta), covariance matrix, kalman gain)
  fn update(r: &na::Matrix2<f64>, a_priori_x: &na::Vector3<f64>, a_priori_p: &na::Matrix3<f64>, observed: &Observed)
    -> (na::Vector3<f64>, na::Matrix3<f64>, na::Matrix3x2<f64>) {
    let yhat = na::Vector2::new(observed.distance, observed.angle) - camera::observe(&observed.landmark, a_priori_x);
    let h = camera::calc_h(&observed.landmark, a_priori_x);
    let s = h * a_priori_p * h.transpose() + r;
    let k = a_priori_p * h.transpose() * s.try_inverse().unwrap();
    let xhat = a_priori_x + k * yhat;
    let p = (na::Matrix3::identity() - k * h) * a_priori_p;
    (xhat, p, k)
  }
}

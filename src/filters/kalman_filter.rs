use std::time::Instant;

extern crate nalgebra as na;

use crate::agent::Agent;
use crate::data::{Pose, Observed};
use crate::planners::dwa_wo_obstacle;
use crate::models::{robot, camera};

const Q: f64 = 0.01;
const R: f64 = 0.02;

#[derive(Debug)]
pub struct EKF {
  pub agent: Box<dyn Agent>,
  xhat: na::Vector3<f64>,
  p: na::Matrix3<f64>,
  q: na::Matrix3<f64>,
  r: na::Matrix2<f64>,
  input: na::Vector2<f64>,
  start_t: Instant,
  t: Instant,
}

impl EKF {
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

  fn predict(xhat: &na::Vector3<f64>, p: &na::Matrix3<f64>, q: &na::Matrix3<f64>, input: &na::Vector2<f64>, delta: f64)
    -> (na::Vector3<f64>, na::Matrix3<f64>) {
    let a_priori_x = robot::ideal_move(xhat, input, delta);
    let f = robot::calc_f(xhat, input, delta);
    let a_priori_p = f * p * f.transpose() + q;
    (a_priori_x, a_priori_p)
  }

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

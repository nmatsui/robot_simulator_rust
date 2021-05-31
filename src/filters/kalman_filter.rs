use std::time::Instant;

extern crate nalgebra as na;

use crate::agent::Agent;
use crate::data::Pose;
use crate::planners::dwa_wo_obstacle;
use crate::models::robot;

const Q: f64 = 0.01;

#[derive(Debug)]
pub struct EKF {
  pub agent: Box<dyn Agent>,
  xhat: na::Vector3<f64>,
  p: na::Matrix3<f64>,
  q: na::Matrix3<f64>,
  input: na::Vector2<f64>,
  start_t: Instant,
  t: Instant,
}

impl EKF {
  pub fn new(agent: Box<dyn Agent>, initial_pose: Pose) -> EKF {
    let xhat = na::Vector3::new(initial_pose.x, initial_pose.y, initial_pose.theta);
    let p = na::Matrix3::zeros();
    let q = Q * na::Matrix3::identity();
    let input = na::Vector2::new(0.0, 0.0);
    let start_t = Instant::now();
    let t = start_t.clone();

    EKF { agent, xhat, p, q, input, start_t, t }
  }

  pub fn step(&mut self) -> (Pose, Pose, Vec<f64>, Vec<f64>) {
    let t = Instant::now();
    let delta = (t - self.t).as_secs_f64();

    let ideal = self.agent.get_ideal(self.start_t.elapsed().as_secs_f64());
    let input = dwa_wo_obstacle::get_input(&self.xhat, &ideal, &self.input, delta);
    self.agent.noisy_move(&self.xhat, &input, delta);
    let (xhat, p) = self.predict(&input, delta);

    self.xhat = xhat;
    self.p = p;
    self.t = t;

    (
      Pose::from_vector3(&ideal),
      Pose::from_vector3(&xhat),
      p.transpose().as_slice().to_vec(),
      vec![0.0],
    )
  }

  fn predict(&self, input: &na::Vector2<f64>, delta: f64) -> (na::Vector3<f64>, na::Matrix3<f64>) {
    let a_priori_x = robot::ideal_move(&self.xhat, input, delta);
    let f = robot::calc_f(&self.xhat, input, delta);
    let a_priori_p = f * self.p * f.transpose() + self.q;
    (a_priori_x, a_priori_p)
  }
}

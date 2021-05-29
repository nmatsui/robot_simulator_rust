use std::time::Instant;

extern crate nalgebra as na;

use crate::agent::Agent;
use crate::data::Pose;
use crate::planners::dwa_wo_obstacle;

#[derive(Debug)]
pub struct EKF {
  agent: Box<dyn Agent>,
  xhat: na::Vector3<f64>,
  input: na::Vector2<f64>,
  start_t: Instant,
  t: Instant,
}

impl EKF {
  pub fn new(agent: Box<dyn Agent>, initial_pose: Pose) -> EKF {
    let xhat = na::Vector3::new(initial_pose.x, initial_pose.y, initial_pose.theta);
    let input = na::Vector2::new(0.0, 0.0);
    let start_t = Instant::now();
    let t = start_t.clone();

    EKF { agent, xhat, input, start_t, t }
  }

  pub fn step(&mut self) -> (Pose, Pose, Vec<f64>, Vec<f64>) {
    let t = Instant::now();
    let delta = (t - self.t).as_secs_f64();

    let ideal = self.agent.get_ideal(self.start_t.elapsed().as_secs_f64());
    let input = dwa_wo_obstacle::get_input(&self.xhat, &ideal, &self.input, delta);
    println!("#### {:?}", input);

    self.t = t;

    (Pose {
      x: ideal[0],
      y: ideal[1],
      theta: ideal[2],
    }, Pose {
      x: 0.0,
      y: 0.0,
      theta: 0.0,
    }, vec![0.0], vec![0.0])
  }
}

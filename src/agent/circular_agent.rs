use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::Point;
use crate::utils;
use agent_derive::AgentDerive;

const INPUT_OMEGA: f64 = 0.2;

#[derive(AgentDerive)]
pub struct CircularAgent {
  landmarks: Vec<Point>,
}

impl Agent for CircularAgent {
  fn get_ideal(&self, t: f64) ->  na::Vector3<f64> {
    let angle = INPUT_OMEGA * t;
    let x = angle.cos();
    let y = angle.sin();
    let theta = utils::normalize_angle(angle + PI / 2.0);

    na::Vector3::new(x, y, theta)
  }
}

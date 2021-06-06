use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::{Point, Observed};
use crate::utils;
use agent_derive::AgentDerive;

const INPUT_OMEGA: f64 = 0.4;

#[derive(AgentDerive)]
pub struct CircularAgent {
  landmarks: Vec<Point>,
  actual: na::Vector3<f64>,
  observed: Vec<Observed>,
}

impl Agent for CircularAgent {
  fn get_ideal(&self, _: &na::Vector3<f64>, t: f64) ->  na::Vector3<f64> {
    let angle = INPUT_OMEGA * t;
    let x = angle.cos();
    let y = angle.sin();
    let theta = utils::normalize_angle(angle + PI / 2.0);

    na::Vector3::new(x, y, theta)
  }
}

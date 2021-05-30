use std::f64::consts::PI;

extern crate nalgebra as na;

use crate::agent::{AgentDerive, Agent};
use crate::data::Point;
use crate::utils;
use agent_derive::AgentDerive;

const INPUT_V: f64 = 0.2;
const INPUT_OMEGA: f64 = 0.2;

#[derive(AgentDerive)]
pub struct SquareAgent {
  landmarks: Vec<Point>,
}

impl Agent for SquareAgent {
  fn get_ideal(&self, t: f64) ->  na::Vector3<f64> {

    let d0 = 0.0;
    let d1 = d0 + 1.0 / INPUT_V;
    let d2 = d1 + PI / 2.0 / INPUT_OMEGA;
    let d3 = d2 + 2.0 / INPUT_V;
    let d4 = d3 + PI / 2.0 / INPUT_OMEGA;
    let d5 = d4 + 2.0 / INPUT_V;
    let d6 = d5 + PI / 2.0 / INPUT_OMEGA;
    let d7 = d6 + 2.0 / INPUT_V;
    let d8 = d7 + PI / 2.0 / INPUT_OMEGA;
    let d9 = d8 + 1.0 / INPUT_V;

    let delta = t % d9;

    let (x, y, theta) = match delta {
      dlt if d0 <= dlt && dlt < d1 => ( 1.0,                         0.0 + INPUT_V * (dlt - d0), PI * 1.0 / 2.0),
      dlt if d1 <= dlt && dlt < d2 => ( 1.0,                         1.0,                        PI * 1.0 / 2.0 + INPUT_OMEGA * (dlt - d1)),
      dlt if d2 <= dlt && dlt < d3 => ( 1.0 - INPUT_V * (dlt - d2),  1.0,                        PI * 2.0 / 2.0),
      dlt if d3 <= dlt && dlt < d4 => (-1.0,                         1.0,                        PI * 2.0 / 2.0 + INPUT_OMEGA * (dlt - d3)),
      dlt if d4 <= dlt && dlt < d5 => (-1.0,                         1.0 - INPUT_V * (dlt - d4), PI * 3.0 / 2.0),
      dlt if d5 <= dlt && dlt < d6 => (-1.0,                        -1.0,                        PI * 3.0 / 2.0 + INPUT_OMEGA * (dlt - d5)),
      dlt if d6 <= dlt && dlt < d7 => (-1.0 + INPUT_V * (dlt - d6), -1.0,                        PI * 0.0 / 2.0),
      dlt if d7 <= dlt && dlt < d8 => ( 1.0,                        -1.0,                        PI * 0.0 / 2.0 + INPUT_OMEGA * (dlt - d7)),
      dlt if d8 <= dlt && dlt < d9 => ( 1.0,                        -1.0 + INPUT_V * (dlt - d8), PI * 1.0 / 2.0),
      _ => panic!("assert error")
    };

    na::Vector3::new(x, y, utils::normalize_angle(theta))
  }
}

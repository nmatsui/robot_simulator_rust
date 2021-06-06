mod circular_agent;
mod square_agent;
mod waypoints_agent;

use std::fmt;

extern crate nalgebra as na;
extern crate rand;
use rand_distr::{Normal, Distribution};

use crate::data::{Point, Observed};
use crate::models::robot;

const ACTUAL_XY_SD: f64 = 0.005;
const ACTUAL_THETA_SD: f64 = 0.01;
const OBSERVED_DIST_SD: f64 = 0.02;
const OBSERVED_ANGLE_SD: f64 = 0.02;

pub fn create_agent(mut args: std::env::Args, landmarks: Vec<Point>) -> Result<Box<dyn Agent>, String> {
  args.next();

  let agent: Box<dyn Agent> = match args.next() {
    Some(name) => {
      match name.to_lowercase().as_str() {
        "circular" => Box::new(circular_agent::CircularAgent::new(landmarks)),
        "square" => Box::new(square_agent::SquareAgent::new(landmarks)),
        "waypoints" => Box::new(waypoints_agent::WaypointsAgent::new(landmarks)),
        _ => return Err(format!("No agent found: {}", name)),
      }
    },
    None => return Err(format!("Agent name does not found")),
  };
  Ok(agent)
}

pub trait AgentDerive: Send {
  fn get_name(&self) -> &str;
  fn get_landmarks(&self) -> &Vec<Point>;
  fn set_actual(&mut self, actual: na::Vector3<f64>) -> ();
  fn get_actual(&self) -> &na::Vector3<f64>;
  fn set_observed(&mut self, observed: Vec<Observed>) -> ();
  fn get_observed(&self) -> &Vec<Observed>;
  fn noisy_move(&mut self, current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> () {
    let ideal_pose = robot::ideal_move(current, input, delta);
    let noisy_pose = na::Vector3::new(
      Normal::new(ideal_pose[0], ACTUAL_XY_SD).unwrap().sample(&mut rand::thread_rng()),
      Normal::new(ideal_pose[1], ACTUAL_XY_SD).unwrap().sample(&mut rand::thread_rng()),
      Normal::new(ideal_pose[2], ACTUAL_THETA_SD).unwrap().sample(&mut rand::thread_rng()),
    );
    self.set_actual(noisy_pose);
  }
  fn noisy_observe(&mut self) -> &Vec<Observed> {
    let observed = self.get_landmarks()
                       .iter()
                       .map(|landmark| {
                         let actual = self.get_actual();
                         let actual_point = actual.fixed_rows::<2>(0);
                         let dist = (na::Vector2::new(landmark.x, landmark.y) - actual_point).norm_squared().sqrt();
                         let noisy_dist = Normal::new(dist, OBSERVED_DIST_SD).unwrap().sample(&mut rand::thread_rng());
                         let angle = (landmark.y - actual[1]).atan2(landmark.x - actual[0]) - actual[2];
                         let noisy_angle = Normal::new(angle, OBSERVED_ANGLE_SD).unwrap().sample(&mut rand::thread_rng());

                         Observed {
                           landmark: landmark.clone(),
                           distance: noisy_dist,
                           angle: noisy_angle,
                         }
                       })
                       .collect::<Vec<_>>();
    self.set_observed(observed);
    self.get_observed()
  }
}

pub trait Agent: AgentDerive {
  fn get_max_accelarations(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_LIN_ACC, robot::MAX_ANG_ACC)
  }
  fn get_linear_velocities(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_V, robot::MIN_V)
  }
  fn get_angular_velocities(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_OMEGA, robot::MIN_OMEGA)
  }
  fn get_ideal(&self, current: &na::Vector3<f64>, t: f64) -> na::Vector3<f64>;
}

impl fmt::Debug for dyn Agent {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    f.debug_struct(self.get_name()).finish()
  }
}

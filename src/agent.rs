//! The `agent` module provides some robot agents which define the ideal trajectory that the simulated robot should be move.

mod circular_agent;
mod square_agent;
mod waypoints_agent;

use std::fmt;

extern crate nalgebra as na;
extern crate rand;
use rand_distr::{Normal, Distribution};

use crate::data::{Point, Observed};
use crate::models::robot;

/// **\[private\]** The standard deviation value which is used to simulate the gaussian noise of the autonomous mobile robot's (position: x, y)
const ACTUAL_XY_SD: f64 = 0.005;
/// **\[private\]** The standard deviation value which is used to simulate the gaussian noise of the autonomous mobile robot's (direction: theta)
const ACTUAL_THETA_SD: f64 = 0.01;
/// **\[private\]** The standard deviation value which is used to simulate the gaussian noise of the camera's observation (distance to marker)
const OBSERVED_DIST_SD: f64 = 0.02;
/// **\[private\]** The standard deviation value which is used to simulate the gaussian noise of the camera's observation (angle between robot heading and marker)
const OBSERVED_ANGLE_SD: f64 = 0.02;

/// Create a concrete Agent specified by the commandline argument such as CircularAgent, SquareAgent and WaypontsAgnet, and returns it as Agent trait object
///
/// ## Arguments
/// * `args` - a command line argument that means the Agent to be used. This crate can receive the following arguments:
///     * circular
///     * square
///     * waypoints
/// * `landmarks` - the vector of landmark points which will be observed from robot
///
/// ## Returns
/// This function returns a instanciated Agent as an Agent trait object
///
/// ## Errors
/// When no command line argumet is given or unknown argument is given, this function returns Error
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

/// A trait which has common implementations for all Agents
///
/// Some functions that uses `self`'s field(s) are implemented by [agent_derive] Derive Macro
pub trait AgentDerive: Send {

  /// Get the name of the concrete `struct` that implemented this trait
  fn get_name(&self) -> &str;

  /// Get the landmarks field of the concrete `struct` that implemented this trait
  fn get_landmarks(&self) -> &Vec<Point>;

  /// Set the actual pose (x, y, theta) to the concrete `struct` that implemented this trait
  /// ## Arguments
  /// * `actual` - actual pose which is defined as nalgebra::Vector3::new(x, y, theta)
  fn set_actual(&mut self, actual: na::Vector3<f64>) -> ();

  /// Get the actual pose (x, y, theta) of the concrete `struct` that implemented this trait
  ///
  /// ## Returns
  /// Actual pose which is defined as nalgebra::Vector3::new(x, y, theta)
  fn get_actual(&self) -> &na::Vector3<f64>;

  /// Set the vector of observed data (landmark, distance, angle) to the concrete `struct` that implemented this trait
  fn set_observed(&mut self, observed: Vec<Observed>) -> ();

  /// Get the vector of observed data (landmark, distance, angle) of the concrete `struct` that implemented this trait
  fn get_observed(&self) -> &Vec<Observed>;

  /// Move the simulated robot with gaussian noise according to the robot's motion model
  ///
  /// The simulated actual pose is stored to the concrete Agent's field
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  /// * `input` - input velocity vector (linear velocity, angular velocity) which is defined as nalgebra::Vector2::new(v, omega)
  /// * `delta` - time delta
  fn noisy_move(&mut self, current: &na::Vector3<f64>, input: &na::Vector2<f64>, delta: f64) -> () {
    let ideal_pose = robot::ideal_move(current, input, delta);
    let noisy_pose = na::Vector3::new(
      Normal::new(ideal_pose[0], ACTUAL_XY_SD).unwrap().sample(&mut rand::thread_rng()),
      Normal::new(ideal_pose[1], ACTUAL_XY_SD).unwrap().sample(&mut rand::thread_rng()),
      Normal::new(ideal_pose[2], ACTUAL_THETA_SD).unwrap().sample(&mut rand::thread_rng()),
    );
    self.set_actual(noisy_pose);
  }

  /// Observe the landmarks with gaussian noise according to the camera's obervation model
  ///
  /// The simulated actual observations is stored to the concrete Agent's field
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

/// A trait which has specific implementations for individual Agents
///
/// Some functions have default implementations
pub trait Agent: AgentDerive {

  /// Get the maximum accelaration values
  ///
  /// ## Arguments
  /// * `_` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///     *  Since the current pose is not used in this default implementation, the argument name is defind as `_`
  ///
  /// ## Returns
  /// Tuple of (maximum linear accelaration, maximum angular accelaration)
  fn get_max_accelarations(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_LIN_ACC, robot::MAX_ANG_ACC)
  }

  /// Get the maximum and minimum linear velocity values
  ///
  /// ## Arguments
  /// * `_` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///     *  Since the current pose is not used in this default implementation, the argument name is defind as `_`
  ///
  /// ## Returns
  /// Tuple of (maximum linear velocity, minimum linear velocity)
  fn get_linear_velocities(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_V, robot::MIN_V)
  }

  /// Get the maximum and minimum angular velocity values
  ///
  /// ## Arguments
  /// * `_` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  ///     *  Since the current pose is not used in this default implementation, the argument name is defind as `_`
  ///
  /// ## Returns
  /// Tuple of (maximum angular velocity, minimum angular velocity)
  fn get_angular_velocities(&self, _: &na::Vector3<f64>) -> (f64, f64) {
    (robot::MAX_OMEGA, robot::MIN_OMEGA)
  }

  /// Get the ideal pose of the simulated robot
  ///
  /// ## Arguments
  /// * `current` - current pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  /// * `t` - elapsed time (sec) from the start of this simulation
  ///
  /// ## Returns
  /// ideal pose of robot which is defined as nalgebra::Vector3::new(x, y, theta)
  fn get_ideal(&self, current: &na::Vector3<f64>, t: f64) -> na::Vector3<f64>;
}

/// The implementation for Debug format of Agent
impl fmt::Debug for dyn Agent {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    f.debug_struct(self.get_name()).finish()
  }
}

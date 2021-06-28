//! The `dwa_wo_obstacle` module provides the `get_input` function which plans a local path in a "[Dynamic Window Approach](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)" like way
//!
//! <u>This module does not implement the obstacle avoidance feature</u>

extern crate nalgebra as na;
extern crate itertools;

use crate::agent::Agent;
use crate::models::robot;
use crate::utils;

/// **\[private\]** The linear velocity resolution to sampling the Dynamic Window
const V_RESOLUTION: f64 = 0.01;
/// **\[private\]** The angular velocity resolution to sampling the Dynamic Window
const OMEGA_RESOLUTION: f64 = 0.01;

/// **\[private\]** The weight of the "target heading" evaluation function used when the goal is "far away"
const FAR_ERROR_ANGLE_GAIN: f64 = 1.0;
/// **\[private\]** The weight of the "velocity" evaluation function used when the goal is "far away"
const FAR_VELOCITY_GAIN: f64 = 0.5;
/// **\[private\]** The weight of the "distance" evaluation function used when the goal is "far away"
const FAR_DISTANCE_GAIN: f64 = 0.8;
/// **\[private\]** The weight of the "theta" evaluation function used when the goal is "far away"
const FAR_THETA_GAIN: f64 = 0.01;

/// **\[private\]** The weight of the "target heading" evaluation function used when the goal is "near by"
const NEAR_ERROR_ANGLE_GAIN: f64 = 1.0;
/// **\[private\]** The weight of the "velocity" evaluation function used when the goal is "near by"
const NEAR_VELOCITY_GAIN: f64 = 0.01;
/// **\[private\]** The weight of the "distance" evaluation function used when the goal is "near by"
const NEAR_DISTANCE_GAIN: f64 = 0.8;
/// **\[private\]** The weight of the "theta" evaluation function used when the goal is "near by"
const NEAR_THETA_GAIN: f64 = 0.8;

/// **\[private\]** The threshold to determine whether the goal is "far away" or "near by"
const DISTANCE_SQUARED_THRESHOLD: f64 = 0.01;

/// Get the input vector (linear velocity, angular velocity) of next tick based on the goal of next tick and the current pose and input vector of the simulated robot
///
/// ## Arguments
/// * `agent` - the agent instance of this simulated robot
/// * `current` - the current pose(x, y, theta) of this simulated robot
/// * `destination` - the goal pose(x, y, theta)
/// * `current_input` - the current input vector(linear velocity, angular velocity) of this simulated robot
/// * `delta` - time delta to next tick
///
/// ## Returns
/// * The input vector(linear velocity, angular velocity) of next tick
pub fn get_input(agent: &Box<dyn Agent>, current: &na::Vector3<f64>, destination: &na::Vector3<f64>,
                 current_input: &na::Vector2<f64>, delta: f64) -> na::Vector2<f64> {
  let max_accelarations = agent.get_max_accelarations(current);
  let linear_velocities = agent.get_linear_velocities(current);
  let angular_velocities = agent.get_angular_velocities(current);

  let (v_range, omega_range) = get_window(max_accelarations, linear_velocities, angular_velocities, current_input, delta);

  let mut input_vec: Vec<na::Vector2<f64>> = Vec::new();
  let mut heading_vec: Vec<f64> = Vec::new();
  let mut velocity_vec: Vec<f64> = Vec::new();
  let mut distance_vec: Vec<f64> = Vec::new();
  let mut theta_vec: Vec<f64> = Vec::new();

  for (v, omega) in itertools::iproduct!(v_range, omega_range) {
    let input = na::Vector2::new(v, omega);
    input_vec.push(input);

    let next = robot::ideal_move(current, &input, delta);
    heading_vec.push(eval_heading(&next, destination));
    velocity_vec.push(eval_velocity(&input));
    distance_vec.push(eval_distance(&next, destination));
    theta_vec.push(eval_theta(&next, destination));
  }

  let heading_vec = utils::normalize_min_max(heading_vec);
  let velocity_vec = utils::normalize_min_max(velocity_vec);
  let distance_vec = utils::normalize_min_max(distance_vec);
  let theta_vec = utils::normalize_min_max(theta_vec);

  let min_idx = itertools::izip!(&heading_vec, &velocity_vec, &distance_vec, &theta_vec)
                .map(|(heading, velocity, distance, theta)|
                  if (current.fixed_rows::<2>(0) - destination.fixed_rows::<2>(0)).norm_squared() < DISTANCE_SQUARED_THRESHOLD {
                    NEAR_ERROR_ANGLE_GAIN * heading + NEAR_VELOCITY_GAIN * velocity + NEAR_DISTANCE_GAIN * distance + NEAR_THETA_GAIN * theta
                  } else {
                    FAR_ERROR_ANGLE_GAIN * heading + FAR_VELOCITY_GAIN * velocity + FAR_DISTANCE_GAIN * distance + FAR_THETA_GAIN * theta
                  }
                )
                .enumerate()
                .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(idx, _)| idx)
                .unwrap();

  input_vec[min_idx]
}

/// **\[private\]** Get the sampled values of linear and angular velocities in the next Dynamic Window
///
/// ## Arguments
/// * `max_accelarations` - the tuple of (maximum linear accelaration, maximum angular accelaration)
/// * `linear_velocities` - the tuple of (maximum linear velocity, minimum linear velocity)
/// * `angular_velocities` - the tuple of (maximum angular velocity, minimum angular velocity)
/// * `current_input` - the current input vector(linear velocity, angular velocity) of this simulated robot
/// * `delta` - time delta to next tick
///
/// ## Returns
/// The tuple of sampled values (Vec of sampled linear velocities, Vec of sampled angular velocities)
fn get_window(max_accelarations: (f64, f64), linear_velocities: (f64, f64), angular_velocities: (f64, f64), current_input: &na::Vector2<f64>, delta: f64) -> (Vec<f64>, Vec<f64>) {
  let delta_v = max_accelarations.0 * delta;
  let delta_omega = max_accelarations.1 * delta;

  let min_v = linear_velocities.1.max(current_input[0] - delta_v);
  let max_v = linear_velocities.0.min(current_input[0] + delta_v);
  let min_omega = angular_velocities.1.max(current_input[1] - delta_omega);
  let max_omega = angular_velocities.0.min(current_input[1] + delta_omega);

  (
    utils::step_by_float(min_v, max_v, V_RESOLUTION),
    utils::step_by_float(min_omega, max_omega, OMEGA_RESOLUTION),
  )
}

/// **\[private\]** Evaluate whether the simulated robot are heading toward the goal
///
/// ## Arguments
/// * `next` -　the ideal pose(x, y, theta) moved by the input vector being considered
/// * `destination` - the goal pose(x, y, theta)
///
/// ## Returns
/// The angle between the direction of the robot moved by the input vector considered and the position of the goal
fn eval_heading(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  let angle = (destination[1] - next[1]).atan2(destination[0] - next[0]);
  utils::normalize_angle(angle - next[2]).abs()
}

/// **\[private\]** Evaluate the simulated robot's speed toward the goal
///
/// ## Arguments
/// * `input` - the input vector(linear velocity, angular velocity) being considered
///
/// ## Returns
/// The value subtracting input linear velocity from maximum linear velocity
fn eval_velocity(input: &na::Vector2<f64>) -> f64 {
  robot::MAX_V - input[0]
}

/// **\[private\]** Evaluate the distance between the simulated robot and the goal
///
/// ## Arguments
/// * `next` -　the ideal pose(x, y, theta) moved by the input vector being considered
/// * `destination` - the goal pose(x, y, theta)
///
/// ## Returns
/// The distance between the position of the robot moved by the input vector considered and the position of the goal
fn eval_distance(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  (next.fixed_rows::<2>(0) - destination.fixed_rows::<2>(0)).norm_squared().sqrt()
}

/// **\[private\]** Evaluate whether the direction of the simulated robot and the direction of the goal match
///
/// ## Arguments
/// * `next` -　the ideal pose(x, y, theta) moved by the input vector being considered
/// * `destination` - the goal pose(x, y, theta)
///
/// ## Returns
/// The absolute value between the direction of the robot moved by the input vector considered and the direction of the goal
fn eval_theta(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  utils::normalize_angle(next[2] - destination[2]).abs()
}

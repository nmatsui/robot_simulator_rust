extern crate nalgebra as na;
extern crate itertools;

use crate::agent::Agent;
use crate::models::robot;
use crate::utils;

const V_RESOLUTION: f64 = 0.01;
const OMEGA_RESOLUTION: f64 = 0.01;

const FAR_ERROR_ANGLE_GAIN: f64 = 1.0;
const FAR_VELOCITY_GAIN: f64 = 0.5;
const FAR_DISTANCE_GAIN: f64 = 0.8;
const FAR_THETA_GAIN: f64 = 0.01;

const NEAR_ERROR_ANGLE_GAIN: f64 = 1.0;
const NEAR_VELOCITY_GAIN: f64 = 0.01;
const NEAR_DISTANCE_GAIN: f64 = 0.8;
const NEAR_THETA_GAIN: f64 = 0.8;

const DISTANCE_SQUARED_THRESHOLD: f64 = 0.01;

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

fn eval_heading(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  let angle = (destination[1] - next[1]).atan2(destination[0] - next[0]);
  utils::normalize_angle(angle - next[2]).abs()
}

fn eval_velocity(input: &na::Vector2<f64>) -> f64 {
  robot::MAX_V - input[0]
}

fn eval_distance(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  (next.fixed_rows::<2>(0) - destination.fixed_rows::<2>(0)).norm_squared().sqrt()
}

fn eval_theta(next: &na::Vector3<f64>, destination: &na::Vector3<f64>) -> f64 {
  utils::normalize_angle(next[2] - destination[2]).abs()
}

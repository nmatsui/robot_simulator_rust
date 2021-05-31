extern crate nalgebra as na;

use crate::data::Point;

pub fn observe(landmark: &Point, current: &na::Vector3<f64>) -> na::Vector2<f64> {
  let dist = (na::Vector2::new(landmark.x, landmark.y) - current.fixed_rows::<2>(0)).norm_squared().sqrt();
  let angle = (landmark.y - current[1]).atan2(landmark.x - current[0]) - current[2];
  na::Vector2::new(dist, angle)
}

pub fn calc_h(landmark: &Point, current: &na::Vector3<f64>) -> na::Matrix2x3<f64> {
  let q = (landmark.x - current[0]).powf(2.0) + (landmark.y - current[1]).powf(2.0);

  na::Matrix2x3::new((current[0] - landmark.x)/q.sqrt(), (current[1] - landmark.y)/q.sqrt(),  0.0,
                     (landmark.y - current[1])/q,        (current[0] - landmark.x)/q,        -1.0)
}

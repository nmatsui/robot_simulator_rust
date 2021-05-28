use std::f64::consts::PI;

pub fn normalize_angle(r: f64) -> f64 {
  (r + PI) % (2.0 * PI) - PI
}

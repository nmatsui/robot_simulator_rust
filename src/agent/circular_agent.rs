use crate::agent::Agent;

pub struct CircularAgent {
  landmarks: Vec<(f32, f32)>,
}

impl CircularAgent {
  pub fn new(landmarks: Vec<(f32, f32)>) -> CircularAgent {
    CircularAgent { landmarks }
  }
}

impl Agent for CircularAgent {
  fn get_name(&self) -> &str {
    "CircularAgent"
  }

  fn get_ideal(&self, t: f32) -> (f32, f32, f32) {
    (1.0, 2.0, 3.0)
  }
}

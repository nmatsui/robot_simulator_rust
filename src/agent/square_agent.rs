use crate::agent::Agent;

pub struct SquareAgent {
  landmarks: Vec<(f32, f32)>,
}

impl SquareAgent {
  pub fn new(landmarks: Vec<(f32, f32)>) -> SquareAgent {
    SquareAgent { landmarks }
  }
}

impl Agent for SquareAgent {
  fn get_name(&self) -> &str {
    "SquareAgent"
  }

  fn get_ideal(&self, t: f32) -> (f32, f32, f32) {
    (1.0, 2.0, 3.0)
  }
}

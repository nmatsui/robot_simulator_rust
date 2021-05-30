mod circular_agent;
mod square_agent;

use std::fmt;

extern crate nalgebra as na;

use crate::data::Point;

pub fn create_agent(mut args: std::env::Args, landmarks: Vec<Point>) -> Result<Box<dyn Agent>, String> {
  args.next();

  let agent: Box<dyn Agent> = match args.next() {
    Some(name) => {
      match name.to_lowercase().as_str() {
        "circular" => Box::new(circular_agent::CircularAgent::new(landmarks)),
        "square" => Box::new(square_agent::SquareAgent::new(landmarks)),
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
  fn move_next(&self) -> () {
    println!("move");
  }
}

pub trait Agent: AgentDerive {
  fn get_ideal(&self, t: f64) -> na::Vector3<f64>;
}

impl fmt::Debug for dyn Agent {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    f.debug_struct(self.get_name()).finish()
  }
}

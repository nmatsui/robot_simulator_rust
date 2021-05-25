mod circular_agent;
mod square_agent;

use std::fmt;

pub fn create_agent(mut args: std::env::Args) -> Result<Box<dyn Agent>, String> {
  args.next();

  let landmarks = vec![(1.0, 2.0)];

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

pub trait Agent {
  fn get_ideal(&self, t: f32) -> (f32, f32, f32);
  fn get_name(&self) -> &str;
  fn move_next(&self) {
    println!("move");
  }
}

impl fmt::Debug for dyn Agent {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    f.debug_struct(self.get_name()).finish()
  }
}

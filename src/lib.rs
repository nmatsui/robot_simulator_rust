pub mod agent;

use std::error::Error;

pub fn run(args: std::env::Args) -> Result<(), Box<dyn Error>> {
  let agt = agent::create_agent(args)?;

  println!("{:?}", agt);

  Ok(())
}
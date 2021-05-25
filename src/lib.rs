pub mod agent;
pub mod filters;
pub mod timers;

use std::error::Error;

pub fn run(args: std::env::Args) -> Result<(), Box<dyn Error>> {
  let agt = agent::create_agent(args)?;
  let ekf = filters::EKF::new();
  timers::start(ekf)?;

  Ok(())
}
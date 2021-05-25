use crate::agent::Agent;

#[derive(Debug)]
pub struct EKF {
}

impl EKF {
  pub fn new() -> EKF {
    EKF {}
  }

  pub fn step(&self) -> () {
    println!("step");
  }
}
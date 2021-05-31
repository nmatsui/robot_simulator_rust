use std::time::Duration;

use tokio;
use zmq;
use serde::{Deserialize, Serialize};
use serde_json;

use crate::filters::kalman_filter::EKF;
use crate::data::{Pose, Observed};

const INTERVAL_MS: u64 = 200;
const PORT: u64 = 5556;


pub fn start(mut ekf: EKF) -> Result<(), Box<dyn std::error::Error>> {
  let rt = tokio::runtime::Runtime::new()?;
  let zeromq = ZeroMQ::new(PORT)?;

  rt.block_on(async {
    let forever = tokio::task::spawn(async move {
      let mut interval = tokio::time::interval(Duration::from_millis(INTERVAL_MS));

      loop {
        interval.tick().await;
        let (ideal, xhat, p, k) = ekf.step();
        let actual = Pose::from_vector3(ekf.agent.get_actual());
        let observed = ekf.agent.get_observed();
        if let Err(e) = zeromq.send(ideal, actual, xhat, observed, p, k) {
          eprintln!("send message error: {:?}", e);
        }
      }
    });
    let _ = forever.await;
  });
  Ok(())
}

struct ZeroMQ {
  publisher: zmq::Socket,
}

impl ZeroMQ {
  fn new(port: u64) -> Result<ZeroMQ, Box<dyn std::error::Error>> {
    let context = zmq::Context::new();
    let publisher = context.socket(zmq::PUB)?;
    publisher.bind(&format!("tcp://*:{}", port))?;
    Ok(ZeroMQ { publisher })
  }

  fn send(&self, ideal: Pose, actual: Pose, xhat: Pose, observed: &Vec<Observed>, p: Vec<f64>, k: Vec<f64>) -> Result<(), Box<dyn std::error::Error>> {
    let payload = Payload {
      ideal: ideal,
      actual: actual,
      xhat: xhat,
      observed: observed.to_vec(),
      covariance: p,
      kalmanGain: k,
    };
    println!("payload = {:?}", payload);
    let j = serde_json::to_string(&payload)?;
    self.publisher.send(&j, 0)?;
    Ok(())
  }
}

#[allow(non_snake_case)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
struct Payload {
  ideal: Pose,
  actual: Pose,
  xhat: Pose,
  observed: Vec<Observed>,
  covariance: Vec<f64>,
  kalmanGain: Vec<f64>,
}

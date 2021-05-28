use std::time::Duration;

use tokio;
use zmq;
use serde::{Deserialize, Serialize};
use serde_json;

use crate::filters::EKF;
use crate::data::{Point, Pose};

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
        if let Err(e) = zeromq.send(ideal, xhat, p, k) {
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

  fn send(&self, ideal: Pose, xhat: Pose, p: Vec<f64>, k: Vec<f64>) -> Result<(), Box<dyn std::error::Error>> {
    let payload = Payload {
      ideal: ideal,
      actual: Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
      },
      xhat: xhat,
      observed: vec![Observed {
        landmark: Point {
          x: 1.0,
          y: 1.0,
        },
        distance: 1.0,
        angle: 0.1,
      }],
      covariance: p,
      kalmanGain: k,
    };
    println!("payload = {:?}", payload);
    let j = serde_json::to_string(&payload)?;
    self.publisher.send(&j, 0)?;
    Ok(())
  }
}

#[derive(Debug)]
#[derive(Serialize, Deserialize)]
struct Observed {
  landmark: Point,
  distance: f64,
  angle: f64,
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

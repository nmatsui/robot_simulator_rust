use std::time::Duration;

use tokio;
use zmq;
use serde::{Deserialize, Serialize};
use serde_json;

use crate::filters::EKF;

const INTERVAL_MS: u64 = 200;
const PORT: u64 = 5556;


pub fn start(ekf: EKF) -> Result<(), Box<dyn std::error::Error>> {
  let rt = tokio::runtime::Runtime::new()?;
  let zeromq = ZeroMQ::new(PORT)?;

  rt.block_on(async {
    let forever = tokio::task::spawn(async move {
      let mut interval = tokio::time::interval(Duration::from_millis(INTERVAL_MS));

      loop {
        interval.tick().await;
        ekf.step();
        if let Err(e) = zeromq.send() {
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

  fn send(&self) -> Result<(), Box<dyn std::error::Error>> {
    let payload = Payload {
      ideal: Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
      },
      actual: Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
      },
      xhat: Pose {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
      },
      observed: vec![Observed {
        landmark: Landmark {
          x: 1.0,
          y: 1.0,
        },
        distance: 1.0,
        angle: 0.1,
      }],
      covariance: vec![0.0],
      kalmanGain: vec![0.0],
    };
    let j = serde_json::to_string(&payload)?;
    self.publisher.send(&j, 0)?;
    Ok(())
  }
}

#[derive(Serialize, Deserialize)]
struct Pose {
  x: f64,
  y: f64,
  theta: f64,
}

#[derive(Serialize, Deserialize)]
struct Landmark {
  x: f64,
  y: f64,
}

#[derive(Serialize, Deserialize)]
struct Observed {
  landmark: Landmark,
  distance: f64,
  angle: f64,
}

#[allow(non_snake_case)]
#[derive(Serialize, Deserialize)]
struct Payload {
  ideal: Pose,
  actual: Pose,
  xhat: Pose,
  observed: Vec<Observed>,
  covariance: Vec<f64>,
  kalmanGain: Vec<f64>,
}

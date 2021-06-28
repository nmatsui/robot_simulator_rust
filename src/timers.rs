//! The `timers` module provides the `start` function which executes [EKF] periodically and sends it's results to a drawing engine by ZeroMQ

use std::time::Duration;

use tokio;
use zmq;
use serde::{Deserialize, Serialize};
use serde_json;

use crate::filters::kalman_filter::EKF;
use crate::data::{Pose, Observed};

/// **\[private\]** The interval (milliseconds) to call [EKF]
const INTERVAL_MS: u64 = 200;
/// **\[private\]** The zeromq port number for the drawing engine
const PORT: u64 = 5556;

/// Start an async timer event which executes the following processing
/// 1. calls [crate::filters::kalman_filter::EKF::step] method and gets the estimated pose and other results of simulated robot
/// 1. gets the distances and angles of observed markers
/// 1. gets the hidden actual pose of simulated robot
/// 1. sends above data to the drawing engine by using ZeroMQ
///
/// ## Arguments
/// * `ekf` - EKF object
///
/// ## Errors
/// Raises an error when ZeroMQ
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

/// **\[private\]** A struct which stores the ZeroMQ Socket
struct ZeroMQ {
  /// ZeroMQ Socket
  publisher: zmq::Socket,
}

impl ZeroMQ {
  /// **\[private\]** Create a new ZeroMQ instance
  ///
  /// ## Arguments
  /// * `port` - the ZeroMQ port of drawing engine
  ///
  /// ## Returns
  /// A ZeroMQ instance
  ///
  /// ## Errors
  /// When failed to create ZeroMQ Socket, the error is raised
  fn new(port: u64) -> Result<ZeroMQ, Box<dyn std::error::Error>> {
    let context = zmq::Context::new();
    let publisher = context.socket(zmq::PUB)?;
    publisher.bind(&format!("tcp://*:{}", port))?;
    Ok(ZeroMQ { publisher })
  }

  /// **\[private\]** Send the robot's data as JSON to the drawing engine by using ZeroMQ
  ///
  /// ## Arguments
  /// * `ideal` - the ideal pose of the simulated robot
  /// * `actual` - the hidden actual pose of the simulated robot
  /// * `xhat` - the estimated pose of the simulated robot
  /// * `observed` - the list of the observed marker's data
  /// * `p` - the error covariance matrix
  /// * `k` - the Kalman Gain
  ///
  /// ## Errors
  /// When the given data cannot be serialized as JSON, or when the serialized data cannot be sent to drawing engine by ZeroMQ, the error is raised
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

/// **\[private\]** A struct which stores the data to be sent to drawing engine
#[allow(non_snake_case)]
#[derive(Debug)]
#[derive(Serialize, Deserialize)]
struct Payload {
  /// the ideal pose of the simulated robot
  ideal: Pose,
  /// the hidden actual pose of the simulated robot
  actual: Pose,
  /// the estimated pose of the simulated robot
  xhat: Pose,
  /// the list of the observed marker's data
  observed: Vec<Observed>,
  /// the error covariance matrix
  covariance: Vec<f64>,
  /// the Kalman Gain
  kalmanGain: Vec<f64>,
}

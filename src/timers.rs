use std::time::Duration;

use tokio;

use crate::filters::EKF;

const INTERVAL_MS: u64 = 200;

pub fn start(ekf: EKF) -> Result<(), Box<dyn std::error::Error>> {
  let rt = tokio::runtime::Runtime::new()?;
  rt.block_on(async {
    let forever = tokio::task::spawn(async move {
      let mut interval = tokio::time::interval(Duration::from_millis(INTERVAL_MS));

      loop {
        interval.tick().await;
        ekf.step();
      }
    });
    let _ = forever.await;
  });
  Ok(())
}
use std::f64::consts::PI;

use num::Float;

pub fn normalize_angle<F: Float>(r: F) -> F {
  let pi = F::from(PI).unwrap();
  (r + pi) % (F::from(2.0).unwrap() * pi) - pi
}

pub fn step_by_float<F: Float>(start: F, stop: F, step: F) -> Vec<F> {
  let num = (stop - start) / step + F::from(1).unwrap();
  let r = 0..num.to_usize().unwrap();
  r.map(|i| start + (stop - start) * F::from(i).unwrap() / (num - F::from(1).unwrap())).collect::<Vec<_>>()
}

pub fn normalize_min_max<F: Float>(v: Vec<F>) -> Vec<F> {
  let min = v.iter().fold(F::from(0.0 / 0.0).unwrap(), |m, n| n.min(m));
  let max = v.iter().fold(F::from(0.0 / 0.0).unwrap(), |m, n| n.max(m));
  match max - min {
    d if d == F::from(0.0).unwrap() => {
      vec![F::from(1.0).unwrap(); v.len()]
    },
    d => {
      v.iter().map(|f| (*f - min) / d).collect::<Vec<_>>()
    }
  }
}

#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]

use libm::sqrtf;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub mean: u16,
    pub stddev: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    count: u16,
    mean: f32,
    m2: f32,
}

impl Calibration {
    pub fn new() -> Calibration {
        Calibration {
            count: 0,
            mean: 0.0,
            m2: 0.0,
        }
    }

    pub fn add_sample(&mut self, value: u16) {
        self.count += 1;
        let delta = value as f32 - self.mean;
        self.mean += delta / self.count as f32;
        let delta2 = value as f32 - self.mean;
        self.m2 += delta * delta2;
    }

    pub fn num_samples(&self) -> u16 {
        self.count
    }

    pub fn get_point(&self) -> Point {
        assert!(self.count > 1);
        let stddev = sqrtf(self.m2 / (self.count - 1) as f32);

        assert!(self.mean >= 0.0);
        assert!(self.mean <= u16::MAX.into());
        assert!(stddev <= u16::MAX as f32);

        Point {
            mean: self.mean as u16,
            stddev: stddev as u16,
        }
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use statrs::statistics::Statistics;

    #[test]
    fn test_point() {
        let samples = [1300, 1150, 1407, 1345];

        let mut cal = Calibration::new();
        for value in samples {
            cal.add_sample(value);
        }
        let point = cal.get_point();

        let mean = samples.map(|v| f64::from(v)).mean();
        let stddev = samples.map(|v| f64::from(v)).std_dev();

        assert_eq!(point.mean, mean as u16);
        assert_eq!(point.stddev, stddev as u16);
    }
}

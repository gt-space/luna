//! Where the GPS frame sits relative to the filter's.
//!
//! The filter's origin is under the vehicle on the pad; the GPS reports its own
//! frame. The offset between them cannot be read off one fix: each fix carries the
//! receiver's noise (1.5 m on the CTV's, standalone), so an origin taken from one
//! fix is off by that much for the whole flight, and every later fix pulls the
//! estimate toward the wrong place. The offset is the mean of the fixes taken over
//! a window instead, against where the filter's own estimate (inertial, GPS
//! velocity, LiDAR and barometer) says the antenna was.

use std::time::Duration;

use nalgebra::Vector3;

/// Averages the offset between the filter's antenna position and the GPS frame
/// over the first `window` of fixes.
pub struct OriginAverager {
    window: Duration,
    first: Option<Duration>,
    sum: Vector3<f64>,
    count: u32,
}

impl OriginAverager {
    /// A zero window takes the first fix alone.
    pub fn new(window: Duration) -> Self {
        OriginAverager {
            window,
            first: None,
            sum: Vector3::zeros(),
            count: 0,
        }
    }

    /// Adds the offset one fix gives: the filter's antenna position (NED) minus the
    /// fix in the GPS frame. Returns the mean offset once the window has passed
    /// (including this fix), `None` while it is still open. A fix in the GPS frame
    /// plus the offset is in the filter's frame.
    pub fn add(&mut self, time: Duration, offset: Vector3<f64>) -> Option<Vector3<f64>> {
        let first = *self.first.get_or_insert(time);

        self.sum += offset;
        self.count += 1;

        (time.saturating_sub(first) >= self.window).then(|| self.sum / f64::from(self.count))
    }

    /// Fixes averaged so far.
    pub fn count(&self) -> u32 {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zero_window_is_the_first_fix() {
        let mut average = OriginAverager::new(Duration::ZERO);
        let offset = Vector3::new(1.0, -2.0, 3.0);

        assert_eq!(average.add(Duration::from_secs(5), offset), Some(offset));
    }

    #[test]
    fn the_offset_is_the_mean_over_the_window_and_waits_for_it() {
        let mut average = OriginAverager::new(Duration::from_secs(2));
        let at = |ms| Duration::from_millis(ms);

        assert_eq!(average.add(at(1000), Vector3::new(1.0, 0.0, 0.0)), None);
        assert_eq!(average.add(at(2000), Vector3::new(3.0, 0.0, 0.0)), None);
        // the fix that closes the window counts: (1 + 3 + 5) / 3
        assert_eq!(
            average.add(at(3000), Vector3::new(5.0, 0.0, 0.0)),
            Some(Vector3::new(3.0, 0.0, 0.0))
        );
        assert_eq!(average.count(), 3);
    }
}

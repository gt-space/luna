use std::time::Duration;

pub(crate) fn fallback_channels() -> std::ops::Range<u32> {
    1..7
}

/// The timer decision used by SAM for each configured abort valve.
pub(crate) fn timer_expired(
    elapsed_since_abort: Duration,
    valve_timer: Duration,
    heartbeat_limit: Duration,
) -> bool {
    if heartbeat_limit.is_zero() {
        elapsed_since_abort >= valve_timer
    } else {
        elapsed_since_abort + heartbeat_limit > valve_timer
    }
}

/// Advances configured valve actions and returns whether all have fired.
pub(crate) fn advance_valves<T>(
    valves: &mut [(T, bool)],
    mut elapsed: impl FnMut() -> Duration,
    heartbeat_limit: Duration,
    timer: impl Fn(&T) -> Duration,
    mut actuate: impl FnMut(&T),
) -> bool {
    let mut pending = false;
    for (valve, fired) in valves {
        if !*fired && timer_expired(elapsed(), timer(valve), heartbeat_limit) {
            actuate(valve);
            *fired = true;
        }
        if !*fired {
            pending = true;
        }
    }
    !pending
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEARTBEAT_LIMIT: Duration = Duration::from_millis(1000);
    const TIMERS_MS: [u64; 6] = [0, 200, 500, 1000, 2000, 5000];

    #[test]
    fn empty_stage_fallback_depowers_all_six_channels() {
        assert_eq!(fallback_channels().collect::<Vec<_>>(), [1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn commanded_abort_timing_matrix() {
        let observed_at_one_ms = [true, false, false, false, false, false];
        for (timer_ms, expected) in TIMERS_MS.into_iter().zip(observed_at_one_ms) {
            assert_eq!(
                timer_expired(
                    Duration::from_millis(1),
                    Duration::from_millis(timer_ms),
                    Duration::ZERO
                ),
                expected,
                "{timer_ms} ms timer"
            );
        }
    }

    #[test]
    fn heartbeat_loss_preserves_one_second_detection_allowance() {
        for timer_ms in TIMERS_MS {
            let timer = Duration::from_millis(timer_ms);
            let expected = timer_ms <= 1000;
            assert_eq!(
                timer_expired(Duration::from_millis(1), timer, HEARTBEAT_LIMIT),
                expected
            );
        }
        assert!(!timer_expired(
            Duration::ZERO,
            Duration::from_millis(1000),
            HEARTBEAT_LIMIT
        ));
        assert!(!timer_expired(
            Duration::from_millis(1000),
            Duration::from_millis(2000),
            HEARTBEAT_LIMIT
        ));
        assert!(timer_expired(
            Duration::from_millis(1001),
            Duration::from_millis(2000),
            HEARTBEAT_LIMIT
        ));
    }

    #[test]
    fn commanded_abort_respects_all_configured_delays() {
        for timer_ms in TIMERS_MS {
            let timer = Duration::from_millis(timer_ms);
            for elapsed_ms in [0, timer_ms.saturating_sub(1), timer_ms, timer_ms + 1] {
                let elapsed = Duration::from_millis(elapsed_ms);
                assert_eq!(
                    timer_expired(elapsed, timer, Duration::ZERO),
                    elapsed >= timer,
                    "commanded abort with {timer_ms} ms valve timer at {elapsed_ms} ms"
                );
            }
        }
    }

    #[test]
    fn valve_state_progression_clear_and_repeat_across_two_sams() {
        let mut sam_a = vec![
            ((1, Duration::from_millis(2000)), false),
            ((2, Duration::from_millis(3000)), false),
        ];
        let mut sam_b = vec![((3, Duration::from_millis(5000)), false)];
        let mut outputs = Vec::new();
        let advance =
            |valves: &mut Vec<((u8, Duration), bool)>, elapsed_ms, outputs: &mut Vec<u8>| {
                advance_valves(
                    valves,
                    || Duration::from_millis(elapsed_ms),
                    HEARTBEAT_LIMIT,
                    |v| v.1,
                    |v| outputs.push(v.0),
                )
            };
        assert!(!advance(&mut sam_a, 1, &mut outputs));
        assert!(!advance(&mut sam_b, 1, &mut outputs));
        assert!(!advance(&mut sam_a, 1100, &mut outputs));
        assert!(!advance(&mut sam_b, 1100, &mut outputs));
        assert!(advance(&mut sam_a, 2001, &mut outputs));
        assert!(advance(&mut sam_b, 4001, &mut outputs));
        assert_eq!(outputs, [1, 2, 3]);
        for (_, fired) in &mut sam_a {
            *fired = false;
        }
        for (_, fired) in &mut sam_b {
            *fired = false;
        }
        outputs.clear();
        assert!(advance(&mut sam_a, 5001, &mut outputs));
        assert!(advance(&mut sam_b, 5001, &mut outputs));
        assert_eq!(outputs, [1, 2, 3]);
    }
}

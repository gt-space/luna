use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

pub(crate) fn required_boards<'a>(
    mappings: impl Iterator<Item = &'a str>,
    previous: impl Iterator<Item = &'a str>,
    next: impl Iterator<Item = &'a str>,
) -> HashSet<String> {
    mappings
        .filter(|id| id.starts_with("sam"))
        .chain(previous)
        .chain(next)
        .map(str::to_string)
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Phase {
    Preparing,
    Activating,
    Confirmed,
    Failed,
    PartiallyApplied,
}

pub(crate) fn unsynchronized(previous: bool, phase: Phase) -> bool {
    match phase {
        Phase::PartiallyApplied => true,
        Phase::Confirmed => false,
        _ => previous,
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum BoardPhase {
    Pending,
    Prepared,
    Activated,
    Rejected,
    Timeout,
}

pub(crate) struct Transition {
    pub revision: u128,
    pub request_id: Option<u128>,
    pub stage: String,
    pub phase: Phase,
    pub boards: HashMap<String, BoardPhase>,
    deadline: Instant,
    last_send: Option<Instant>,
    activation_sent: bool,
}

impl Transition {
    pub fn new(
        revision: u128,
        request_id: Option<u128>,
        stage: String,
        boards: impl IntoIterator<Item = String>,
        now: Instant,
    ) -> Self {
        Self {
            revision,
            request_id,
            stage,
            phase: Phase::Preparing,
            boards: boards
                .into_iter()
                .map(|id| (id, BoardPhase::Pending))
                .collect(),
            deadline: now + Duration::from_secs(5),
            last_send: None,
            activation_sent: false,
        }
    }

    pub fn ack(
        &mut self,
        board: &str,
        revision: u128,
        prepared: bool,
        activated: bool,
        rejected: bool,
    ) {
        if revision != self.revision {
            return;
        }
        let Some(state) = self.boards.get_mut(board) else {
            return;
        };
        if rejected && matches!(self.phase, Phase::Preparing | Phase::Activating) {
            *state = BoardPhase::Rejected;
            self.fail();
        } else if self.phase == Phase::Preparing && prepared && *state == BoardPhase::Pending {
            *state = BoardPhase::Prepared;
        } else if self.phase == Phase::Activating && activated && *state == BoardPhase::Prepared {
            *state = BoardPhase::Activated;
        }
    }

    pub fn fail(&mut self) {
        self.phase = if self.may_have_activated() {
            Phase::PartiallyApplied
        } else {
            Phase::Failed
        };
    }

    pub fn may_have_activated(&self) -> bool {
        self.activation_sent || self.boards.values().any(|state| *state == BoardPhase::Activated)
    }

    pub fn supersede(&mut self) {
        if matches!(self.phase, Phase::Preparing | Phase::Activating) {
            self.fail();
        }
    }

    /// Returns boards requiring a send or retry in the current phase.
    pub fn tick(&mut self, now: Instant) -> Vec<String> {
        if !matches!(self.phase, Phase::Preparing | Phase::Activating) {
            return vec![];
        }
        if now >= self.deadline {
            for state in self.boards.values_mut() {
                if matches!(state, BoardPhase::Pending | BoardPhase::Prepared) {
                    *state = BoardPhase::Timeout;
                }
            }
            self.fail();
            return vec![];
        }
        if self.phase == Phase::Preparing
            && self
                .boards
                .values()
                .all(|state| *state == BoardPhase::Prepared)
        {
            self.phase = Phase::Activating;
            self.last_send = None;
        }
        if self.phase == Phase::Activating
            && self
                .boards
                .values()
                .all(|state| *state == BoardPhase::Activated)
        {
            self.phase = Phase::Confirmed;
            return vec![];
        }
        if self
            .last_send
            .is_some_and(|last| now.duration_since(last) < Duration::from_millis(250))
        {
            return vec![];
        }
        self.last_send = Some(now);
        let recipients: Vec<_> = self.boards
            .iter()
            .filter_map(|(id, state)| {
                let needs_send = match self.phase {
                    Phase::Preparing => *state == BoardPhase::Pending,
                    Phase::Activating => *state == BoardPhase::Prepared,
                    _ => false,
                };
                needs_send.then(|| id.clone())
            })
            .collect();
        if self.phase == Phase::Activating && !recipients.is_empty() {
            self.activation_sent = true;
        }
        recipients
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn partial_activation_latches_until_full_confirmation() {
        let mut critical = unsynchronized(false, Phase::PartiallyApplied);
        for phase in [Phase::Preparing, Phase::Activating, Phase::Failed] {
            critical = unsynchronized(critical, phase);
            assert!(critical);
        }
        assert!(!unsynchronized(critical, Phase::Confirmed));
    }
    fn start(ids: &[&str], now: Instant) -> Transition {
        Transition::new(
            10,
            Some(2),
            "ascent".into(),
            ids.iter().map(|s| s.to_string()),
            now,
        )
    }

    #[test]
    fn multi_sam_success_and_duplicate_stale_out_of_order_ack() {
        let now = Instant::now();
        let mut t = start(&["sam-a", "sam-b"], now);
        assert_eq!(t.tick(now).len(), 2);
        t.ack("sam-a", 9, true, false, false);
        t.ack("sam-a", 10, false, true, false);
        t.ack("unknown", 10, true, false, false);
        assert_eq!(t.boards["sam-a"], BoardPhase::Pending);
        t.ack("sam-a", 10, true, false, false);
        t.ack("sam-a", 10, true, false, false);
        t.ack("sam-b", 10, true, false, false);
        assert_eq!(t.tick(now + Duration::from_millis(1)).len(), 2);
        t.ack("sam-a", 10, false, true, false);
        t.ack("sam-b", 10, false, true, false);
        t.tick(now + Duration::from_millis(2));
        assert_eq!(t.phase, Phase::Confirmed);
    }

    #[test]
    fn offline_and_rejection_prevent_activation() {
        let now = Instant::now();
        let mut t = start(&["sam-a", "sam-offline"], now);
        t.tick(now);
        t.ack("sam-a", 10, true, false, false);
        assert_eq!(t.tick(now + Duration::from_secs(5)).len(), 0);
        assert_eq!(t.phase, Phase::Failed);
        assert_eq!(t.boards["sam-offline"], BoardPhase::Timeout);
        let mut t = start(&["sam-a"], now);
        t.ack("sam-a", 10, false, false, true);
        assert_eq!(t.phase, Phase::Failed);
    }

    #[test]
    fn partial_activation_never_confirms() {
        let now = Instant::now();
        let mut t = start(&["sam-a", "sam-b"], now);
        t.ack("sam-a", 10, true, false, false);
        t.ack("sam-b", 10, true, false, false);
        t.tick(now);
        t.ack("sam-a", 10, false, true, false);
        t.tick(now + Duration::from_secs(5));
        assert_eq!(t.phase, Phase::PartiallyApplied);
    }

    #[test]
    fn missing_activation_ack_is_potentially_partial() {
        let now = Instant::now();
        let mut t = start(&["sam-a", "sam-b"], now);
        t.ack("sam-a", 10, true, false, false);
        t.ack("sam-b", 10, true, false, false);
        assert_eq!(t.tick(now).len(), 2); // ACTIVATE may have reached either SAM.
        t.tick(now + Duration::from_secs(5));
        assert_eq!(t.phase, Phase::PartiallyApplied);
    }

    #[test]
    fn activation_rejection_and_emergency_abort_are_potentially_partial() {
        let now = Instant::now();
        let mut t = start(&["sam-a", "sam-b"], now);
        t.ack("sam-a", 10, true, false, false);
        t.ack("sam-b", 10, true, false, false);
        t.tick(now);
        t.ack("sam-b", 10, false, false, true);
        assert_eq!(t.phase, Phase::PartiallyApplied);
        let mut interrupted = start(&["sam-a"], now);
        interrupted.ack("sam-a", 10, true, false, false);
        interrupted.tick(now);
        interrupted.fail(); // Flight's emergency-abort path.
        assert_eq!(interrupted.phase, Phase::PartiallyApplied);
    }

    #[test]
    fn clearing_a_confirmed_configuration_requires_reconfirmation() {
        let now = Instant::now();
        let mut t = start(&["sam-a"], now);
        t.ack("sam-a", 10, true, false, false);
        t.tick(now);
        t.ack("sam-a", 10, false, true, false);
        t.tick(now + Duration::from_millis(1));
        assert_eq!(t.phase, Phase::Confirmed);
        assert!(t.may_have_activated());
    }

    #[test]
    fn superseding_unacknowledged_activation_preserves_critical_state() {
        let now = Instant::now();
        let mut old = start(&["sam-a"], now);
        old.ack("sam-a", 10, true, false, false);
        old.tick(now);
        old.supersede();
        assert_eq!(old.phase, Phase::PartiallyApplied);
        assert!(unsynchronized(false, old.phase));
    }

    #[test]
    fn removed_and_offline_sams_remain_required() {
        let boards = required_boards(
            ["sam-offline", "bms-1"].into_iter(),
            ["sam-removed"].into_iter(),
            ["sam-new"].into_iter(),
        );
        assert_eq!(boards.len(), 3);
        assert!(
            boards.contains("sam-offline")
                && boards.contains("sam-removed")
                && boards.contains("sam-new")
        );
    }

    #[test]
    fn rapid_new_request_ignores_old_ack() {
        let now = Instant::now();
        let mut current =
            Transition::new(11, Some(3), "landing".into(), ["sam-a".to_string()], now);
        current.ack("sam-a", 10, true, false, false);
        assert_eq!(current.boards["sam-a"], BoardPhase::Pending);
        current.ack("sam-a", 11, true, false, false);
        assert_eq!(current.tick(now).len(), 1);
        assert_eq!(current.phase, Phase::Activating);
    }

    #[test]
    fn emergency_abort_interrupts_prepare_or_partial_activation() {
        let now = Instant::now();
        let mut preparing = start(&["sam-a"], now);
        preparing.fail();
        assert_eq!(preparing.phase, Phase::Failed);
        assert!(preparing.tick(now).is_empty());

        let mut activating = start(&["sam-a", "sam-b"], now);
        activating.ack("sam-a", 10, true, false, false);
        activating.ack("sam-b", 10, true, false, false);
        activating.tick(now);
        activating.ack("sam-a", 10, false, true, false);
        activating.fail();
        assert_eq!(activating.phase, Phase::PartiallyApplied);
        assert!(activating.tick(now).is_empty());
    }
}

/// Revisioned configuration held separately until activation.
#[derive(Clone)]
pub(crate) struct StageTransition<T> {
    latest_revision: Option<u128>,
    active_revision: Option<u128>,
    prepared: Option<(u128, Vec<T>)>,
    pub(crate) acknowledgment: Option<(u128, AckPhase)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AckPhase {
    Prepared,
    Activated,
    Rejected,
}

impl<T> Default for StageTransition<T> {
    fn default() -> Self {
        Self {
            latest_revision: None,
            active_revision: None,
            prepared: None,
            acknowledgment: None,
        }
    }
}

impl<T> StageTransition<T> {
    pub(crate) fn legacy_sync_allowed(&self) -> bool {
        self.latest_revision.is_none()
    }

    pub(crate) fn clear_legacy(&mut self) -> bool {
        if !self.legacy_sync_allowed() {
            return false;
        }
        self.cancel();
        true
    }

    pub(crate) fn prepare(&mut self, revision: u128, valves: Vec<T>) {
        if self
            .latest_revision
            .is_some_and(|latest| revision <= latest)
        {
            return;
        }
        self.latest_revision = Some(revision);
        self.prepared = Some((revision, valves));
        self.acknowledgment = Some((revision, AckPhase::Prepared));
    }

    pub(crate) fn activate(&mut self, revision: u128, aborting: bool) -> Option<Vec<T>> {
        if self.active_revision == Some(revision) {
            self.acknowledgment = Some((revision, AckPhase::Activated));
            return None;
        }
        if self.prepared.as_ref().is_none_or(|(id, _)| *id != revision) {
            return None;
        }
        if aborting {
            self.prepared = None;
            self.acknowledgment = Some((revision, AckPhase::Rejected));
            return None;
        }
        let (_, valves) = self.prepared.take().unwrap();
        self.active_revision = Some(revision);
        self.acknowledgment = Some((revision, AckPhase::Activated));
        Some(valves)
    }

    pub(crate) fn abort(&mut self) {
        if let Some((revision, _)) = self.prepared.take() {
            self.acknowledgment = Some((revision, AckPhase::Rejected));
        }
    }

    pub(crate) fn cancel(&mut self) {
        self.prepared = None;
        self.active_revision = None;
        self.acknowledgment = None;
    }

    pub(crate) fn clear(&mut self, revision: u128) -> bool {
        if self
            .latest_revision
            .is_some_and(|latest| revision <= latest)
        {
            return false;
        }
        self.latest_revision = Some(revision);
        self.cancel();
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prepare_does_not_change_active_and_empty_activation_clears_it() {
        let mut stage = StageTransition::default();
        let mut active = vec![1, 2];
        stage.prepare(1, vec![]);
        assert_eq!(active, [1, 2]);
        assert_eq!(stage.acknowledgment, Some((1, AckPhase::Prepared)));
        active = stage.activate(1, false).unwrap();
        assert!(active.is_empty()); // production safe_valves depowers all channels on empty.
        assert_eq!(stage.acknowledgment, Some((1, AckPhase::Activated)));
    }

    #[test]
    fn duplicate_and_reordered_commands_do_not_rollback() {
        let mut stage = StageTransition::default();
        stage.prepare(20, vec![2]);
        stage.prepare(20, vec![9]);
        assert_eq!(stage.activate(19, false), None);
        assert_eq!(stage.activate(20, false), Some(vec![2]));
        stage.prepare(19, vec![9]);
        assert_eq!(stage.activate(19, false), None);
        assert_eq!(stage.activate(20, false), None);
        assert_eq!(stage.acknowledgment, Some((20, AckPhase::Activated)));
        stage.prepare(21, vec![3]);
        stage.prepare(20, vec![9]);
        assert_eq!(stage.activate(20, false), None);
        assert_eq!(stage.activate(21, false), Some(vec![3]));
    }

    #[test]
    fn abort_rejects_prepared_stage_without_changing_active() {
        let mut stage = StageTransition::default();
        stage.prepare(7, vec![1]);
        stage.abort();
        assert_eq!(stage.activate(7, false), None);
        assert_eq!(stage.acknowledgment, Some((7, AckPhase::Rejected)));
        stage.prepare(8, vec![2]);
        assert_eq!(stage.activate(8, true), None);
        assert_eq!(stage.acknowledgment, Some((8, AckPhase::Rejected)));
    }

    #[test]
    fn clearing_stored_stage_cancels_pending_transition() {
        let mut stage = StageTransition::default();
        stage.prepare(4, vec![1]);
        stage.cancel();
        assert_eq!(stage.activate(4, false), None);
        assert_eq!(stage.acknowledgment, None);
    }

    #[test]
    fn legacy_sync_is_blocked_after_v2_prepare() {
        let mut stage = StageTransition::<u8>::default();
        assert!(stage.legacy_sync_allowed());
        stage.prepare(1, vec![1]);
        assert!(!stage.legacy_sync_allowed());
        stage.cancel();
        assert!(!stage.legacy_sync_allowed());
    }

    #[test]
    fn clear_before_newer_prepare_and_activate() {
        let mut stage = StageTransition::default();
        assert!(stage.clear(1));
        stage.prepare(2, vec![7]);
        assert_eq!(stage.activate(2, false), Some(vec![7]));
        assert_eq!(stage.acknowledgment, Some((2, AckPhase::Activated)));
    }

    #[test]
    fn delayed_or_lost_clear_cannot_erase_newer_activation() {
        let mut stage = StageTransition::default();
        stage.prepare(2, vec![7]);
        assert_eq!(stage.activate(2, false), Some(vec![7]));
        assert!(!stage.clear(1));
        assert!(!stage.clear(2));
        assert_eq!(stage.acknowledgment, Some((2, AckPhase::Activated)));
        stage.prepare(3, vec![8]); // A lost clear is superseded by the next revision.
        assert_eq!(stage.activate(3, false), Some(vec![8]));
    }

    #[test]
    fn clear_invalidates_duplicate_activation_ack() {
        let mut stage = StageTransition::default();
        stage.prepare(2, vec![7]);
        assert_eq!(stage.activate(2, false), Some(vec![7]));
        assert!(stage.clear(3));
        assert_eq!(stage.activate(2, false), None);
        assert_eq!(stage.acknowledgment, None);
        assert!(!stage.clear(1));
        stage.prepare(4, vec![]);
        assert_eq!(stage.activate(4, false), Some(vec![]));
    }

    #[test]
    fn legacy_clear_is_allowed_only_before_revisioned_commands() {
        let mut stage = StageTransition::default();
        assert!(stage.clear_legacy());
        stage.prepare(1, vec![7]);
        assert_eq!(stage.activate(1, false), Some(vec![7]));
        assert!(!stage.clear_legacy());
        assert_eq!(stage.acknowledgment, Some((1, AckPhase::Activated)));
    }
}

/// Revisioned configuration held separately until activation.
#[derive(Clone)]
pub(crate) struct StageTransition<T> {
    latest_revision: Option<u128>,
    active_revision: Option<u128>,
    prepared: Option<(u128, Vec<T>)>,
    pub(crate) acknowledgment: Option<(u128, AckPhase)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AckPhase { Prepared, Activated, Rejected }

impl<T> Default for StageTransition<T> {
    fn default() -> Self {
        Self { latest_revision: None, active_revision: None, prepared: None, acknowledgment: None }
    }
}

impl<T> StageTransition<T> {
    pub(crate) fn prepare(&mut self, revision: u128, valves: Vec<T>) {
        if self.latest_revision.is_some_and(|latest| revision <= latest) { return; }
        self.latest_revision = Some(revision);
        self.prepared = Some((revision, valves));
        self.acknowledgment = Some((revision, AckPhase::Prepared));
    }

    pub(crate) fn activate(&mut self, revision: u128, aborting: bool) -> Option<Vec<T>> {
        if self.active_revision == Some(revision) {
            self.acknowledgment = Some((revision, AckPhase::Activated));
            return None;
        }
        if self.prepared.as_ref().is_none_or(|(id, _)| *id != revision) { return None; }
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
        self.acknowledgment = None;
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
    fn legacy_configuration_cancels_pending_transition() {
        let mut stage = StageTransition::default();
        stage.prepare(4, vec![1]);
        stage.cancel();
        assert_eq!(stage.activate(4, false), None);
        assert_eq!(stage.acknowledgment, None);
    }
}

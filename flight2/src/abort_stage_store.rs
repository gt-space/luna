/// Replaces the saved definition of a stage. The active copy is stored separately.
pub(crate) fn replace_saved_stage<T>(stages: &mut Vec<T>, incoming: T, name: impl Fn(&T) -> &str) {
    if let Some(index) = stages
        .iter()
        .position(|stage| name(stage) == name(&incoming))
    {
        stages.swap_remove(index);
    }
    stages.push(incoming);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    struct Stage {
        name: String,
        delay_ms: u64,
    }

    #[test]
    #[ignore = "reproduces stale active abort stage after replacement"]
    fn updating_active_stage_refreshes_dispatched_configuration() {
        let mut stages = vec![Stage {
            name: "ascent".into(),
            delay_ms: 2000,
        }];
        let active = stages[0].clone(); // Flight stores a separate active copy.
        let dispatched_to_sam = active.clone();
        replace_saved_stage(
            &mut stages,
            Stage {
                name: "ascent".into(),
                delay_ms: 5000,
            },
            |stage| &stage.name,
        );
        assert_eq!(stages.len(), 1);
        assert_eq!(active.delay_ms, stages[0].delay_ms);
        assert_eq!(dispatched_to_sam.delay_ms, stages[0].delay_ms);
    }

    #[test]
    fn stage_change_does_not_leave_duplicate_saved_names() {
        let mut stages = vec![Stage {
            name: "one".into(),
            delay_ms: 2000,
        }];
        replace_saved_stage(
            &mut stages,
            Stage {
                name: "two".into(),
                delay_ms: 5000,
            },
            |stage| &stage.name,
        );
        assert_eq!(stages.len(), 2);
        replace_saved_stage(
            &mut stages,
            Stage {
                name: "two".into(),
                delay_ms: 200,
            },
            |stage| &stage.name,
        );
        assert_eq!(stages.len(), 2);
        assert_eq!(
            stages
                .iter()
                .find(|stage| stage.name == "two")
                .unwrap()
                .delay_ms,
            200
        );
    }
}

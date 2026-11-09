/// Flight stops user sequences when sending an abort, retaining its abort monitor.
pub(crate) fn should_stop_on_abort(name: &str) -> bool {
    name != "AbortStage"
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abort_stops_running_sequence_but_keeps_abort_monitor() {
        assert!(should_stop_on_abort("Launch"));
        assert!(should_stop_on_abort("Ignition"));
        assert!(!should_stop_on_abort("AbortStage"));
    }
}

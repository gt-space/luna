#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Outcome {
    Continue,
    Confirmed,
    Failed,
    Superseded,
}

pub(crate) fn outcome(
    expected: u128,
    observed: Option<u128>,
    phase: &str,
    saw_expected: bool,
) -> Outcome {
    if observed == Some(expected) {
        return match phase {
            "confirmed" => Outcome::Confirmed,
            "failed" | "partially_applied" => Outcome::Failed,
            _ => Outcome::Continue,
        };
    }
    if saw_expected {
        Outcome::Superseded
    } else {
        Outcome::Continue
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn only_matching_confirmed_request_succeeds() {
        assert_eq!(outcome(2, Some(1), "confirmed", false), Outcome::Continue);
        assert_eq!(outcome(2, Some(2), "preparing", true), Outcome::Continue);
        assert_eq!(outcome(2, Some(2), "confirmed", true), Outcome::Confirmed);
        assert_eq!(outcome(2, Some(2), "failed", true), Outcome::Failed);
        assert_eq!(
            outcome(2, Some(2), "partially_applied", true),
            Outcome::Failed
        );
        assert_eq!(outcome(2, Some(3), "confirmed", true), Outcome::Superseded);
    }
}

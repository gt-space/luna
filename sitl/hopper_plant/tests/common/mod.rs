//! Shared by the integration tests.
//!
//! These tests only mean something against the generated plant. A build
//! without it (HOPPER_PLANT_CODE unset) used to make them pass, which looks
//! like a green result for a plant that was never run. They now fail unless
//! the caller says skipping is fine with `HOPPER_PLANT_ALLOW_SKIP=1`.

/// Called when the test can't run because something it needs is missing.
/// Returns (so the test can return early) only if skipping was allowed.
pub fn cannot_run(what: &str) {
    if std::env::var_os("HOPPER_PLANT_ALLOW_SKIP").is_some() {
        eprintln!("SKIPPED: {what} (HOPPER_PLANT_ALLOW_SKIP is set)");
        return;
    }
    panic!(
        "{what}. This test needs the generated plant and cannot pass without it. \
         Set HOPPER_PLANT_ALLOW_SKIP=1 to skip it on purpose."
    );
}

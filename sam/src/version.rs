use std::sync::LazyLock;
use crate::communication; // where get_version() is defined

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamVersion {
    Rev3,
    Rev4Ground,
    Rev4Flight,
}

#[cfg(any(test, feature = "python"))]
fn get_version_mock() -> SamVersion {
    // Mocked version for tests or Python
    SamVersion::Rev4Ground
}

#[cfg(not(any(test, feature = "python")))]
fn get_version_real() -> SamVersion {
    communication::get_version()
}

// Select which one to use depending on build context
pub static SAM_VERSION: LazyLock<SamVersion> = LazyLock::new(|| {
    #[cfg(any(test, feature = "python"))]
    {
        get_version_mock()
    }
    #[cfg(not(any(test, feature = "python")))]
    {
        get_version_real()
    }
});

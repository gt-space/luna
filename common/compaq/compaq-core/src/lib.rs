pub mod compress;

pub type Result<T> = ::core::result::Result<T, CompaqError>;

pub enum CompaqError {
    DesynchronizedPolicy
}
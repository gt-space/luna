use axum::{http::StatusCode, response::IntoResponse};

/// Any error that the server can throw in a route function.
#[derive(Debug)]
pub enum ServerError {
  /// Error originating from a SQL query.
  Sql(rusqlite::Error),

  /// Error that may be converted directly into a `Response`.
  Raw(String, StatusCode),
}

impl From<rusqlite::Error> for ServerError {
  fn from(error: rusqlite::Error) -> Self {
    ServerError::Sql(error)
  }
}

impl IntoResponse for ServerError {
  fn into_response(self) -> axum::response::Response {
    match self {
      Self::Sql(error) => {
        (StatusCode::INTERNAL_SERVER_ERROR, error.to_string())
      }
      Self::Raw(message, status) => (status, message),
    }
    .into_response()
  }
}

/// A `Result` type containing a `ServerError` as its `Err` variant.
pub type ServerResult<T> = Result<T, ServerError>;

/// Converts any arbitrary error type into a standardized `ServerError::Raw`for
/// a bad request.
pub fn bad_request(message: impl ToString) -> ServerError {
  ServerError::Raw(message.to_string(), StatusCode::BAD_REQUEST)
}

/// Converts any arbitrary error type into a standardized `ServerError::Raw`
/// when a resource is not found.
pub fn not_found(message: impl ToString) -> ServerError {
  ServerError::Raw(message.to_string(), StatusCode::NOT_FOUND)
}

/// Converts any arbitrary error type into a standardized internal
/// `ServerError`.
pub fn internal(message: impl ToString) -> ServerError {
  ServerError::Raw(message.to_string(), StatusCode::INTERNAL_SERVER_ERROR)
}

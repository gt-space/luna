use serde::Deserialize;
use serde::Serialize;

#[derive(Serialize)]
pub struct AuthRequest {
    pub username: String,
    pub password: String
}

#[derive(Deserialize)]
pub struct AuthResponse {
    pub session_id: String
}
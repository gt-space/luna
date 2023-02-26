use std::{net::{UdpSocket, SocketAddr, Ipv4Addr}, error::Error};
use crate::auth::{AuthRequest, AuthResponse};
use serde::Serialize;
use reqwest::{Client, Response};

impl Connection {
  pub async fn send_auth_req(&self, request: AuthRequest) -> Result<AuthResponse, reqwest::Error> {
    let response = self.client.clone().unwrap_or(Client::new()).post(format!(
        "http://{}:{}/auth", 
        self.server_ip.unwrap_or(Ipv4Addr::new(0, 0, 0, 0)).to_string(), 
        self.server_port.unwrap_or(0)))
      .json(&request)
      .send()
      .await;

    return match response {
        Ok(res) => res.json::<AuthResponse>().await,
        Err(err) => Err(err),
    };
  }
}

#[derive(Debug)]
pub struct Connection {
  pub self_ip: Option<Ipv4Addr>,
  pub self_port: Option<u16>,
  pub server_ip: Option<Ipv4Addr>,
  pub server_port: Option<u16>,
  pub forwarding_id: Option<String>,
  pub session_id: Option<String>,
  pub socket: Option<UdpSocket>,
  pub client: Option<Client>
}

#[derive(Serialize)]
pub struct LedCommand {

}
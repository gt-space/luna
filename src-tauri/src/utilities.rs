use std::net::Ipv4Addr;

// UNSAFE CODE
pub fn parseIpFromString(ip: &str) -> Ipv4Addr {
  let mut split_ip: [u8;4] = [0,0,0,0];
  let mut count = 0;
  for section in ip.split(".") {
    split_ip[count] = section.parse().unwrap();
    count += 1;
  }
  return Ipv4Addr::new(split_ip[0], split_ip[1], split_ip[2], split_ip[3]);
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Alert {
  pub time: String,
  pub agent: String,
  pub message: String,
}
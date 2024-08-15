use std::collections::HashMap;
use std::net::IpAddr;

pub fn get_ips(hostnames: &[&str]) -> HashMap<String, Option<IpAddr>> {
  let mut ips: HashMap<String, Option<IpAddr>> = HashMap::new();
  for hostname in hostnames {
    let ip = dns_lookup::lookup_host(hostname);
    ips.insert(
      hostname.to_string(),
      ip.ok().and_then(|ip| ip.first().copied()),
    );
  }
  ips
}

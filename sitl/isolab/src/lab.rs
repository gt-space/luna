use crate::process::run_command;
use anyhow::Result;
use std::{
  fs,
  path::{Path, PathBuf},
  process::{Command, Stdio},
};

pub struct Lab {
  pub workdir: PathBuf,
}

pub struct VethPair<'a> {
  pub left: &'a str,
  pub right: &'a str,
}

pub struct BridgeNode<'a> {
  pub namespace: &'a str,
  pub namespace_if: &'a str,
  pub peer_if: &'a str,
  pub bridge: &'a str,
  pub addr: &'a str,
}

pub struct ControlLink<'a> {
  pub root_if: &'a str,
  pub namespace: &'a str,
  pub namespace_if: &'a str,
  pub root_addr: &'a str,
  pub namespace_addr: &'a str,
}

impl Lab {
  pub fn new(workdir: PathBuf) -> Result<Self> {
    if workdir.exists() {
      fs::remove_dir_all(&workdir).ok();
    }
    fs::create_dir_all(&workdir)?;
    Ok(Self { workdir })
  }

  pub fn disable_bridge_netfilter(&self) {
    for (path, sysctl) in [
      (
        "/proc/sys/net/bridge/bridge-nf-call-iptables",
        "net.bridge.bridge-nf-call-iptables=0",
      ),
      (
        "/proc/sys/net/bridge/bridge-nf-call-arptables",
        "net.bridge.bridge-nf-call-arptables=0",
      ),
      (
        "/proc/sys/net/bridge/bridge-nf-call-ip6tables",
        "net.bridge.bridge-nf-call-ip6tables=0",
      ),
    ] {
      if Path::new(path).exists() {
        let _ = Command::new("sysctl")
          .args(["-q", "-w", sysctl])
          .stdout(Stdio::null())
          .stderr(Stdio::null())
          .status();
      }
    }
  }

  pub fn create_namespace(&self, namespace: &str) -> Result<()> {
    run_command(&["ip", "netns", "add", namespace])?;
    run_command(&["ip", "netns", "exec", namespace, "ip", "link", "set", "lo", "up"])
  }

  pub fn create_bridge(&self, bridge: &str) -> Result<()> {
    run_command(&["ip", "link", "add", bridge, "type", "bridge", "stp_state", "0", "forward_delay", "0"])?;
    run_command(&["ip", "link", "set", bridge, "up"])
  }

  pub fn attach_bridge_node(&self, node: BridgeNode<'_>) -> Result<()> {
    run_command(&["ip", "link", "add", node.peer_if, "type", "veth", "peer", "name", node.namespace_if])?;
    run_command(&["ip", "link", "set", node.namespace_if, "netns", node.namespace])?;
    run_command(&["ip", "link", "set", node.peer_if, "master", node.bridge])?;
    run_command(&["ip", "link", "set", node.peer_if, "up"])?;
    run_command(&["ip", "netns", "exec", node.namespace, "ip", "link", "set", node.namespace_if, "name", "eth0"])?;
    run_command(&["ip", "netns", "exec", node.namespace, "ip", "addr", "add", node.addr, "dev", "eth0"])?;
    run_command(&["ip", "netns", "exec", node.namespace, "ip", "link", "set", "eth0", "up"])
  }

  pub fn create_bridge_link(&self, bridge_a: &str, bridge_b: &str, pair: VethPair<'_>) -> Result<()> {
    run_command(&["ip", "link", "add", pair.left, "type", "veth", "peer", "name", pair.right])?;
    run_command(&["ip", "link", "set", pair.left, "master", bridge_a])?;
    run_command(&["ip", "link", "set", pair.right, "master", bridge_b])?;
    run_command(&["ip", "link", "set", pair.left, "up"])?;
    run_command(&["ip", "link", "set", pair.right, "up"])
  }

  pub fn create_namespaced_veth(&self, left_ns: &str, right_ns: &str, pair: VethPair<'_>) -> Result<()> {
    run_command(&["ip", "link", "add", pair.left, "type", "veth", "peer", "name", pair.right])?;
    run_command(&["ip", "link", "set", pair.left, "netns", left_ns])?;
    run_command(&["ip", "link", "set", pair.right, "netns", right_ns])
  }

  pub fn rename_interface(&self, namespace: &str, from: &str, to: &str) -> Result<()> {
    run_command(&["ip", "netns", "exec", namespace, "ip", "link", "set", from, "name", to])
  }

  pub fn set_interface_mtu(&self, namespace: &str, interface: &str, mtu: &str) -> Result<()> {
    run_command(&["ip", "netns", "exec", namespace, "ip", "link", "set", interface, "mtu", mtu])
  }

  pub fn add_addr(&self, namespace: &str, addr: &str, interface: &str) -> Result<()> {
    run_command(&["ip", "netns", "exec", namespace, "ip", "addr", "add", addr, "dev", interface])
  }

  pub fn set_link_up(&self, namespace: &str, interface: &str) -> Result<()> {
    run_command(&["ip", "netns", "exec", namespace, "ip", "link", "set", interface, "up"])
  }

  pub fn create_control_link(&self, link: ControlLink<'_>) -> Result<()> {
    run_command(&["ip", "link", "add", link.root_if, "type", "veth", "peer", "name", link.namespace_if])?;
    run_command(&["ip", "link", "set", link.namespace_if, "netns", link.namespace])?;
    run_command(&["ip", "addr", "add", link.root_addr, "dev", link.root_if])?;
    run_command(&["ip", "link", "set", link.root_if, "up"])?;
    self.rename_interface(link.namespace, link.namespace_if, "ctl0")?;
    self.add_addr(link.namespace, link.namespace_addr, "ctl0")?;
    self.set_link_up(link.namespace, "ctl0")
  }

  pub fn toggle_link_pair(&self, pair: VethPair<'_>, up: bool) -> Result<()> {
    let state = if up { "up" } else { "down" };
    run_command(&["ip", "link", "set", pair.left, state])?;
    run_command(&["ip", "link", "set", pair.right, state])
  }

  pub fn enable_ipv4_forwarding(&self, namespace: &str) -> Result<()> {
    run_command(&["ip", "netns", "exec", namespace, "sysctl", "-q", "-w", "net.ipv4.ip_forward=1"])
  }

  pub fn exec(&self, args: &[&str]) -> Result<()> {
    run_command(args)
  }

  pub fn cleanup_links(&self, links: &[&str]) {
    for link in links {
      let _ = Command::new("ip")
        .args(["link", "del", link])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    }
  }

  pub fn cleanup_namespaces(&self, namespaces: &[&str]) {
    for namespace in namespaces {
      let _ = Command::new("ip")
        .args(["netns", "del", namespace])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    }
  }
}

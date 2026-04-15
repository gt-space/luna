use crate::lab::{BridgeNode, ControlLink, Lab, VethPair};
use anyhow::Result;
use std::{path::PathBuf, process::{Command, Stdio}};

pub const NS_FLIGHT: &str = "flight";
pub const NS_SERVO: &str = "servo";
pub const NS_FTEL: &str = "ftel";
pub const NS_GTEL: &str = "gtel";
const BR_ROCKET: &str = "br-rocket";
const BR_GROUND: &str = "br-ground";
const UMBILICAL_LEFT: &str = "umb-rkt";
const UMBILICAL_RIGHT: &str = "umb-gnd";

pub const SERVO_IP: &str = "192.168.1.10";
pub const FLIGHT_IP: &str = "192.168.1.11";
const FTEL_IP: &str = "192.168.1.132";
const GTEL_IP: &str = "192.168.1.140";

const FLIGHT_CTL_ROOT_IP: &str = "172.20.0.1/30";
const FLIGHT_CTL_NS_IP: &str = "172.20.0.2/30";
const SERVO_CTL_ROOT_IP: &str = "172.20.0.5/30";
const SERVO_CTL_NS_IP: &str = "172.20.0.6/30";

const RULE_PRIORITY: &str = "246";
const FWMARK: &str = "246";
const TABLE: &str = "246";
const RADIO_DSCP: &str = "46";

pub struct ServoFlightLab {
  pub inner: Lab,
}

impl ServoFlightLab {
  pub fn new(workdir: PathBuf) -> Result<Self> {
    Ok(Self {
      inner: Lab::new(workdir)?,
    })
  }

  pub fn setup(&self) -> Result<()> {
    self.cleanup();
    self.inner.disable_bridge_netfilter();

    for namespace in [NS_FLIGHT, NS_SERVO, NS_FTEL, NS_GTEL] {
      self.inner.create_namespace(namespace)?;
    }

    for bridge in [BR_ROCKET, BR_GROUND] {
      self.inner.create_bridge(bridge)?;
    }

    for node in [
      BridgeNode {
        namespace: NS_FLIGHT,
        namespace_if: "flight-eth",
        peer_if: "p-flight",
        bridge: BR_ROCKET,
        addr: &format!("{FLIGHT_IP}/24"),
      },
      BridgeNode {
        namespace: NS_FTEL,
        namespace_if: "ftel-eth",
        peer_if: "p-ftel",
        bridge: BR_ROCKET,
        addr: &format!("{FTEL_IP}/24"),
      },
      BridgeNode {
        namespace: NS_SERVO,
        namespace_if: "servo-eth",
        peer_if: "p-servo",
        bridge: BR_GROUND,
        addr: &format!("{SERVO_IP}/24"),
      },
      BridgeNode {
        namespace: NS_GTEL,
        namespace_if: "gtel-eth",
        peer_if: "p-gtel",
        bridge: BR_GROUND,
        addr: &format!("{GTEL_IP}/24"),
      },
    ] {
      self.inner.attach_bridge_node(node)?;
    }

    self.inner.create_bridge_link(
      BR_ROCKET,
      BR_GROUND,
      VethPair {
        left: UMBILICAL_LEFT,
        right: UMBILICAL_RIGHT,
      },
    )?;

    self.inner.create_namespaced_veth(
      NS_FTEL,
      NS_GTEL,
      VethPair {
        left: "ftel-radio",
        right: "gtel-radio",
      },
    )?;
    self.inner.rename_interface(NS_FTEL, "ftel-radio", "radio0")?;
    self.inner.rename_interface(NS_GTEL, "gtel-radio", "radio0")?;
    self.inner.set_interface_mtu(NS_FTEL, "radio0", "255")?;
    self.inner.set_interface_mtu(NS_GTEL, "radio0", "255")?;
    self.inner.add_addr(NS_FTEL, "10.8.8.0/31", "radio0")?;
    self.inner.add_addr(NS_GTEL, "10.8.8.1/31", "radio0")?;
    self.inner.set_link_up(NS_FTEL, "radio0")?;
    self.inner.set_link_up(NS_GTEL, "radio0")?;

    for link in [
      ControlLink {
        root_if: "host-flight",
        namespace: NS_FLIGHT,
        namespace_if: "flight-ctl",
        root_addr: FLIGHT_CTL_ROOT_IP,
        namespace_addr: FLIGHT_CTL_NS_IP,
      },
      ControlLink {
        root_if: "host-servo",
        namespace: NS_SERVO,
        namespace_if: "servo-ctl",
        root_addr: SERVO_CTL_ROOT_IP,
        namespace_addr: SERVO_CTL_NS_IP,
      },
    ] {
      self.inner.create_control_link(link)?;
    }

    for namespace in [NS_FTEL, NS_GTEL] {
      self.inner.enable_ipv4_forwarding(namespace)?;
    }

    self.inner.exec(&[
      "ip", "netns", "exec", NS_FTEL, "iptables", "-t", "nat", "-A", "PREROUTING",
      "-s", FLIGHT_IP, "-d", SERVO_IP, "-m", "dscp", "--dscp", RADIO_DSCP,
      "-j", "DNAT", "--to-destination", "10.8.8.1",
    ])?;
    self.inner.exec(&[
      "ip", "netns", "exec", NS_FTEL, "iptables", "-t", "nat", "-A", "POSTROUTING",
      "-o", "radio0", "-j", "SNAT", "--to-source", "10.8.8.0",
    ])?;
    self.inner.exec(&[
      "ip", "netns", "exec", NS_GTEL, "iptables", "-t", "nat", "-A", "PREROUTING",
      "-i", "radio0", "-j", "DNAT", "--to-destination", SERVO_IP,
    ])?;
    self.inner.exec(&[
      "ip", "netns", "exec", NS_GTEL, "iptables", "-t", "nat", "-A", "POSTROUTING",
      "-o", "eth0", "-d", SERVO_IP, "-j", "SNAT", "--to-source", GTEL_IP,
    ])?;

    self.inner.exec(&[
      "ip", "netns", "exec", NS_FLIGHT, "ip", "route", "replace", "table", TABLE,
      &format!("{SERVO_IP}/32"), "via", FTEL_IP, "dev", "eth0", "src", FLIGHT_IP,
    ])?;
    self.inner.exec(&["ip", "netns", "exec", NS_FLIGHT, "iptables", "-t", "mangle", "-F", "OUTPUT"])?;
    self.inner.exec(&[
      "ip", "netns", "exec", NS_FLIGHT, "iptables", "-t", "mangle", "-A", "OUTPUT",
      "-d", SERVO_IP, "-m", "dscp", "--dscp", RADIO_DSCP,
      "-j", "MARK", "--set-mark", FWMARK,
    ])?;
    let _ = Command::new("ip")
      .args(["netns", "exec", NS_FLIGHT, "ip", "rule", "del", "priority", RULE_PRIORITY])
      .stdout(Stdio::null())
      .stderr(Stdio::null())
      .status();
    self.inner.exec(&[
      "ip", "netns", "exec", NS_FLIGHT, "ip", "rule", "add", "priority", RULE_PRIORITY,
      "fwmark", FWMARK, "lookup", TABLE,
    ])?;
    self.inner.exec(&["ip", "netns", "exec", NS_FLIGHT, "ip", "route", "flush", "cache"])?;

    Ok(())
  }

  pub fn toggle_umbilical(&self, up: bool) -> Result<()> {
    self.inner.toggle_link_pair(
      VethPair {
        left: UMBILICAL_LEFT,
        right: UMBILICAL_RIGHT,
      },
      up,
    )
  }

  pub fn cleanup(&self) {
    self.inner.cleanup_links(&[BR_ROCKET, BR_GROUND, UMBILICAL_LEFT, "host-flight", "host-servo"]);
    self.inner.cleanup_namespaces(&[NS_FLIGHT, NS_SERVO, NS_FTEL, NS_GTEL]);
  }
}

impl Drop for ServoFlightLab {
  fn drop(&mut self) {
    self.cleanup();
  }
}

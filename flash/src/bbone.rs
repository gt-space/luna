mod bootp;
mod tftp;

use indicatif::{ProgressBar, ProgressStyle};
use log::info;
use nix::sched::CloneFlags;
use nix::sys::signal::{self, SigHandler, Signal};
use std::{
  collections::{HashMap, HashSet},
  fs::{self, File},
  io::{self, Read, Write},
  net::Ipv4Addr,
  os::unix::fs::OpenOptionsExt,
  path::Path,
  process::{Command, Stdio},
  thread,
  time::Duration,
};

const SERVER_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 0, 1);
const CLIENT_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 0, 2);

pub fn flash(spl_path: &Path, uboot_path: &Path, image_path: &Path) {
  let spl = fs::read(spl_path).expect("failed to read SPL");
  let uboot = fs::read(uboot_path).expect("failed to read U-Boot");

  info!(
    "loaded spl ({} bytes), uboot ({} bytes)",
    spl.len(), uboot.len(),
  );

  let files = HashMap::from([
    ("spl".to_string(), spl.into_boxed_slice()),
    ("uboot".to_string(), uboot.into_boxed_slice()),
  ]);

  let ns = Namespace::new("flash").expect("failed to create network namespace");

  // Enter the flash namespace so the BOOTP and TFTP sockets are bound
  // inside it, completely isolated from the host's port 67 / 69.
  ns.enter().expect("failed to enter flash namespace");

  let tftp = tftp::Server::new(files).expect("failed to start TFTP server");
  let bootp = bootp::Server::new(SERVER_IP, CLIENT_IP).expect("failed to start BOOTP server");

  // Return to the default namespace so we can detect new USB interfaces
  // as they appear on the host.
  ns.exit().expect("failed to exit flash namespace");

  // Phase 1: Serve SPL and U-Boot via BOOTP+TFTP.
  let stages = [("spl", "SPL"), ("uboot", "U-Boot")];
  let mut prev_iface: Option<String> = None;

  for (i, (filename, label)) in stages.iter().enumerate() {
    if let Some(ref iface) = prev_iface {
      info!("waiting for {iface} to disconnect...");
      ns.wait_for_disconnect(iface);
    }

    let iface = detect_interface();
    ns.adopt_interface(&iface);
    ns.configure_interface(&iface);

    // Enter the flash namespace for the serving phase.  The BOOTP/TFTP
    // server sockets are already bound here, but the TFTP Transfer also
    // opens an ephemeral socket — it must land in this namespace too.
    ns.enter().expect("failed to enter flash namespace");

    info!("stage {}/3: serving {label}...", i + 1);
    bootp.respond(filename).expect("BOOTP exchange failed");
    tftp.serve().expect("TFTP transfer failed");

    ns.exit().expect("failed to exit flash namespace");

    prev_iface = Some(iface);
  }

  // Phase 2: Wait for U-Boot to expose eMMC as USB mass storage, then
  // write the image directly to the block device.
  //
  // Snapshot block devices BEFORE waiting for disconnect: the UMS gadget
  // may appear at the same instant the network interface disappears.
  let block_baseline = block_devices();

  if let Some(ref iface) = prev_iface {
    info!("waiting for {iface} to disconnect...");
    ns.wait_for_disconnect(iface);
  }

  info!("stage 3/3: waiting for USB mass storage device...");
  let device = detect_block_device(&block_baseline);
  info!("writing image to /dev/{device}...");
  write_image(image_path, &device);

  info!("flash complete");
}

// ---------------------------------------------------------------------------
// Network namespace
// ---------------------------------------------------------------------------

/// A Linux network namespace used to isolate the BOOTP/TFTP servers from the
/// host network stack, avoiding port conflicts with services like dnsmasq.
///
/// The namespace is created as a named namespace (visible via `ip netns list`)
/// so that interfaces can be moved into it by name.  The calling process
/// switches between the default and flash namespaces using `setns(2)`.
struct Namespace {
  name: String,
  default_ns: fs::File,
  flash_ns: fs::File,
}

impl Namespace {
  fn new(name: &str) -> io::Result<Self> {
    // Hold an fd to the default namespace so we can return to it later.
    let default_ns = fs::File::open("/proc/self/ns/net")?;

    // Clean up a stale namespace from a previous crashed run.
    let _ = Command::new("ip")
      .args(["netns", "delete", name])
      .stdout(Stdio::null())
      .stderr(Stdio::null())
      .status();

    assert!(
      Command::new("ip")
        .args(["netns", "add", name])
        .status()?
        .success(),
      "failed to create network namespace (are you root?)",
    );

    let flash_ns = fs::File::open(format!("/var/run/netns/{name}"))?;

    // Ensure the namespace is cleaned up on Ctrl+C / SIGTERM.
    let handler = SigHandler::Handler(cleanup_namespace);
    unsafe {
      signal::signal(Signal::SIGINT, handler).expect("failed to set SIGINT handler");
      signal::signal(Signal::SIGTERM, handler).expect("failed to set SIGTERM handler");
    }

    info!("created network namespace: {name}");
    Ok(Self { name: name.to_string(), default_ns, flash_ns })
  }

  /// Switch the calling thread into the flash namespace.
  fn enter(&self) -> io::Result<()> {
    nix::sched::setns(&self.flash_ns, CloneFlags::CLONE_NEWNET)?;
    Ok(())
  }

  /// Switch the calling thread back to the default namespace.
  fn exit(&self) -> io::Result<()> {
    nix::sched::setns(&self.default_ns, CloneFlags::CLONE_NEWNET)?;
    Ok(())
  }

  /// Move an interface from the default namespace into this namespace.
  fn adopt_interface(&self, iface: &str) {
    assert!(
      Command::new("ip")
        .args(["link", "set", iface, "netns", &self.name])
        .status()
        .expect("failed to run `ip`")
        .success(),
      "failed to move {iface} into namespace {}",
      self.name,
    );
    info!("moved {iface} into namespace {}", self.name);
  }

  /// Assign the server IP and bring the interface up inside the namespace.
  fn configure_interface(&self, iface: &str) {
    let addr = format!("{SERVER_IP}/24");

    // Enter the namespace so `ip` operates on the right network stack.
    self.enter().expect("failed to enter namespace");

    assert!(
      Command::new("ip")
        .args(["addr", "add", &addr, "dev", iface])
        .status()
        .expect("failed to run `ip`")
        .success(),
      "failed to assign {addr} to {iface}",
    );

    assert!(
      Command::new("ip")
        .args(["link", "set", iface, "up"])
        .status()
        .expect("failed to run `ip`")
        .success(),
      "failed to bring up {iface}",
    );

    self.exit().expect("failed to exit namespace");
    info!("configured {iface} with {addr}");
  }

  /// Block until the given interface disappears from the namespace
  /// (i.e. the USB device physically disconnected).
  fn wait_for_disconnect(&self, iface: &str) {
    let path = format!("/sys/class/net/{iface}");

    // `/sys/class/net/` is namespace-aware — it only shows interfaces
    // belonging to the current namespace.
    self.enter().expect("failed to enter namespace");
    while Path::new(&path).exists() {
      thread::sleep(Duration::from_millis(250));
    }
    self.exit().expect("failed to exit namespace");

    info!("{iface} disconnected");
  }
}

impl Drop for Namespace {
  fn drop(&mut self) {
    let _ = Command::new("ip")
      .args(["netns", "delete", &self.name])
      .status();
    info!("deleted network namespace: {}", self.name);
  }
}

/// Signal handler that tears down the flash namespace and exits.
/// Uses raw libc calls because they must be async-signal-safe.
extern "C" fn cleanup_namespace(_: libc::c_int) {
  unsafe {
    libc::umount2(c"/var/run/netns/flash".as_ptr(), libc::MNT_DETACH);
    libc::unlink(c"/var/run/netns/flash".as_ptr());
    libc::_exit(1);
  }
}

// ---------------------------------------------------------------------------
// Interface detection
// ---------------------------------------------------------------------------

fn interfaces() -> HashSet<String> {
  fs::read_dir("/sys/class/net")
    .into_iter()
    .flatten()
    .filter_map(|e| e.ok())
    .map(|e| e.file_name().to_string_lossy().into_owned())
    .collect()
}

fn is_usb_gadget(iface: &str) -> bool {
  let path = format!("/sys/class/net/{iface}/device/driver");
  let Ok(target) = fs::read_link(path) else { return false };
  let Some(driver) = target.file_name().and_then(|f| f.to_str()) else { return false };
  matches!(driver, "rndis_host" | "cdc_ether")
}

fn detect_interface() -> String {
  let baseline = interfaces();
  info!("waiting for new USB network interface...");

  loop {
    thread::sleep(Duration::from_millis(250));

    for iface in interfaces().difference(&baseline) {
      if is_usb_gadget(iface) {
        info!("detected interface: {iface}");
        return iface.clone();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Block device detection + image writing
// ---------------------------------------------------------------------------

fn block_devices() -> HashSet<String> {
  fs::read_dir("/sys/class/block")
    .into_iter()
    .flatten()
    .filter_map(|e| e.ok())
    .map(|e| e.file_name().to_string_lossy().into_owned())
    .collect()
}

fn is_usb_block_device(name: &str) -> bool {
  // Only match whole-disk devices (sd[a-z]), not partitions (sd[a-z][0-9]).
  if !name.starts_with("sd") || name.len() != 3 {
    return false;
  }
  // Verify it's USB-backed: /sys/class/block/sdb is a symlink whose target
  // traverses the USB subsystem (e.g. ../../devices/platform/.../usb1/...).
  // We check this rather than /sys/block/sdb/device, which only points to
  // the SCSI target node and doesn't contain "usb" in its path.
  let path = format!("/sys/class/block/{name}");
  let Ok(target) = fs::read_link(path) else { return false };
  target.to_string_lossy().contains("usb")
}

fn detect_block_device(baseline: &HashSet<String>) -> String {
  info!("waiting for new USB block device...");

  loop {
    thread::sleep(Duration::from_millis(250));

    for dev in block_devices().difference(&baseline) {
      if is_usb_block_device(dev) {
        info!("detected block device: {dev}");
        return dev.clone();
      }
    }
  }
}

fn write_image(image_path: &Path, device: &str) {
  let mut src = File::open(image_path).expect("failed to open image file");
  let total = src.metadata().expect("failed to stat image file").len();

  let dev_path = format!("/dev/{device}");
  let mut dst = File::options()
    .write(true)
    .custom_flags(libc::O_DSYNC)
    .open(&dev_path)
    .unwrap_or_else(|e| panic!("failed to open {dev_path}: {e}"));

  let pb = ProgressBar::new(total);
  pb.set_style(
    ProgressStyle::default_bar()
      .template("[{bar:40}] {bytes}/{total_bytes} ({eta})")
      .expect("invalid progress bar template")
      .progress_chars("=> "),
  );

  let mut buf = vec![0u8; 128 * 1024];
  loop {
    let n = src.read(&mut buf).expect("failed to read image");
    if n == 0 {
      break;
    }
    dst.write_all(&buf[..n]).expect("failed to write to device");
    pb.inc(n as u64);
  }

  dst.sync_all().expect("failed to sync device");
  pb.finish();
}

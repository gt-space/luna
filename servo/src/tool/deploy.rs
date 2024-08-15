use clap::ArgMatches;
use jeflog::{fail, pass, task, warn};
use ssh2::Session as SshSession;

use std::{
  env,
  fmt,
  fs,
  io::{Read, Write},
  net::{TcpStream, ToSocketAddrs},
  path::{Path, PathBuf},
  process,
  time::Duration,
};

const RUST_VERSION: &str = "1.76.0";

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum Platform {
  AppleSilicon,
  Beaglebone,
  Meerkat,
}

impl Platform {
  pub fn triple(self) -> &'static str {
    match self {
      Self::AppleSilicon => "aarch64-apple-darwin",
      Self::Beaglebone => "armv7-unknown-linux-gnueabihf",
      Self::Meerkat => "x86_64-unknown-linux-gnu",
    }
  }

  pub fn default_login(self) -> (&'static str, &'static str) {
    match self {
      Self::AppleSilicon => ("none", "none"),
      Self::Beaglebone => ("debian", "temppwd"),
      Self::Meerkat => ("yjsp", "yjspfullscale"),
    }
  }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum Repository {
  Ahrs,
  Flight,
  Gui,
  Sam,
  Servo,
}

impl Repository {
  pub fn all() -> Vec<Self> {
    vec![
      Repository::Servo,
      Repository::Flight,
      Repository::Ahrs,
      Repository::Gui,
      Repository::Sam,
    ]
  }

  pub fn remote(self) -> &'static str {
    match self {
      Self::Ahrs => "git@github-research.gatech.edu:YJSP/ahrs",
      Self::Flight => "git@github-research.gatech.edu:YJSP/flight",
      Self::Gui => "git@github-research.gatech.edu:YJSP/fs-gui",
      Self::Sam => "git@github-research.gatech.edu:YJSP/fs-sam-software",
      Self::Servo => "git@github-research.gatech.edu:YJSP/servo",
    }
  }

  /// Fetches the latest version of the repository.
  ///
  /// If there is an existing cache of the repo, then it will pull the latest
  /// changes from GitHub. If no cache exists yet, it will create one by cloning
  /// the remote repo.
  pub fn fetch_latest(self, cache: &Path) -> bool {
    task!("Locating local cache of \x1b[1m{self}\x1b[0m.");

    let repo_cache = cache.join(self.to_string());

    if repo_cache.exists() {
      pass!(
        "Using local cache found at \x1b[1m{}\x1b[0m.",
        repo_cache.to_string_lossy()
      );
      task!("Pulling latest version of branch \x1b[1mmain\x1b[0m from GitHub.");

      let pull = process::Command::new("git")
        .args(["-C", &repo_cache.to_string_lossy(), "pull"])
        .output()
        .unwrap(); // TODO: remove

      if pull.status.success() {
        pass!("Pulled latest version of \x1b[1mmain\x1b[0m from GitHub.");
      } else {
        fail!(
          "Pulling from GitHub failed: {}",
          String::from_utf8_lossy(&pull.stderr)
        );
        return false;
      }
    } else {
      warn!("Did not find an existing local cache.");

      let remote = self.remote();
      task!("Cloning remote repository at \x1b[1m{remote}\x1b[0m.");

      let clone = process::Command::new("git")
        .args(["clone", remote, &repo_cache.to_string_lossy()])
        .output()
        .unwrap();

      if clone.status.success() {
        pass!("Cloned remote repository at \x1b[1m{remote}\x1b[0m.");
      } else {
        fail!(
          "Failed to clone remote repository at \x1b[1m{remote}\x1b[0m: {}",
          String::from_utf8_lossy(&clone.stderr)
        );
        return false;
      }
    }

    true
  }

  /// Bundles the repository files
  pub fn bundle(self, cache: &Path) -> bool {
    task!("Vendoring dependencies of repository \x1b[1m{self}\x1b[0m.");

    let repo_path = cache.join(self.to_string());
    let manifest_path = repo_path.join("Cargo.toml");
    let vendor_path = repo_path.join("vendor");

    let vendor = process::Command::new("cargo")
      .args([
        "vendor",
        "--manifest-path",
        &manifest_path.to_string_lossy(),
        &vendor_path.to_string_lossy(),
      ])
      .output()
      .unwrap();

    if vendor.status.success() {
      pass!("Vendored dependencies of repository \x1b[1m{self}\x1b[0m.");
    } else {
      fail!(
        "Failed to vendor dependencies of repository \x1b[1m{self}\x1b[0m: {}",
        String::from_utf8_lossy(&vendor.stderr)
      );
      return false;
    }

    task!("Compressing repository \x1b[1m{self}\x1b[0m into a tarball.");

    let tarball_path = cache.join(format!("{self}.tar.gz"));

    let tar = process::Command::new("tar")
      .args([
        "czf",
        &tarball_path.to_string_lossy(),
        "-C",
        &cache.to_string_lossy(),
        &format!("./{self}"),
      ])
      .output()
      .unwrap();

    if tar.status.success() {
      pass!("Compressed \x1b[1m{self}\x1b[0m into a tarball.");
    } else {
      fail!("Failed to compress \x1b[1m{self}\x1b[0m into a tarball.");
      return false;
    }

    true
  }
}

impl fmt::Display for Repository {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Ahrs => write!(f, "ahrs"),
      Self::Flight => write!(f, "flight"),
      Self::Gui => write!(f, "gui"),
      Self::Sam => write!(f, "sam"),
      Self::Servo => write!(f, "servo"),
    }
  }
}

pub fn locate_cache() -> anyhow::Result<PathBuf> {
  task!("Locating cache directory.");

  let cache_path;

  if cfg!(target_os = "macos") {
    cache_path = PathBuf::from(env::var("HOME")?).join("Library/Caches/servo");
  } else if cfg!(target_os = "windows") {
    cache_path = PathBuf::from(env::var("LOCALAPPDATA")?).join("servo");
  // cache_display_path = "%LOCALAPPDATA%\\servo";
  } else {
    cache_path = PathBuf::from("/var/cache/servo");
  }

  fs::create_dir_all(&cache_path)?;
  pass!("Located cache at {}.", cache_path.to_string_lossy());
  Ok(cache_path)
}

struct Target {
  hostname: &'static str,
  repository: Repository,
  platform: Platform,

  session: Option<SshSession>,
}

impl Target {
  pub const fn new(
    hostname: &'static str,
    repository: Repository,
    platform: Platform,
  ) -> Self {
    Target {
      hostname,
      repository,
      platform,
      session: None,
    }
  }

  pub fn connect(&mut self) -> bool {
    task!("Locating target \x1b[1m{}\x1b[0m.", self.hostname);

    let address = format!("{}.local:22", self.hostname)
      .to_socket_addrs()
      .ok()
      .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

    let Some(address) = address else {
      fail!(
        "Target \x1b[1m{}\x1b[0m could not be located.",
        self.hostname
      );
      return false;
    };

    pass!(
      "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
      self.hostname,
      address.ip()
    );
    task!("Establishing TCP connection with target.");

    let Ok(socket) =
      TcpStream::connect_timeout(&address, Duration::from_millis(1000))
    else {
      fail!("Failed to establish a TCP connection with target.");
      return false;
    };

    pass!("Established TCP connection with target.");
    task!("Converting raw TCP connection into SSH session.");

    let Ok(mut session) = SshSession::new() else {
      fail!("Failed to construct new SSH session object.");
      return false;
    };

    session.set_tcp_stream(socket);

    if let Err(error) = session.handshake() {
      fail!("SSH handshake failed: {error}");
      return false;
    }

    pass!("Converted raw TCP connection into SSH session.");
    task!("Authenticating with username and password.");

    let (user, password) = self.platform.default_login();
    let auth = session.userauth_password(user, password);

    if auth.is_ok() && session.authenticated() {
      pass!("Authenticated with login \x1b[1muser : mpassword\x1b[0m.")
    } else {
      fail!("Failed to authenticate with the default login credentials.");
      return false;
    }

    self.session = Some(session);
    true
  }

  pub fn deploy(&self, cache: &Path) {
    task!(
      "Deploying \x1b[1m{}\x1b[0m to target \x1b[1m{}\x1b[0m.",
      self.repository,
      self.hostname
    );

    // self.install_rust();
    self.transfer(cache);
    self.check_rust();
    self.install();

    pass!(
      "Deployed \x1b[1m{}\x1b[0m to target \x1b[1m{}\x1b[0m.",
      self.repository,
      self.hostname
    );
  }

  /// Ensures that Rust is installed on the target machine.
  pub fn check_rust(&self) -> bool {
    task!(
      "Checking for Rust installation on target \x1b[1m{}\x1b[0m.",
      self.hostname,
    );

    let Some(session) = &self.session else {
      fail!(
        "Target \x1b[1m{}\x1b[0m was not connected before checking Rust version.",
        self.hostname,
      );
      return false;
    };

    let mut cargo_version = String::new();

    let mut channel = session.channel_session().unwrap();
    channel.exec("cargo --version").unwrap();
    channel.read_to_string(&mut cargo_version).unwrap();
    channel.wait_close().unwrap();

    if let Some(version) = cargo_version.split(' ').nth(1) {
      pass!(
        "Found Rust installation on target \x1b[1m{}\x1b[0m with Cargo v{version}.",
        self.hostname
      );
    } else {
      warn!("Did not locate an existing Rust installation.");
      self.install_rust();
    }

    true
  }

  pub fn install_rust(&self) -> bool {
    task!("Installing Rust version \x1b[1m{}\x1b[0m.", RUST_VERSION);

    let Some(session) = &self.session else {
      fail!(
        "Target \x1b[1m{}\x1b[0m was not connected before attempting to install Rust.",
        self.hostname
      );
      return false;
    };

    let download_url = format!(
      "https://static.rust-lang.org/dist/rust-{RUST_VERSION}-{}.tar.gz",
      self.platform.triple()
    );

    task!("Downloading offline installer from \x1b[1m{download_url}\x1b[0m.");

    let response = reqwest::blocking::Client::new()
      .get(&download_url)
      .timeout(Duration::from_secs(5 * 60))
      .send()
      .and_then(|response| response.bytes());

    let tarball = match response {
      Ok(bytes) => bytes,
      Err(error) => {
        fail!("Failed to fetch offline installer: {error}");
        return false;
      }
    };

    pass!("Downloaded offline installer from \x1b[1m{download_url}\x1b[0m.");
    task!("Transferring installer tarball to target.");

    let mut remote_tarball = session
      .scp_send(
        Path::new("/tmp/rust.tar.gz"),
        0o644,
        tarball.len() as u64,
        None,
      )
      .unwrap();
    remote_tarball.write_all(&tarball).unwrap();
    remote_tarball.send_eof().unwrap();
    remote_tarball.wait_eof().unwrap();
    remote_tarball.close().unwrap();
    remote_tarball.wait_close().unwrap();

    pass!("Transferred installer tarball to target.");
    task!("Uncompressing installer tarball on target.");

    let mut ret = Vec::new();

    let mut channel = session.channel_session().unwrap();
    channel.exec("tar xzf /tmp/rust.tar.gz -C /tmp").unwrap();
    channel.read_to_end(&mut ret).unwrap();
    channel.wait_close().unwrap();

    pass!("Uncompressed installer tarball on target.");
    pass!("Installed Rust version \x1b[1m{RUST_VERSION}\x1b[0m.");

    true
  }

  pub fn transfer(&self, cache: &Path) -> bool {
    task!(
      "Transferring \x1b[1m{}\x1b[0m to remote target \x1b[1m{}\x1b[0m.",
      self.repository,
      self.hostname
    );

    let Some(session) = &self.session else {
      fail!(
        "Target \x1b[1m{}\x1b[0m was not connected before attempting a transfer.",
        self.hostname
      );
      return false;
    };

    let repo = self.repository;
    let local_tarball_path = cache.join(format!("{repo}.tar.gz"));
    let remote_tarball_path = PathBuf::from(format!("/tmp/{repo}.tar.gz"));

    task!("Reading locally cached \x1b[1m{repo}\x1b[0m tarball.");

    let tarball = fs::read(local_tarball_path).unwrap();

    pass!("Read locally cached \x1b[1m{repo}\x1b[0m tarball into mempory.");
    task!("Transferring \x1b{repo}\x1b[0m tarball to remote target.");

    let mut remote_tarball = session
      .scp_send(&remote_tarball_path, 0o664, tarball.len() as u64, None)
      .unwrap();
    remote_tarball.write_all(&tarball).unwrap();
    remote_tarball.send_eof().unwrap();
    remote_tarball.wait_eof().unwrap();
    remote_tarball.close().unwrap();
    remote_tarball.wait_close().unwrap();

    pass!("Transferred \x1b[1m{repo}\x1b[0m tarball to remote target.");
    task!("Uncompressing \x1b[1m{repo}\x1b[0m tarball on remote target.");

    let mut ret = Vec::new();

    let mut channel = session.channel_session().unwrap();
    channel
      .exec(&format!(
        "tar xzf {} -C /tmp",
        remote_tarball_path.to_string_lossy()
      ))
      .unwrap();
    channel.read_to_end(&mut ret).unwrap();
    channel.wait_close().unwrap();

    pass!("Uncompressed \x1b[1m{repo}\x1b[0m tarball on remote target.");
    pass!(
      "Transferred \x1b[1m{repo}\x1b[0m to remote target \x1b[1m{}\x1b[0m.",
      self.hostname
    );

    true
  }

  pub fn install(&self) -> bool {
    task!(
      "Installing \x1b[1m{}\x1b[0m on remote target.",
      self.repository,
    );

    let Some(session) = &self.session else {
      fail!(
        "Target \x1b[1m{}\x1b[0m was not connected before attempting an install.",
        self.hostname
      );
      return false;
    };

    let mut shell_output = Vec::new();

    let mut channel = session.channel_session().unwrap();
    channel
      .exec(&format!(
        "cargo install --path /tmp/{} --offline",
        self.repository
      ))
      .unwrap();
    channel.read_to_end(&mut shell_output).unwrap();
    channel.wait_close().unwrap();

    pass!(
      "Installed \x1b[1m{}\x1b[0m on remote target.",
      self.repository
    );
    true
  }
}

// const DEFAULT_TARGETS: [Target; 8] = [
// 	Target::new("sam-01", Repository::Sam, Platform::Beaglebone),
// 	Target::new("sam-02", Repository::Sam, Platform::Beaglebone),
// 	Target::new("sam-03", Repository::Sam, Platform::Beaglebone),
// 	Target::new("server-01", Repository::Servo, Platform::Meerkat),
// 	Target::new("server-02", Repository::Servo, Platform::Meerkat),
// 	Target::new("flight-01", Repository::Flight, Platform::Beaglebone),
// 	Target::new("ground-01", Repository::Flight, Platform::Beaglebone),
// 	Target::new("gui-01", Repository::Gui, Platform::Meerkat),
// ];

/// Compiles and deploys MCFS binaries to respective machines.
pub fn deploy(args: &ArgMatches) {
  let prepare = *args.get_one::<bool>("prepare").unwrap();
  let offline = *args.get_one::<bool>("offline").unwrap();
  // let target = args.get_one::<String>("to");
  // let path = args.get_one::<String>("path");

  if prepare && offline {
    fail!("Cannot prepare for deployment while offline.");
    return;
  }

  let cache = match locate_cache() {
    Ok(cache) => cache,
    Err(error) => {
      fail!("Failed to locate cache: {error}");
      return;
    }
  };

  // TODO: Take into account --to flag
  // let targets = DEFAULT_TARGETS;
  let targets = vec![
    Target::new(
      "jeffs-macbook-pro",
      Repository::Servo,
      Platform::AppleSilicon,
    ),
    Target::new("sam-01", Repository::Sam, Platform::Beaglebone),
    Target::new("sam-02", Repository::Sam, Platform::Beaglebone),
    Target::new("sam-03", Repository::Sam, Platform::Beaglebone),
    Target::new("sam-04", Repository::Sam, Platform::Beaglebone),
    Target::new("sam-05", Repository::Sam, Platform::Beaglebone),
    Target::new("sam-06", Repository::Sam, Platform::Beaglebone),
    Target::new("gui-01", Repository::Gui, Platform::Meerkat),
    Target::new("gui-02", Repository::Gui, Platform::Meerkat),
    Target::new("gui-03", Repository::Gui, Platform::Meerkat),
    Target::new("gui-04", Repository::Gui, Platform::Meerkat),
    Target::new("gui-05", Repository::Gui, Platform::Meerkat),
    Target::new("server-01", Repository::Servo, Platform::Meerkat),
    Target::new("server-02", Repository::Servo, Platform::Meerkat),
    Target::new("ahrs", Repository::Ahrs, Platform::Beaglebone),
    Target::new("flight-01", Repository::Flight, Platform::Beaglebone),
    Target::new("flight-02", Repository::Flight, Platform::Beaglebone),
  ];

  let mut repositories = Repository::all();

  for target in &targets {
    if !repositories.contains(&target.repository) {
      repositories.push(target.repository);
    }
  }

  for repo in repositories {
    task!("Fetching and caching latest version of \x1b[1m{repo}\x1b[0m.");

    if repo.fetch_latest(&cache) {
      // succeeded
      pass!("Fetched and cached latest version of \x1b[1m{repo}\x1b[0m.");
    } else {
      // failed
      fail!("Failed to fetch latest version of \x1b[1m{repo}\x1b[0m.");
      continue;
    }

    task!("Bundling and compressing \x1b[1m{repo}\x1b[0m into a tarball.");

    if repo.bundle(&cache) {
      pass!("Bundled and compressed \x1b[1m{repo}\x1b[0m into a tarball.");
    } else {
      fail!(
        "Failed to bundle and compress \x1b[1m{repo}\x1b[0m into a tarball."
      );
      continue;
    }
  }

  for mut target in targets {
    target.connect();
    target.deploy(&cache);
  }
}

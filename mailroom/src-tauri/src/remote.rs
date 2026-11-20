use std::{
    collections::HashMap,
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    time::Duration,
};

use russh::{
    client,
    keys::{HashAlg, PublicKeyOrCertificate},
};
use russh_sftp::client::SftpSession;
use serde::{Deserialize, Serialize};
use tempfile::NamedTempFile;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    time::timeout,
};

use crate::{dataset::is_postcard, jobs::Job};

const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);
const BUFFER_SIZE: usize = 64 * 1024;

#[derive(Clone, Serialize, Deserialize)]
pub struct Endpoint {
    host: String,
    username: String,
    port: u16,
    pub folder: String,
}

impl Default for Endpoint {
    fn default() -> Self {
        Self {
            host: "flight.local".into(),
            username: "ubuntu".into(),
            port: 22,
            folder: "/home/ubuntu/flight_logs".into(),
        }
    }
}

impl Endpoint {
    fn validate(&mut self) -> Result<(), String> {
        self.host = self.host.trim().trim_end_matches('.').to_ascii_lowercase();
        self.username = self.username.trim().into();
        self.folder = self.folder.trim().into();
        if self.host.is_empty()
            || self.host.chars().any(char::is_whitespace)
            || self.host.contains(['/', '@', '\\'])
        {
            return Err("Enter a hostname or IP address.".into());
        }
        if self.username.is_empty()
            || self.username.chars().any(char::is_whitespace)
            || self.port == 0
        {
            return Err("Enter a username and a port between 1 and 65535.".into());
        }
        if !self.folder.starts_with('/') {
            return Err("Enter an absolute folder path on the flight computer.".into());
        }
        Ok(())
    }

    fn key(&self) -> String {
        format!("[{}]:{}", self.host, self.port)
    }
}

#[derive(Default, Serialize, Deserialize)]
struct Settings {
    endpoint: Endpoint,
    trusted: HashMap<String, String>,
}

struct HostCheck {
    expected: Option<String>,
    observed: Arc<Mutex<Option<String>>>,
}

impl client::Handler for HostCheck {
    type Error = russh::Error;

    async fn check_server_key(
        &mut self,
        server_key: &PublicKeyOrCertificate,
    ) -> Result<bool, Self::Error> {
        let fingerprint = server_key
            .public_key()
            .fingerprint(HashAlg::Sha256)
            .to_string();
        let matches = self.expected.as_ref() == Some(&fingerprint);
        *self.observed.lock().unwrap() = Some(fingerprint);
        Ok(matches)
    }
}

type Session = client::Handle<HostCheck>;

#[derive(Serialize)]
#[serde(tag = "status", rename_all = "camelCase")]
pub enum ConnectStatus {
    Verify { fingerprint: String },
    Password,
}

#[derive(Serialize)]
pub struct Listing {
    path: String,
    entries: Vec<RemoteEntry>,
}

#[derive(Serialize)]
struct RemoteEntry {
    name: String,
    directory: bool,
    bytes: u64,
}

#[derive(Serialize)]
pub struct DownloadReport {
    pub files: Vec<PathBuf>,
    pub notices: Vec<String>,
}

pub struct Remote {
    settings_path: PathBuf,
    settings: Settings,
    settings_error: Option<String>,
    pending: Option<(Endpoint, String)>,
    connection: Option<(Endpoint, Session)>,
    sftp: Option<SftpSession>,
}

impl Remote {
    pub fn new(settings_path: PathBuf) -> Self {
        let read = match fs::read(&settings_path) {
            Ok(data) => serde_json::from_slice(&data).map_err(io::Error::from),
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(Settings::default()),
            Err(error) => Err(error),
        };
        let (settings, settings_error) = match read {
            Ok(settings) => (settings, None),
            Err(error) => (
                Settings::default(),
                Some(format!(
                    "Couldn't read {}: {error}",
                    settings_path.display()
                )),
            ),
        };
        Self {
            settings_path,
            settings,
            settings_error,
            pending: None,
            connection: None,
            sftp: None,
        }
    }

    pub fn endpoint(&self) -> Endpoint {
        self.settings.endpoint.clone()
    }

    fn save(&self) -> io::Result<()> {
        let parent = self
            .settings_path
            .parent()
            .expect("settings has a parent directory");
        fs::create_dir_all(parent)?;
        let mut temp = NamedTempFile::new_in(parent)?;
        temp.write_all(&serde_json::to_vec_pretty(&self.settings)?)?;
        temp.persist(&self.settings_path)?;
        Ok(())
    }

    pub async fn connect(&mut self, mut endpoint: Endpoint) -> Result<ConnectStatus, String> {
        if let Some(error) = &self.settings_error {
            return Err(error.clone());
        }
        endpoint.validate()?;
        self.disconnect().await;
        let expected = self.settings.trusted.get(&endpoint.key()).cloned();
        self.open(endpoint, expected).await
    }

    pub async fn confirm(&mut self) -> Result<ConnectStatus, String> {
        let (endpoint, fingerprint) = self
            .pending
            .take()
            .ok_or("The fingerprint prompt expired. Connect again.")?;
        let status = self
            .open(endpoint.clone(), Some(fingerprint.clone()))
            .await?;
        self.settings.trusted.insert(endpoint.key(), fingerprint);
        if let Err(error) = self.save() {
            self.settings.trusted.remove(&endpoint.key());
            self.disconnect().await;
            return Err(format!("Could not save the trusted fingerprint: {error}"));
        }
        Ok(status)
    }

    async fn open(
        &mut self,
        endpoint: Endpoint,
        expected: Option<String>,
    ) -> Result<ConnectStatus, String> {
        let observed = Arc::new(Mutex::new(None));
        let check = HostCheck {
            expected: expected.clone(),
            observed: observed.clone(),
        };
        let config = client::Config::default();
        let address = (endpoint.host.as_str(), endpoint.port);
        let result = timeout(
            CONNECT_TIMEOUT,
            client::connect(Arc::new(config), address, check),
        )
        .await;
        match result {
            Ok(Ok(handle)) => {
                self.connection = Some((endpoint, handle));
                Ok(ConnectStatus::Password)
            }
            Ok(Err(error)) => {
                let seen = observed.lock().unwrap().take();
                match (expected, seen) {
                    (None, Some(fingerprint)) => {
                        self.pending = Some((endpoint, fingerprint.clone()));
                        Ok(ConnectStatus::Verify { fingerprint })
                    }
                    (Some(expected), Some(actual)) if expected != actual => Err(format!(
                        "The SSH fingerprint for {} changed. Saved: {expected}. Received: {actual}. Verify this change with the FC operator before resetting trust.", endpoint.host)),
                    _ => Err(format!("Couldn't connect to {}: {error}", endpoint.host)),
                }
            }
            Err(_) => Err(format!("Connection to {} timed out.", endpoint.host)),
        }
    }

    pub async fn authenticate(&mut self, password: String) -> Result<String, String> {
        let (endpoint, connection) = self
            .connection
            .as_mut()
            .ok_or("Connect to the flight computer first.")?;
        let sftp = timeout(
            CONNECT_TIMEOUT,
            password_session(connection, &endpoint.username, password),
        )
        .await
        .map_err(|_| "SSH sign-in timed out. Reconnect to try again.")??;
        self.sftp = Some(sftp);
        self.settings.endpoint = endpoint.clone();
        let folder = endpoint.folder.clone();
        self.save()
            .map_err(|error| format!("Couldn't save connection settings: {error}"))?;
        Ok(folder)
    }

    pub async fn disconnect(&mut self) {
        self.pending = None;
        self.sftp = None;
        if let Some((_, connection)) = self.connection.take() {
            let _ = timeout(
                Duration::from_secs(1),
                connection.disconnect(
                    russh::Disconnect::ByApplication,
                    "Mailroom disconnected",
                    "",
                ),
            )
            .await;
        }
    }

    pub async fn list(&self, path: String) -> Result<Listing, String> {
        let sftp = self
            .sftp
            .as_ref()
            .ok_or("Sign in to the flight computer first.")?;
        let path = sftp
            .canonicalize(path)
            .await
            .map_err(|error| format!("Couldn't open the remote folder: {error}"))?;
        let mut entries: Vec<_> = sftp
            .read_dir(path.clone())
            .await
            .map_err(|error| format!("Couldn't list {path}: {error}"))?
            .filter_map(|entry| {
                let name = entry.file_name();
                let metadata = entry.metadata();
                if !(safe_name(&name)
                    && (metadata.is_dir()
                        || metadata.is_regular() && is_postcard(Path::new(&name))))
                {
                    return None;
                }
                Some(RemoteEntry {
                    name,
                    directory: metadata.is_dir(),
                    bytes: metadata.len(),
                })
            })
            .collect();
        entries.sort_unstable_by(|a, b| {
            b.directory.cmp(&a.directory).then_with(|| {
                if a.directory {
                    a.name.cmp(&b.name)
                } else {
                    b.name.cmp(&a.name)
                }
            })
        });
        Ok(Listing { path, entries })
    }

    pub async fn download(
        &self,
        folder: &str,
        names: Vec<String>,
        dest: &Path,
        job: &Job,
    ) -> Result<DownloadReport, String> {
        let sftp = self
            .sftp
            .as_ref()
            .ok_or("Sign in to the flight computer first.")?;
        if !dest.is_dir() || !folder.starts_with('/') {
            return Err("Choose a valid local destination and remote folder.".into());
        }
        let mut report = DownloadReport {
            files: Vec::new(),
            notices: Vec::new(),
        };
        for name in names {
            if job.check().is_err() {
                report
                    .notices
                    .push("Download canceled. Completed files were kept.".into());
                break;
            }
            if !safe_name(&name) || !is_postcard(Path::new(&name)) {
                report
                    .notices
                    .push(format!("Skipped an invalid postcard filename: {name}"));
                continue;
            }
            let remote_path = format!("{}/{name}", folder.trim_end_matches('/'));
            match fetch(sftp, remote_path, &name, dest, job).await {
                Ok(path) => report.files.push(path),
                Err(error) => {
                    report.notices.push(format!("{name}: {error}"));
                    if job.check().is_err() {
                        break;
                    }
                }
            }
        }
        Ok(report)
    }
}

async fn password_session(
    connection: &mut Session,
    username: &str,
    password: String,
) -> Result<SftpSession, String> {
    let result = connection
        .authenticate_password(username, password.as_str())
        .await
        .map_err(|error| error.to_string())?;
    if let client::AuthResult::Failure {
        remaining_methods, ..
    } = result
    {
        if remaining_methods.contains(&russh::MethodKind::Password)
            || !remaining_methods.contains(&russh::MethodKind::KeyboardInteractive)
        {
            return Err("Sign-in failed. Check the username and password and try again.".into());
        }
        use client::KeyboardInteractiveAuthResponse::{InfoRequest, Success};
        let response = connection
            .authenticate_keyboard_interactive_start(username, None)
            .await
            .map_err(|error| error.to_string())?;
        let response = match response {
            InfoRequest { prompts, .. } if prompts.len() == 1 && !prompts[0].echo => connection
                .authenticate_keyboard_interactive_respond(vec![password])
                .await
                .map_err(|error| error.to_string())?,
            other => other,
        };
        if !matches!(response, Success) {
            return Err("Sign-in failed. Check the username and password and try again.".into());
        }
    }
    let channel = connection
        .channel_open_session()
        .await
        .map_err(|error| error.to_string())?;
    channel
        .request_subsystem(true, "sftp")
        .await
        .map_err(|error| format!("Couldn't start SFTP: {error}"))?;
    SftpSession::new(channel.into_stream())
        .await
        .map_err(|error| format!("Couldn't open SFTP: {error}"))
}

fn safe_name(name: &str) -> bool {
    !name.is_empty() && name != "." && name != ".." && !name.contains(['/', '\\', '\0'])
}

async fn fetch(
    sftp: &SftpSession,
    remote_path: String,
    name: &str,
    dest: &Path,
    job: &Job,
) -> Result<PathBuf, Box<dyn std::error::Error + Send + Sync>> {
    let local_name = Path::new(name);
    if local_name.file_name().and_then(|name| name.to_str()) != Some(name)
        || (cfg!(windows) && name.contains([':', '*', '?', '"', '<', '>', '|']))
    {
        return Err("This name can't be saved on this laptop.".into());
    }
    let mut remote = sftp.open(remote_path).await?;
    let meta = remote.metadata().await?;
    if !meta.is_regular() {
        return Err("The remote path is no longer a regular file.".into());
    }
    let total = meta
        .size
        .ok_or("The server didn't report a size for this file.")?;
    let mut temp = NamedTempFile::new_in(dest)?;
    let mut local = tokio::fs::File::from_std(temp.reopen()?);
    let mut buffer = vec![0; BUFFER_SIZE];
    let mut done = 0;
    let mut reported = 0;
    job.progress(name, 0, total);
    while done < total {
        job.check()?;
        let next = (total - done).min(buffer.len() as u64) as usize;
        let count = remote.read(&mut buffer[..next]).await?;
        if count == 0 {
            return Err(
                "The connection closed or the remote file shrank before the download finished."
                    .into(),
            );
        }
        local.write_all(&buffer[..count]).await?;
        done += count as u64;
        if done - reported >= 1024 * 1024 || done == total {
            job.progress(name, done, total);
            reported = done;
        }
    }
    local.flush().await?;
    drop(local);
    job.check()?;
    let stem = local_name
        .file_stem()
        .and_then(|stem| stem.to_str())
        .ok_or("Invalid postcard filename.")?;
    for suffix in 0.. {
        job.check()?;
        let path = if suffix == 0 {
            dest.join(name)
        } else {
            dest.join(format!("{stem} ({suffix}).postcard"))
        };
        match temp.persist_noclobber(&path) {
            Ok(_) => return Ok(path),
            Err(error) if error.error.kind() == io::ErrorKind::AlreadyExists => temp = error.file,
            Err(error) => return Err(error.into()),
        }
    }
    unreachable!("the loop returns when a file is saved or an error occurs")
}

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod dataset;
mod jobs;
mod remote;

use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
};

use dataset::{Dataset, Summary};
use jobs::{Jobs, Progress};
use remote::{ConnectStatus, DownloadReport, Endpoint, Listing, Remote};
use tauri::{async_runtime::spawn_blocking, ipc::Channel, Manager, State};

#[derive(Default)]
struct Workspace {
    dataset: Mutex<Option<Arc<Dataset>>>,
    next_id: AtomicU32,
    jobs: Jobs,
}

impl Workspace {
    fn next_id(&self) -> u32 {
        self.next_id.fetch_add(1, Ordering::Relaxed) + 1
    }

    fn dataset(&self, id: u32) -> Result<Arc<Dataset>, String> {
        self.dataset
            .lock()
            .unwrap()
            .as_ref()
            .filter(|dataset| dataset.summary.id == id)
            .cloned()
            .ok_or_else(|| "This view has changed. Try the action again.".into())
    }
}

type FlightComputer = tokio::sync::Mutex<Remote>;

#[tauri::command]
async fn load_logs(
    state: State<'_, Workspace>,
    paths: Vec<PathBuf>,
    append: bool,
    progress: Channel<Progress>,
) -> Result<Summary, String> {
    let job = Arc::new(state.jobs.start(Some(progress))?);
    let worker = job.clone();
    let previous = if append {
        state.dataset.lock().unwrap().clone()
    } else {
        None
    };
    let id = state.next_id();
    let dataset = spawn_blocking(move || Dataset::load(id, paths, previous.as_deref(), &worker))
        .await
        .map_err(|error| error.to_string())??;
    job.check().map_err(|error| error.to_string())?;
    let summary = dataset.summary.clone();
    *state.dataset.lock().unwrap() = Some(Arc::new(dataset));
    Ok(summary)
}

#[tauri::command]
async fn remove_source(
    state: State<'_, Workspace>,
    id: u32,
    path: PathBuf,
) -> Result<Option<Summary>, String> {
    let _job = state.jobs.start(None)?;
    let dataset = state.dataset(id)?;
    let next_id = state.next_id();
    let updated = spawn_blocking(move || dataset.remove_source(next_id, &path))
        .await
        .map_err(|error| error.to_string())?;
    let summary = updated.as_ref().map(|dataset| dataset.summary.clone());
    *state.dataset.lock().unwrap() = updated.map(Arc::new);
    Ok(summary)
}

#[tauri::command]
async fn get_page(
    state: State<'_, Workspace>,
    id: u32,
    start: usize,
    columns: Vec<String>,
) -> Result<Vec<Vec<String>>, String> {
    let dataset = state.dataset(id)?;
    spawn_blocking(move || dataset.page(start, columns))
        .await
        .map_err(|error| error.to_string())?
}

#[tauri::command]
async fn export_csv(
    state: State<'_, Workspace>,
    id: u32,
    path: PathBuf,
    overwrite: bool,
    columns: Option<Vec<String>>,
    progress: Channel<Progress>,
) -> Result<(), String> {
    let dataset = state.dataset(id)?;
    let job = state.jobs.start(Some(progress))?;
    spawn_blocking(move || {
        dataset
            .export(&path, overwrite, columns, &job)
            .map_err(|error| error.to_string())
    })
    .await
    .map_err(|error| error.to_string())?
}

#[tauri::command]
fn cancel_job(state: State<'_, Workspace>) {
    state.jobs.cancel();
}

#[tauri::command]
async fn fc_settings(fc: State<'_, FlightComputer>) -> Result<Endpoint, String> {
    Ok(fc.lock().await.endpoint())
}

#[tauri::command]
async fn connect_fc(
    fc: State<'_, FlightComputer>,
    endpoint: Endpoint,
) -> Result<ConnectStatus, String> {
    fc.lock().await.connect(endpoint).await
}

#[tauri::command]
async fn confirm_fc(fc: State<'_, FlightComputer>) -> Result<ConnectStatus, String> {
    fc.lock().await.confirm().await
}

#[tauri::command]
async fn authenticate_fc(
    fc: State<'_, FlightComputer>,
    password: String,
) -> Result<String, String> {
    fc.lock().await.authenticate(password).await
}

#[tauri::command]
async fn disconnect_fc(fc: State<'_, FlightComputer>) -> Result<(), String> {
    fc.lock().await.disconnect().await;
    Ok(())
}

#[tauri::command]
async fn list_remote(fc: State<'_, FlightComputer>, path: String) -> Result<Listing, String> {
    fc.lock().await.list(path).await
}

#[tauri::command]
async fn download_logs(
    fc: State<'_, FlightComputer>,
    state: State<'_, Workspace>,
    folder: String,
    names: Vec<String>,
    destination: PathBuf,
    progress: Channel<Progress>,
) -> Result<DownloadReport, String> {
    let job = state.jobs.start(Some(progress))?;
    fc.lock()
        .await
        .download(&folder, names, &destination, &job)
        .await
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(Workspace::default())
        .setup(|app| {
            let path = app.path().app_config_dir()?.join("flight-computer.json");
            app.manage(FlightComputer::new(Remote::new(path)));
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            load_logs,
            remove_source,
            get_page,
            export_csv,
            cancel_job,
            fc_settings,
            connect_fc,
            confirm_fc,
            authenticate_fc,
            disconnect_fc,
            list_remote,
            download_logs,
        ])
        .run(tauri::generate_context!())
        .expect("Mailroom failed to start");
}

use std::{
    io,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
};

use serde::Serialize;
use tauri::ipc::Channel;

#[derive(Clone, Serialize)]
pub struct Progress {
    label: String,
    done: u64,
    total: u64,
}

#[derive(Clone, Default)]
pub struct Jobs(Arc<Mutex<Option<Arc<AtomicBool>>>>);

pub struct Job {
    owner: Jobs,
    cancelled: Arc<AtomicBool>,
    channel: Option<Channel<Progress>>,
}

impl Jobs {
    pub fn start(&self, channel: Option<Channel<Progress>>) -> Result<Job, String> {
        let mut active = self.0.lock().unwrap();
        if active.is_some() {
            return Err("Wait for the current operation to finish or cancel it.".into());
        }
        let cancelled = Arc::new(AtomicBool::new(false));
        *active = Some(cancelled.clone());
        Ok(Job {
            owner: self.clone(),
            cancelled,
            channel,
        })
    }

    pub fn cancel(&self) {
        if let Some(active) = self.0.lock().unwrap().as_ref() {
            active.store(true, Ordering::Relaxed);
        }
    }
}

impl Job {
    pub fn check(&self) -> io::Result<()> {
        if self.cancelled.load(Ordering::Relaxed) {
            return Err(io::Error::new(
                io::ErrorKind::Interrupted,
                "Operation canceled.",
            ));
        }
        Ok(())
    }

    pub fn progress(&self, label: &str, done: u64, total: u64) {
        if let Some(channel) = &self.channel {
            let _ = channel.send(Progress {
                label: label.into(),
                done,
                total,
            });
        }
    }
}

impl Drop for Job {
    fn drop(&mut self) {
        *self.owner.0.lock().unwrap() = None;
    }
}

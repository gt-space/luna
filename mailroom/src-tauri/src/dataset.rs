use std::{
    collections::HashMap,
    fs::{self, File},
    io::{self, BufReader, Read},
    path::{Path, PathBuf},
    sync::Arc,
    time::SystemTime,
};

use postcard_log::{Columns, FrameReader, LogKind, Projection, Record};
use serde::Serialize;
use sha2::{Digest, Sha256};
use tempfile::NamedTempFile;

use crate::jobs::Job;

#[cfg(test)]
#[path = "dataset_tests.rs"]
mod tests;

pub const PAGE_SIZE: usize = 50;
const BUFFER_SIZE: usize = 64 * 1024;

#[derive(Clone, Copy, PartialEq, Eq)]
struct Stamp {
    length: u64,
    modified: SystemTime,
}

impl Stamp {
    fn read(file: &File) -> io::Result<Self> {
        let meta = file.metadata()?;
        if !meta.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "select a regular file",
            ));
        }
        Ok(Self {
            length: meta.len(),
            modified: meta.modified()?,
        })
    }
}

struct HashReader<R> {
    input: R,
    hash: Sha256,
}

impl<R: Read> Read for HashReader<R> {
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let count = self.input.read(buffer)?;
        self.hash.update(&buffer[..count]);
        Ok(count)
    }
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
struct SourceSummary {
    path: PathBuf,
    name: String,
    rows: usize,
    first_timestamp: f64,
    last_timestamp: f64,
    gap_seconds: Option<f64>,
    warning: Option<String>,
}

struct Segment {
    summary: SourceSummary,
    kind: LogKind,
    stamp: Stamp,
    hash: [u8; 32],
    columns: Vec<String>,
    rows: Vec<Frame>,
}

#[derive(Clone, Copy)]
struct Frame {
    timestamp: f64,
    offset: u64,
}

impl Segment {
    fn scan(path: PathBuf, job: &Job, done: u64, total: u64) -> io::Result<Self> {
        let file = File::open(&path)?;
        let stamp = Stamp::read(&file)?;
        let name = path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned();
        let source = HashReader {
            input: (&file).take(stamp.length),
            hash: Sha256::new(),
        };
        let mut reader = FrameReader::new(
            BufReader::with_capacity(BUFFER_SIZE, source),
            stamp.length,
            None,
        );
        let mut rows = Vec::new();
        let mut columns = Columns::default();
        let mut warning = None;
        let mut first = f64::INFINITY;
        let mut last = f64::NEG_INFINITY;
        loop {
            job.check()?;
            match reader.read_record() {
                Ok(Some(record)) => {
                    first = first.min(record.timestamp);
                    last = last.max(record.timestamp);
                    columns.observe(&record);
                    rows.push(Frame {
                        timestamp: record.timestamp,
                        offset: record.offset,
                    });
                    if rows.len() % 512 == 0 {
                        job.progress(&name, done + reader.position(), total);
                    }
                }
                Ok(None) => break,
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::UnexpectedEof
                            | io::ErrorKind::InvalidData
                            | io::ErrorKind::InvalidInput
                    ) =>
                {
                    warning = Some(format!(
                        "Stopped at byte {} after {} complete rows. {error}.",
                        reader.position(),
                        rows.len()
                    ));
                    break;
                }
                Err(error) => return Err(error),
            }
        }
        let kind = reader.kind().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                warning.as_deref().unwrap_or("the log is empty").to_owned(),
            )
        })?;
        let mut rest = reader.into_inner();
        let mut buffer = [0; BUFFER_SIZE];
        while rest.read(&mut buffer)? != 0 {
            job.check()?;
        }
        let hash = rest.into_inner().hash.finalize().into();
        if Stamp::read(&file)? != stamp {
            return Err(io::Error::other(
                "the log changed during indexing. Reopen it when writing has finished",
            ));
        }
        Ok(Self {
            summary: SourceSummary {
                path,
                name,
                rows: rows.len(),
                first_timestamp: first,
                last_timestamp: last,
                gap_seconds: None,
                warning,
            },
            kind,
            stamp,
            hash,
            columns: columns.finish(),
            rows,
        })
    }

    fn open(&self) -> io::Result<FrameReader<BufReader<File>>> {
        let file = File::open(&self.summary.path)?;
        if Stamp::read(&file)? != self.stamp {
            return Err(io::Error::other(format!(
                "{} changed. Reopen the log to refresh its index.",
                self.summary.name
            )));
        }
        Ok(FrameReader::new(
            BufReader::with_capacity(BUFFER_SIZE, file),
            self.stamp.length,
            Some(self.kind),
        ))
    }
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Summary {
    pub id: u32,
    columns: Vec<String>,
    sources: Vec<SourceSummary>,
    kind: LogKind,
    rows: usize,
    first_timestamp: f64,
    last_timestamp: f64,
    notices: Vec<String>,
}

#[derive(Clone, Copy)]
struct Row {
    frame: Frame,
    segment: usize,
}

pub struct Dataset {
    pub summary: Summary,
    segments: Vec<Arc<Segment>>,
    rows: Vec<Row>,
}

struct Records<'a> {
    dataset: &'a Dataset,
    readers: HashMap<usize, FrameReader<BufReader<File>>>,
}

impl Records<'_> {
    fn read(&mut self, row: &Row) -> io::Result<Record> {
        if !self.readers.contains_key(&row.segment) {
            // A flight normally keeps a single reader open. Bound the number for overlapping inputs.
            if self.readers.len() == 8 {
                self.readers.clear();
            }
            self.readers
                .insert(row.segment, self.dataset.segments[row.segment].open()?);
        }
        let reader = self
            .readers
            .get_mut(&row.segment)
            .expect("reader was opened");
        reader.seek_to(row.frame.offset)?;
        reader
            .read_record()?
            .ok_or_else(|| io::Error::other("indexed record is missing. Reopen the log"))
    }
}

pub fn is_postcard(path: &Path) -> bool {
    path.extension()
        .is_some_and(|suffix| suffix.eq_ignore_ascii_case("postcard"))
}

impl Dataset {
    pub fn load(
        id: u32,
        paths: Vec<PathBuf>,
        previous: Option<&Self>,
        job: &Job,
    ) -> Result<Self, String> {
        let mut segments = previous.map_or_else(Vec::new, |old| old.segments.clone());
        let mut notices = Vec::new();
        let mut total = 0;
        let mut inputs = Vec::new();
        for path in paths {
            let name = path.display();
            if !is_postcard(&path) {
                notices.push(format!("{name} is not a .postcard file."));
                continue;
            }
            match fs::canonicalize(&path).and_then(|path| Ok((fs::metadata(&path)?.len(), path))) {
                Ok((length, path)) => {
                    total += length;
                    inputs.push((length, path));
                }
                Err(error) => notices.push(format!("{name}: {error}")),
            }
        }

        let mut done = 0;
        for (length, path) in inputs {
            job.check().map_err(|error| error.to_string())?;
            let name = path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            if segments.iter().any(|segment| segment.summary.path == path) {
                notices.push(format!("Skipped {name}. This file is already open."));
            } else {
                match Segment::scan(path, job, done, total) {
                    Ok(segment) => {
                        if let Some(duplicate) = segments.iter().find(|current| {
                            current.stamp.length == segment.stamp.length
                                && current.hash == segment.hash
                        }) {
                            notices.push(format!(
                                "Skipped {name}. It is identical to {}.",
                                duplicate.summary.name
                            ));
                        } else {
                            if segments
                                .first()
                                .is_some_and(|first| first.kind != segment.kind)
                            {
                                return Err(format!("{name} contains {:?} data. Open VehicleState and IMU logs separately.", segment.kind));
                            }
                            segments.push(Arc::new(segment));
                        }
                    }
                    Err(error) => notices.push(format!("{name}: {error}")),
                }
            }
            done += length;
            job.progress(&name, done, total);
        }
        job.check().map_err(|error| error.to_string())?;
        if segments.is_empty() {
            return Err(if notices.is_empty() {
                "Choose one or more .postcard files.".into()
            } else {
                notices.join("\n")
            });
        }
        Ok(Self::assemble(id, segments, notices))
    }

    pub fn remove_source(&self, id: u32, path: &Path) -> Option<Self> {
        let remaining: Vec<_> = self
            .segments
            .iter()
            .filter(|segment| segment.summary.path != path)
            .cloned()
            .collect();
        if remaining.is_empty() {
            None
        } else {
            Some(Self::assemble(id, remaining, Vec::new()))
        }
    }

    fn assemble(id: u32, mut segments: Vec<Arc<Segment>>, notices: Vec<String>) -> Self {
        segments.sort_unstable_by(|left, right| left.summary.path.cmp(&right.summary.path));
        let mut columns = Columns::default();
        let mut rows = Vec::with_capacity(segments.iter().map(|segment| segment.rows.len()).sum());
        let mut sources = Vec::with_capacity(segments.len());
        for (index, segment) in segments.iter().enumerate() {
            sources.push(segment.summary.clone());
            columns.extend(&segment.columns);
            rows.extend(segment.rows.iter().map(|frame| Row {
                frame: *frame,
                segment: index,
            }));
        }
        rows.sort_unstable_by(|left, right| {
            left.frame
                .timestamp
                .total_cmp(&right.frame.timestamp)
                .then(left.segment.cmp(&right.segment))
                .then(left.frame.offset.cmp(&right.frame.offset))
        });
        sources.sort_by(|left, right| left.first_timestamp.total_cmp(&right.first_timestamp));
        let mut max_end: Option<f64> = None;
        for source in &mut sources {
            source.gap_seconds = max_end.map(|end| source.first_timestamp - end);
            max_end =
                Some(max_end.map_or(source.last_timestamp, |end| end.max(source.last_timestamp)));
        }
        let summary = Summary {
            id,
            columns: columns.finish(),
            kind: segments[0].kind,
            sources,
            rows: rows.len(),
            first_timestamp: rows[0].frame.timestamp,
            last_timestamp: rows[rows.len() - 1].frame.timestamp,
            notices,
        };
        Self {
            summary,
            segments,
            rows,
        }
    }

    fn select_columns(&self, selection: Option<Vec<String>>) -> Result<Vec<String>, String> {
        let Some(selection) = selection else {
            return Ok(self.summary.columns.clone());
        };
        let columns: Vec<_> = self
            .summary
            .columns
            .iter()
            .filter(|column| selection.contains(column))
            .cloned()
            .collect();
        if columns.len() != selection.len() {
            return Err("The selection contains an unknown or repeated column.".into());
        }
        Ok(columns)
    }

    pub fn page(&self, start: usize, selection: Vec<String>) -> Result<Vec<Vec<String>>, String> {
        let columns = self.select_columns(Some(selection))?;
        let projection = Projection::new(&columns);
        let mut records = Records {
            dataset: self,
            readers: HashMap::new(),
        };
        self.rows
            .iter()
            .skip(start)
            .take(PAGE_SIZE)
            .map(|row| {
                records
                    .read(row)
                    .map(|record| projection.row(&record))
                    .map_err(|error| error.to_string())
            })
            .collect()
    }

    pub fn export(
        &self,
        path: &Path,
        overwrite: bool,
        selection: Option<Vec<String>>,
        job: &Job,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if !path
            .extension()
            .is_some_and(|suffix| suffix.eq_ignore_ascii_case("csv"))
        {
            return Err("Save the export with a .csv extension.".into());
        }
        let columns = self.select_columns(selection)?;
        if columns.is_empty() {
            return Err("Select at least one column to export.".into());
        }
        let target = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        let mut temp = NamedTempFile::new_in(target)?;
        let projection = Projection::new(&columns);
        let mut records = Records {
            dataset: self,
            readers: HashMap::new(),
        };
        let total = self.rows.len() as u64;
        {
            let mut writer = csv::Writer::from_writer(temp.as_file_mut());
            writer.write_record(&columns)?;
            for (index, row) in self.rows.iter().enumerate() {
                job.check()?;
                writer.write_record(projection.row(&records.read(row)?))?;
                if (index + 1) % 512 == 0 {
                    job.progress("Writing CSV", (index + 1) as u64, total);
                }
            }
            writer.flush()?;
        }
        for segment in &self.segments {
            segment.open()?;
        }
        job.check()?;
        if overwrite {
            temp.persist(path)?;
        } else {
            temp.persist_noclobber(path)?;
        }
        job.progress("CSV saved", total, total);
        Ok(())
    }
}

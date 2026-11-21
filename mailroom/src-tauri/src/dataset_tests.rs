use super::*;
use crate::jobs::Jobs;

fn imu_log(path: &Path, timestamps: &[f64], truncated: bool) {
    let mut bytes = Vec::new();
    for timestamp in timestamps {
        // The IMU record contains one timestamp and six sensor values, all f64.
        bytes.extend(56u64.to_le_bytes());
        bytes.extend(timestamp.to_le_bytes());
        bytes.extend([0; 48]);
    }
    if truncated {
        bytes.extend(56u64.to_le_bytes());
        bytes.extend([0; 6]);
    }
    fs::write(path, bytes).unwrap();
}

#[test]
fn stitches_renamed_chunks_and_retains_overlapping_rows() {
    let root = tempfile::tempdir().unwrap();
    let first = root.path().join("z.postcard");
    let second = root.path().join("a.postcard");
    let copy = root.path().join("copy.postcard");
    imu_log(&first, &[1.0, 3.0, 2.0], true);
    imu_log(&second, &[2.0, 4.0], false);
    fs::copy(&first, &copy).unwrap();
    let jobs = Jobs::default();
    let job = jobs.start(None).unwrap();
    let dataset = Dataset::load(1, vec![second, first, copy], None, &job).unwrap();
    assert_eq!(dataset.summary.kind, LogKind::Imu);
    assert_eq!(dataset.summary.rows, 5);
    assert_eq!(dataset.summary.sources[0].name, "z.postcard");
    assert_eq!(dataset.summary.sources[1].gap_seconds, Some(-1.0));
    assert!(dataset.summary.sources[0]
        .warning
        .as_ref()
        .unwrap()
        .contains("partial record payload"));
    assert!(dataset.summary.notices[0].contains("identical"));
    let dst = root.path().join("flight.csv");
    dataset
        .export(&dst, false, Some(vec!["timestamp".into()]), &job)
        .unwrap();
    assert_eq!(
        fs::read_to_string(&dst).unwrap(),
        "timestamp\n1\n2\n2\n3\n4\n"
    );
    assert!(dataset.export(&dst, false, None, &job).is_err());
    jobs.cancel();
    assert!(dataset.export(&dst, true, None, &job).is_err());
    assert_eq!(
        fs::read_to_string(&dst).unwrap(),
        "timestamp\n1\n2\n2\n3\n4\n"
    );
}

#[test]
#[ignore = "requires the five local GPS fixtures in ../../flight_logs"]
fn supplied_gps_run() {
    let input = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../flight_logs");
    let mut paths: Vec<_> = fs::read_dir(input)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            path.file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .starts_with("flight_data_20261120_")
        })
        .collect();
    paths.sort_unstable_by(|a, b| b.cmp(a));
    assert_eq!(paths.len(), 5);
    let job = Jobs::default().start(None).unwrap();
    let started = std::time::Instant::now();
    let dataset = Dataset::load(1, paths, None, &job).unwrap();
    eprintln!("Indexed five files in {:.2?}", started.elapsed());
    assert_eq!(dataset.summary.rows, 361_200);
    assert_eq!(dataset.summary.kind, LogKind::VehicleState);
    assert_eq!(dataset.summary.sources.len(), 5);
    assert!(dataset
        .summary
        .sources
        .iter()
        .all(|source| source.warning.is_none()));
    assert!(dataset.summary.sources[1..]
        .iter()
        .all(|source| (0.006..0.008).contains(&source.gap_seconds.unwrap())));
    assert!(dataset.segments[0]
        .columns
        .iter()
        .any(|column| dataset.segments[1..]
            .iter()
            .all(|segment| !segment.columns.contains(column))));

    let gps: Vec<_> = dataset
        .summary
        .columns
        .iter()
        .filter(|column| {
            column.as_str() == "timestamp"
                || column.starts_with("gps.")
                || column.as_str() == "gps_valid"
        })
        .cloned()
        .collect();
    assert!(gps.len() > 2);
    let root = tempfile::tempdir().unwrap();
    let selected = root.path().join("gps.csv");
    let started = std::time::Instant::now();
    dataset
        .export(&selected, false, Some(gps.clone()), &job)
        .unwrap();
    eprintln!(
        "Exported {} GPS columns in {:.2?}",
        gps.len(),
        started.elapsed()
    );
    let mut csv = csv::Reader::from_path(selected).unwrap();
    assert_eq!(csv.headers().unwrap().iter().collect::<Vec<_>>(), gps);
    let mut previous = f64::NEG_INFINITY;
    let mut count = 0;
    for record in csv.records() {
        let timestamp: f64 = record.unwrap()[0].parse().unwrap();
        assert!(timestamp > previous);
        previous = timestamp;
        count += 1;
    }
    assert_eq!(count, 361_200);

    let all = root.path().join("full.csv");
    let started = std::time::Instant::now();
    dataset.export(&all, false, None, &job).unwrap();
    eprintln!(
        "Exported {} columns in {:.2?}",
        dataset.summary.columns.len(),
        started.elapsed()
    );
    let mut csv = csv::Reader::from_path(all).unwrap();
    assert_eq!(
        csv.headers().unwrap().iter().collect::<Vec<_>>(),
        dataset.summary.columns
    );
    let expected = dataset.page(0, dataset.summary.columns.clone()).unwrap();
    for row in expected {
        assert_eq!(
            csv.records()
                .next()
                .unwrap()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            row
        );
    }
    assert_eq!(
        csv.records()
            .inspect(|record| assert!(record.is_ok()))
            .count(),
        361_200 - PAGE_SIZE
    );
}

use chrono::{DateTime, Datelike, Local, NaiveDate, NaiveDateTime, NaiveTime};
use serde_json::json;
use std::{fs, path::PathBuf, time::Duration};

const TIME_FORMATS: [&str; 4] = [
  // 12 hour clocks
  "%I:%M:%S%P",
  "%I:%M%P",
  // 24 hour clocks
  "%H:%M:%S",
  "%H:%M",
];

const DATE_FORMATS: [&str; 6] = [
  "%D",       // "08/01/24"
  "%m/%d/%Y", // "08/01/2024"
  "%b %d %y", // "Aug 01 24"
  "%B %d %y", // "August 01 24"
  "%b %d %Y", // "Aug 01 2024"
  "%B %d %Y", // "August 01 2024"
];

// these are achieved by adding (%Y) to the end, and then appending the current
// year to the parsed string
const PARTIAL_DATE_FORMATS: [&str; 3] = [
  "%b %d", // "Aug 01"
  "%B %d", // "August 01"
  "%m/%d", // "08/01"
];

// Currently these functions may do some weird stuff with daylight savings time
// (because 1:00 can map to 2 times on one day of the year etc),
// But the depth of complexity of timezones and edge cases is huge,
// and this is readable and assumes the largest window of time possible,
// so I am sticking with it for now.

fn edit_date_time(
  date_time: NaiveDateTime,
  output: &mut DateTime<Local>,
  earliest: bool,
) -> bool {
  let local_result = date_time.and_local_timezone(Local);

  let output_option = if earliest {
    local_result.earliest()
  } else {
    local_result.latest()
  };
  if let Some(x) = output_option {
    *output = x;
    return true;
  }
  false
}

fn edit_time(
  time: NaiveTime,
  output: &mut DateTime<Local>,
  earliest: bool,
) -> bool {
  edit_date_time(output.date_naive().and_time(time), output, earliest)
}

fn edit_date(
  date: NaiveDate,
  output: &mut DateTime<Local>,
  earliest: bool,
) -> bool {
  edit_date_time(date.and_time(output.time()), output, earliest)
}

// Modifies output as it will keep either the date or time portion of the
// original if only date / time is given
fn parse_date_and_time(
  string: &str,
  output: &mut DateTime<Local>,
  earliest: bool,
) -> bool {
  let curr_year = format!("({:04})", output.year());
  // Full ones
  for t_fmt in TIME_FORMATS {
    for d_fmt in DATE_FORMATS {
      if let Ok(dt) = NaiveDateTime::parse_from_str(
        string,
        format!("{} {}", d_fmt, t_fmt).as_str(),
      ) {
        if edit_date_time(dt, output, earliest) {
          return true;
        }
      }
    }
    for pd_fmt in PARTIAL_DATE_FORMATS {
      if let Ok(dt) = NaiveDateTime::parse_from_str(
        format!("{}{}", string, curr_year).as_str(),
        format!("{} {}(%Y)", pd_fmt, t_fmt).as_str(),
      ) {
        if edit_date_time(dt, output, earliest) {
          return true;
        }
      }
    }
  }

  for t_fmt in TIME_FORMATS {
    if let Ok(time) = NaiveTime::parse_from_str(string, t_fmt) {
      if edit_time(time, output, earliest) {
        return true;
      }
    }
  }
  for d_fmt in DATE_FORMATS {
    if let Ok(date) = NaiveDate::parse_from_str(string, d_fmt) {
      if edit_date(date, output, earliest) {
        return true;
      }
    }
  }
  for pd_fmt in PARTIAL_DATE_FORMATS {
    if let Ok(date) = NaiveDate::parse_from_str(
      format!("{}{}", string, curr_year).as_str(),
      format!("{}(%Y)", pd_fmt).as_str(),
    ) {
      if edit_date(date, output, earliest) {
        return true;
      }
    }
  }

  false
}

/// defaults to the beginning of the current day
fn parse_start(string: &str) -> Option<DateTime<Local>> {
  // get the start of today as default
  let now = Local::now();
  let mut start = now
    .with_time(NaiveTime::from_hms_opt(0, 0, 0).unwrap())
    .unwrap(); // this shouldn't ever fail

  if string.is_empty() {
    return Some(start);
  }

  // try to parse
  if parse_date_and_time(string, &mut start, true) {
    Some(start)
  } else {
    None
  }
}

/// defaults to the current time
fn parse_end(string: &str) -> Option<DateTime<Local>> {
  // get the start of today as default
  let now = Local::now();
  let mut end = now
    .with_time(NaiveTime::from_hms_opt(0, 0, 0).unwrap())
    .unwrap(); // this shouldn't ever fail

  if string.is_empty() {
    return Some(now);
  }

  // try to parse
  if parse_date_and_time(string, &mut end, false) {
    Some(end)
  } else {
    None
  }
}

/// Function for requesting all data between two timestamps as stored on the
/// ground server.
///
/// Used in the export command line routing.
pub fn export(
  from: Option<String>,
  to: Option<String>,
  output_path: &str,
  all: &bool,
) -> anyhow::Result<()> {
  let output_path = PathBuf::from(output_path);

  let start_str = from.unwrap_or_default();
  let end_str = to.unwrap_or_default();

  // Easy error messaging
  let start = parse_start(start_str.as_str()).unwrap_or_else(|| {
    panic!("\n ERROR : \"{}\" is an invalid date / time\n", start_str)
  });

  let end = parse_end(end_str.as_str()).unwrap_or_else(|| {
    panic!("\n ERROR : \"{}\" is an invalid date / time\n", end_str)
  });

  if *all {
    println!("Exporting all data");
  } else {
    println!("Exporting from {} to {}", start, end);
    println!("({} to {})", start.timestamp(), end.timestamp());
  }

  let export_format = output_path.extension().unwrap().to_string_lossy();

  let client = reqwest::blocking::Client::new();
  let export_content = client
    .post("http://localhost:7200/data/export")
    .json(&json!({
      "format": export_format,
      "from": if *all { f64::MIN } else {start.timestamp() as f64},
      "to": if *all { f64::MAX } else {end.timestamp() as f64}
    }))
    .timeout(Duration::from_secs(3600))
    .send()?;

  // Either write the file as text if it's a csv, or bytes if it's a file.
  // (assumed for all other returns)
  if export_format == "csv" {
    let text = export_content.text()?;
    fs::write(output_path, text)?;
  } else {
    let bytes = export_content.bytes()?;
    fs::write(output_path, bytes)?;
  }

  Ok(())
}

#[cfg(test)]
mod tests {
  use chrono::TimeZone;

  use super::*;

  #[test]
  fn date_parsing_correct() {
    let base_date = NaiveDate::from_ymd_opt(1987, 11, 13).unwrap();
    let testing_points = [
      // Time only
      ("4:00pm", base_date.and_hms_opt(16, 0, 0).unwrap()),
      ("4:0pm", base_date.and_hms_opt(16, 0, 0).unwrap()),
      ("06:00:13", base_date.and_hms_opt(6, 0, 13).unwrap()),
      // Date only
      (
        "Aug 12",
        NaiveDate::from_ymd_opt(1987, 8, 12)
          .unwrap()
          .and_hms_opt(0, 0, 0)
          .unwrap(),
      ),
      (
        "Aug 1",
        NaiveDate::from_ymd_opt(1987, 8, 1)
          .unwrap()
          .and_hms_opt(0, 0, 0)
          .unwrap(),
      ),
      (
        "September 7 1983",
        NaiveDate::from_ymd_opt(1983, 9, 7)
          .unwrap()
          .and_hms_opt(0, 0, 0)
          .unwrap(),
      ),
      (
        "02/29/1992",
        NaiveDate::from_ymd_opt(1992, 2, 29)
          .unwrap()
          .and_hms_opt(0, 0, 0)
          .unwrap(),
      ),
      (
        "2/29/1992",
        NaiveDate::from_ymd_opt(1992, 2, 29)
          .unwrap()
          .and_hms_opt(0, 0, 0)
          .unwrap(),
      ),
      // Date and time
      (
        "Aug 13 1:00pm",
        NaiveDate::from_ymd_opt(1987, 8, 13)
          .unwrap()
          .and_hms_opt(13, 0, 0)
          .unwrap(),
      ),
      (
        "Aug 31 14:00:30",
        NaiveDate::from_ymd_opt(1987, 8, 31)
          .unwrap()
          .and_hms_opt(14, 0, 30)
          .unwrap(),
      ),
      (
        "September 7 1983 6:11am",
        NaiveDate::from_ymd_opt(1983, 9, 7)
          .unwrap()
          .and_hms_opt(6, 11, 0)
          .unwrap(),
      ),
      (
        "02/29/1992 14:00",
        NaiveDate::from_ymd_opt(1992, 2, 29)
          .unwrap()
          .and_hms_opt(14, 0, 0)
          .unwrap(),
      ),
      (
        "2/29/1992 2:13:41pm",
        NaiveDate::from_ymd_opt(1992, 2, 29)
          .unwrap()
          .and_hms_opt(14, 13, 41)
          .unwrap(),
      ),
    ];

    for (string, dt) in testing_points {
      // the bite of 87 (It was mangle)
      let mut date = Local
        .with_ymd_and_hms(1987, 11, 13, 0, 0, 0)
        .earliest()
        .expect("Expected base date to be valid");
      assert!(
        parse_date_and_time(string, &mut date, true),
        "Parsing {} should not fail",
        string
      );
      assert_eq!(
        date.naive_local(),
        dt,
        "Date parsed is incorrect, '{}' parsed as {} when it should be {}",
        string,
        date.naive_local(),
        dt
      );
    }
  }

  #[test]
  fn date_parsing_incorrect() {
    let testing_points = [
      // Time only
      "-1:00pm",
      "4",
      "13:00am",
      "25:00:00",
      "24:00:00",
      "12:00:00gm",
      // Date only
      "Agst 12",
      "Oogust 1",
      "September 32 1983",
      "September 18th",
      "02/29/2001",
      "2/29/2001",
    ];
    for string in testing_points {
      let mut date = Local
        .with_ymd_and_hms(1987, 11, 13, 0, 0, 0)
        .earliest()
        .expect("Expected base date to be valid");
      assert!(
        !parse_date_and_time(string, &mut date, true),
        "{} should be an invalid date / time",
        string
      );
    }
  }
}

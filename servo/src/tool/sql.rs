use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SqlResponse {
  pub column_names: Vec<String>,
  pub rows: Vec<Vec<serde_json::Value>>,
}

/// Tool command function that sends a SQL request to Servo.
pub fn sql(sql: &str) -> anyhow::Result<()> {
  let request = serde_json::json!({
    "raw_sql": sql
  });

  let client = reqwest::blocking::Client::new();
  let response: SqlResponse = serde_json::from_str(
    &client
      .post("http://localhost:7200/admin/sql")
      .json(&request)
      .send()?
      .error_for_status()?
      .text()?,
  )?;

  if !response.rows.is_empty() {
    let mut column_widths: Vec<usize> = response
      .column_names
      .iter()
      .map(|name| name.len())
      .collect();

    let mut rows: Vec<Vec<String>> = Vec::with_capacity(response.rows.len());

    for row in response.rows {
      let mut columns = Vec::with_capacity(row.len());

      for column in row {
        let value_string = match column {
          serde_json::Value::Number(val) => format!("\x1b[32m{val}"),
          serde_json::Value::String(val) => format!("\x1b[31m'{val}'"),
          serde_json::Value::Null => "\x1b[34mNULL".to_owned(),
          serde_json::Value::Array(array) => {
            if array.len() <= 8 {
              let mut value_string = "\x1b[32m0x".to_owned();

              for val in array {
                let byte =
                  val.as_i64().ok_or(anyhow::Error::msg("bad response"))?;

                if byte < u8::MIN as i64 || byte > u8::MAX as i64 {
                  return Err(anyhow::Error::msg("bad response"));
                }

                value_string.push_str(&format!("{byte:02X}"));
              }

              value_string
            } else {
              format!("\x1b[32m{} bytes", array.len())
            }
          }
          _ => return Err(anyhow::Error::msg("bad response")),
        };

        columns.push(value_string);
      }

      rows.push(columns);
    }

    for w in 0..column_widths.len() {
      for row in &rows {
        let length = row[w].len() - 5;

        if length > column_widths[w] {
          column_widths[w] = length;
        }
      }
    }

    print!("\x1b[1;4m");

    for (c, name) in response.column_names.iter().enumerate() {
      print!(
        "\x1b[39m|\x1b[33m {name:^width$} ",
        width = column_widths[c]
      );
    }

    println!("\x1b[39m|\x1b[0m");

    for row in rows {
      for (c, column) in row.iter().enumerate() {
        print!(
          "\x1b[39;1m|\x1b[22m {column:<width$} ",
          width = column_widths[c] + 5
        );
      }

      println!("\x1b[39;1m|");
    }
  }

  Ok(())
}

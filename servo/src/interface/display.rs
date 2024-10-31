use crate::server::Shared;
use common::comm::{CompositeValveState, Sequence};
use std::{
  collections::HashMap,
  error::Error,
  io::{self, Stdout},
  ops::Div,
  time::{Duration, Instant},
  vec::Vec,
};
use sysinfo::{CpuExt, System, SystemExt};

use common::comm::{Measurement, ValveState};
use crossterm::{
  event::{
    self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind, KeyModifiers
  },
  execute,
  terminal::{
    disable_raw_mode,
    enable_raw_mode,
    EnterAlternateScreen,
    LeaveAlternateScreen,
  },
};
use ratatui::{prelude::*, widgets::*};
use std::string::String;
use tokio::time::sleep;

const YJSP_YELLOW: Color = Color::from_u32(0x00ffe659);

const WHITE: Color = Color::from_u32(0x00eeeeee);
const BLACK: Color = Color::from_u32(0);

const GREY: Color = Color::from_u32(0x00bbbbbb);
const DARK_GREY: Color = Color::from_u32(0x00444444);

const DESATURATED_GREEN: Color = Color::from_u32(0x007aff85);
const DESATURATED_RED: Color = Color::from_u32(0x00ff5959);
const DESATURATED_BLUE: Color = Color::from_u32(0x0075a8ff);

const YJSP_STYLE: Style = Style::new().bg(Color::from_u32(0)).fg(YJSP_YELLOW);

fn get_state_style(state: ValveState) -> Style {
  match state {
    ValveState::Undetermined => YJSP_STYLE.fg(WHITE).bg(DARK_GREY).bold(),
    ValveState::Disconnected => YJSP_STYLE.fg(BLACK).bg(GREY).bold(),
    ValveState::Open => YJSP_STYLE.fg(BLACK).bg(DESATURATED_GREEN).bold(),
    ValveState::Closed => YJSP_STYLE.fg(BLACK).bg(DESATURATED_RED).bold(),
    ValveState::Fault => YJSP_STYLE.fg(BLACK).bg(DESATURATED_BLUE).bold(),
  }
}

fn get_full_row_style(state: ValveState) -> Style {
  match state {
    ValveState::Undetermined => YJSP_STYLE.fg(WHITE).bg(DARK_GREY),
    ValveState::Disconnected => YJSP_STYLE.fg(BLACK).bg(GREY),
    ValveState::Fault => YJSP_STYLE.fg(BLACK).bg(DESATURATED_RED),
    _ => YJSP_STYLE.fg(WHITE),
  }
}

fn get_valve_name_style(state: ValveState) -> Style {
  match state {
    ValveState::Undetermined => YJSP_STYLE.bg(DARK_GREY).bold(),
    ValveState::Disconnected => YJSP_STYLE.fg(BLACK).bg(GREY).bold(),
    ValveState::Fault => YJSP_STYLE.bg(DESATURATED_RED).bold(),
    _ => YJSP_STYLE.bold(),
  }
}

struct NamedValue<T: Clone> {
  name: String,
  value: T,
}

impl<T: Clone> NamedValue<T> {
  fn new(new_name: String, new_value: T) -> NamedValue<T> {
    NamedValue {
      name: new_name,
      value: new_value,
    }
  }
}

/// A fast and stable ordered vector of objects with a corresponding string key
/// stored in a hashmap.
///
/// Used in TUI to hold items grabbed from a hashmap / hashset for a constant
/// ordering when iterated through and holding historic data.
///
/// TODO: This should likely be moved to common after unit testing is made later
/// down the line.
struct StringLookupVector<T: Clone> {
  lookup: HashMap<String, usize>,
  vector: Vec<NamedValue<T>>,
}

struct StringLookupVectorIter<'a, T: Clone> {
  reference: &'a StringLookupVector<T>,
  index: usize,
}

impl<'a, T: Clone> Iterator for StringLookupVectorIter<'a, T> {
  // we will be counting with usize
  type Item = &'a NamedValue<T>;

  // next() is the only required method
  fn next(&mut self) -> Option<Self::Item> {
    // Check to see if we've finished counting or not.
    let out = if self.index < self.reference.vector.len() {
      Some(self.reference.vector.get(self.index).unwrap())
    } else {
      None
    };

    // Increment the index
    self.index += 1;

    out
  }
}

impl<T: Clone> StringLookupVector<T> {
  const DEFAULT_CAPACITY: usize = 8;

  fn len(&self) -> usize {
    self.vector.len()
  }

  /// Creates a new StringLookupVector with a specified capacity
  fn with_capacity(capacity: usize) -> StringLookupVector<T> {
    StringLookupVector {
      lookup: HashMap::<String, usize>::with_capacity(capacity),
      vector: Vec::<NamedValue<T>>::with_capacity(capacity),
    }
  }

  /// Creates a new StringLookupVector with default capacity
  fn new() -> StringLookupVector<T> {
    StringLookupVector::with_capacity(StringLookupVector::<T>::DEFAULT_CAPACITY)
  }

  /// Checks if a key is contained within the StringLookupVector
  fn contains_key(&self, key: &String) -> bool {
    self.lookup.contains_key(key)
  }

  /// Returns true if the object was added, and false if it was replaced
  fn add(&mut self, name: &String, value: T) {
    if self.contains_key(name) {
      self.vector[self.lookup[name]].value = value;
      return;
    }
    self.lookup.insert(name.clone(), self.vector.len());
    self.vector.push(NamedValue::new(name.clone(), value));
  }

  /// Sorts the backing vector by name, meaning iterating through this structure
  /// will go through alphabetical.
  fn sort_by_name(&mut self) {
    self.vector.sort_unstable_by_key(|x| x.name.to_string());
    for i in 0..self.vector.len() {
      // Key has to exist by the nature of this structure
      *self.lookup.get_mut(&self.vector[i].name).unwrap() = i;
    }
  }

  /// Gets a mutable reference to the item with the given key.
  /// Panics if the key is not valid
  fn get_mut(&mut self, key: &String) -> Option<&mut NamedValue<T>> {
    let index = self.lookup.get(key);
    match index {
      Some(x) => self.vector.get_mut(*x),
      None => None,
    }
  }

  fn iter(&self) -> StringLookupVectorIter<T> {
    StringLookupVectorIter::<T> {
      reference: self,
      index: 0,
    }
  }
}

#[derive(Clone)]
struct FullValveDatapoint {
  voltage: f64,
  current: f64,
  knows_voltage: bool,
  knows_current: bool,
  rolling_voltage_average: f64,
  rolling_current_average: f64,
  state: CompositeValveState,
}

#[derive(Clone)]
struct SensorDatapoint {
  measurement: Measurement,
  rolling_average: f64,
}

#[derive(Clone)]
struct SystemDatapoint {
  cpu_usage: f32,
  mem_usage: f32,
}

struct TuiData {
  sensors: StringLookupVector<SensorDatapoint>,
  valves: StringLookupVector<FullValveDatapoint>,
  system_data: StringLookupVector<SystemDatapoint>,
}

impl TuiData {
  fn new() -> TuiData {
    TuiData {
      sensors: StringLookupVector::<SensorDatapoint>::new(),
      valves: StringLookupVector::<FullValveDatapoint>::new(),
      system_data: StringLookupVector::<SystemDatapoint>::new(),
    }
  }
}

/// Updates the backing tui_data instance that is used in the rendering
/// functions.
async fn update_information(
  tui_data: &mut TuiData,
  shared: &Shared,
  system: &mut System,
) {
  // display system statistics
  system.refresh_cpu();
  system.refresh_memory();

  let hostname = system
    .host_name()
    .unwrap_or("\x1b[33mnone\x1b[0m".to_owned());

  if !tui_data.system_data.contains_key(&hostname) {
    tui_data.system_data.add(
      &hostname,
      SystemDatapoint {
        cpu_usage: 0.0,
        mem_usage: 0.0,
      },
    );
  }

  let servo_usage: &mut SystemDatapoint =
    &mut tui_data.system_data.get_mut(&hostname).unwrap().value;

  servo_usage.cpu_usage = system
    .cpus()
    .iter()
    .fold(0.0, |util, cpu| util + cpu.cpu_usage())
    .div(system.cpus().len() as f32);

  servo_usage.mem_usage =
    system.used_memory() as f32 / system.total_memory() as f32 * 100.0;

  // display sensor data
  let vehicle_state = shared.vehicle.0.lock().await.clone();

  let sensor_readings =
    vehicle_state.sensor_readings.iter().collect::<Vec<_>>();

  let valve_states = vehicle_state.valve_states.iter().collect::<Vec<_>>();
  let mut sort_needed = false;

  for (name, value) in valve_states {
    match tui_data.valves.get_mut(name) {
      Some(x) => x.value.state = value.clone(),
      None => {
        tui_data.valves.add(
          name,
          FullValveDatapoint {
            voltage: 0.0,
            current: 0.0,
            knows_voltage: false,
            knows_current: false,
            rolling_voltage_average: 0.0,
            rolling_current_average: 0.0,
            state: value.clone(),
          },
        );
        sort_needed = true;
      }
    }
  }

  if sort_needed {
    tui_data.valves.sort_by_name();
  }

  const CURRENT_SUFFIX: &str = "_I";
  const VOLTAGE_SUFFIX: &str = "_V";
  sort_needed = true;

  for (name, value) in sensor_readings {
    if name.len() > 2 {
      if name.ends_with(CURRENT_SUFFIX) {
        let mut real_name = name.clone();
        let _ = real_name.split_off(real_name.len() - 2);

        if let Some(valve_datapoint) = tui_data.valves.get_mut(&real_name) {
          valve_datapoint.value.current = value.value;

          if !valve_datapoint.value.knows_current {
            valve_datapoint.value.rolling_current_average = value.value;
            valve_datapoint.value.knows_current = true;
          } else {
            valve_datapoint.value.rolling_current_average *= 0.8;
            valve_datapoint.value.rolling_current_average += 0.2 * value.value;
          }
          continue;
        }
      } else if name.ends_with(VOLTAGE_SUFFIX) {
        let mut real_name = name.clone();
        let _ = real_name.split_off(real_name.len() - 2);

        if let Some(valve_datapoint) = tui_data.valves.get_mut(&real_name) {
          valve_datapoint.value.voltage = value.value;

          if !valve_datapoint.value.knows_voltage {
            valve_datapoint.value.rolling_voltage_average = value.value;
            valve_datapoint.value.knows_voltage = true;
          } else {
            valve_datapoint.value.rolling_voltage_average *= 0.8;
            valve_datapoint.value.rolling_voltage_average += 0.2 * value.value;
          }

          continue;
        }
      }
    }

    match tui_data.sensors.get_mut(name) {
      Some(x) => {
        x.value.measurement = value.clone();
        x.value.rolling_average *= 0.8;
        x.value.rolling_average += 0.2 * value.value;
      }
      None => {
        tui_data.sensors.add(
          name,
          SensorDatapoint {
            measurement: value.clone(),
            rolling_average: value.value,
          },
        );
        sort_needed = true;
      }
    }
  }
  if sort_needed {
    tui_data.sensors.sort_by_name();
  }
}

async fn send_abort(shared: &Shared) -> Result<(), anyhow::Error> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.abort().await
  } else {
    Ok(())
  }
}

/// A function called every display round that draws the ui and handles user
/// input.
///
/// This was removed from display due to certain functions returning generic
/// errors, which cause the serializer to have an aneurysm and thus not work
/// with async.
async fn display_round(
  terminal: &mut Terminal<CrosstermBackend<Stdout>>,
  tui_data: &mut TuiData,
  selected_tab: &mut usize,
  tick_rate: Duration,
  last_tick: &mut Instant,
  shared : &Shared
) -> bool {
  // Draw the TUI
  let _ = terminal.draw(|f| servo_ui(f, *selected_tab, tui_data));

  // Handle user input
  {
    // This is really overly drawn out, but it's manual error handling handled
    // internally to ensure that the generic "Error" returned doesn't mess with
    // async requirements.
    let poll_res = crossterm::event::poll(Duration::from_millis(0));

    if poll_res.is_err() {
      println!("Input polling failed : ");
      println!("{}", poll_res.unwrap_err());
      return false;
    }
    if poll_res.unwrap() {
      let read_res = event::read();
      if read_res.is_err() {
        println!("Input reading failed : ");
        println!("{}", read_res.unwrap_err());
        return false;
      }
      // If a quit command is recieved, return false to signal to quit
      if let Event::Key(key) = read_res.unwrap() {
        if key.kind == KeyEventKind::Press {
          /* if let KeyCode::Char('q') = key.code {
              return false;
          }
          if let KeyCode::Char('Q') = key.code {
              return false;
          } */
          if let KeyCode::Char('c') = key.code {
            if key.modifiers.contains(KeyModifiers::CONTROL) {
              return false;
            }
          }
          if let KeyCode::Char('C') = key.code {
            if key.modifiers.contains(KeyModifiers::CONTROL) {
              return false;
            }
          }
          if let KeyCode::Char('a') = key.code {
            if key.modifiers.contains(KeyModifiers::CONTROL) {
              let _ = send_abort(shared).await;
              return true;
            }
          }
          if let KeyCode::Char('A') = key.code {
            if key.modifiers.contains(KeyModifiers::CONTROL) {
              let _ = send_abort(shared).await;
              return true;
            }
          }
          
          if let KeyCode::F(20) = key.code {
            let _ = send_abort(shared).await;
            return true;
          }
        }
      }
    }
  }

  //
  if last_tick.elapsed() >= tick_rate {
    last_tick.clone_from(&Instant::now());
  }

  // If no quit command is recieved, return false to signal to continue
  true
}

/// Attempts to restore the terminal to the pre-servo TUI state
fn restore_terminal(
  terminal: &mut Terminal<CrosstermBackend<Stdout>>,
) -> Result<(), Box<dyn Error>> {
  // restore terminal
  disable_raw_mode()?;
  execute!(
    terminal.backend_mut(),
    LeaveAlternateScreen,
    DisableMouseCapture
  )?;
  terminal.show_cursor()?;

  //if let Err(err) = res {
  //    println!("{err:?}");
  //}

  Ok(())
}

/// The async function that drives the entire TUI.
/// Returns once it is manually quit (from within display_round)
pub async fn display(shared: Shared) -> io::Result<()> {
  // setup terminal
  enable_raw_mode()?;

  let mut stdout = io::stdout();
  execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;

  let backend = CrosstermBackend::new(stdout);
  let mut terminal = Terminal::new(backend)?;

  let mut system = System::new_all();

  // create tui_data and run the TUI
  let tick_rate = Duration::from_millis(100);
  let mut tui_data: TuiData = TuiData::new();
  let mut last_tick = Instant::now();
  let mut selected_tab: usize = 0;
  loop {
    update_information(&mut tui_data, &shared, &mut system).await;
    // Draw the TUI and handle user input, return if told to.
    if !display_round(
      &mut terminal,
      &mut tui_data,
      &mut selected_tab,
      tick_rate,
      &mut last_tick,
      &shared
    ).await {
      break;
    }
    // Wait until next tick
    sleep(tick_rate).await;
  }

  // Attempt to restore terminal
  if let Err(error) = restore_terminal(&mut terminal) {
    return Err(io::Error::new(io::ErrorKind::Other, error.to_string()));
  }

  Ok(())
}

/// Basic overhead ui drawing function.
/// Creates the main overarching tab and then draws the selected tab in the
/// remaining space.
fn servo_ui(f: &mut Frame, selected_tab: usize, tui_data: &TuiData) {
  let chunks: std::rc::Rc<[Rect]> = Layout::default()
    .direction(Direction::Vertical)
    .constraints([Constraint::Length(3), Constraint::Fill(1)])
    .split(f.size());

  let tab_menu = Tabs::new(vec!["Home", "Unused", "Unused"])
    .block(Block::default().title("Tabs").borders(Borders::ALL))
    .style(YJSP_STYLE)
    .highlight_style(YJSP_STYLE.fg(WHITE).bold())
    .select(selected_tab)
    .divider(symbols::line::VERTICAL);

  f.render_widget(tab_menu, chunks[0]);

  match selected_tab {
    0 => home_menu(f, chunks[1], tui_data),
    _ => bad_tab(f, chunks[1]),
  };
}

/// Tab render function used when the selected tab is invalid
fn bad_tab(_: &mut Frame, _: Rect) {}

/// Home tab render function displaying
/// System, Valves, and Sensor Information
fn home_menu(f: &mut Frame, area: Rect, tui_data: &TuiData) {
  let horizontal = Layout::default()
    .direction(Direction::Horizontal)
    .constraints([
      Constraint::Fill(1),
      Constraint::Length(40),
      Constraint::Length(75),
      Constraint::Length(45),
      Constraint::Fill(1),
    ])
    .split(area);

  // Filler for right side of screen to center actual data
  draw_empty(f, horizontal[0]);

  // System Info Column
  draw_system_info(f, horizontal[1], tui_data);

  // Valve Data Column
  draw_valves(f, horizontal[2], tui_data);

  // Sensor Data Column
  draw_sensors(f, horizontal[3], tui_data);

  // Filler for left side of screen to center actual data
  draw_empty(f, horizontal[4]);
}

/// Draws an empty table within an area. Used to fill a region with the
/// YJSP_STYLE's background.
fn draw_empty(f: &mut Frame, area: Rect) {
  let widths = [Constraint::Fill(1)];

  let empty_table: Table<'_> = Table::new(Vec::<Row>::new(), widths)
    .style(YJSP_STYLE)
    .header(
      Row::new(vec![Span::from("").into_centered_line()])
        .style(Style::new().bold()),
    );

  f.render_widget(empty_table, area);
}

/// Draws system info as listed in tui_data.system_data
/// See update_information for how this data is gathered
fn draw_system_info(f: &mut Frame, area: Rect, tui_data: &TuiData) {
  let all_systems: &StringLookupVector<SystemDatapoint> = &tui_data.system_data;

  // Styles used in table
  let name_style = YJSP_STYLE.bold();
  let data_style = YJSP_STYLE.fg(WHITE);

  // Make rows
  let mut rows: Vec<Row> = Vec::<Row>::with_capacity(all_systems.len() * 3);

  for name_datapoint_pair in all_systems.iter() {
    let name: &String = &name_datapoint_pair.name;
    let datapoint: &SystemDatapoint = &name_datapoint_pair.value;

    // Name of system
    rows.push(
      Row::new(vec![
        Cell::from(Span::from(name.clone()).into_centered_line()),
        Cell::from(Span::from("")),
        Cell::from(Span::from("")),
      ])
      .style(name_style),
    );

    //  CPU Usage
    rows.push(
      Row::new(vec![
        Cell::from(Span::from("CPU Usage").into_right_aligned_line()),
        Cell::from(
          Span::from(format!("{:.1}", datapoint.cpu_usage))
            .into_right_aligned_line(),
        ),
        Cell::from(Span::from("%")),
      ])
      .style(data_style),
    );

    //  Memory Usage
    rows.push(
      Row::new(vec![
        Cell::from(Span::from("Memory Usage").into_right_aligned_line()),
        Cell::from(
          Span::from(format!("{:.1}", datapoint.mem_usage))
            .into_right_aligned_line(),
        ),
        Cell::from(Span::from("%")),
      ])
      .style(data_style),
    );
  }

  //  ~Fixed size widths that can scale to a smaller window
  let widths = [Constraint::Max(20), Constraint::Max(12), Constraint::Max(2)];

  //  Make the table itself
  let sensor_table: Table<'_> = Table::new(rows, widths)
    .style(name_style)
    // It has an optional header, which is simply a Row always visible at the
    // top.
    .header(
      Row::new(vec![
        Span::from("Name").into_centered_line(),
        Span::from("Value").into_centered_line(),
        Line::from(""),
      ])
      .style(Style::new().bold())
      // To add space between the header and the rest of the rows, specify the
      // margin
      .bottom_margin(1),
    )
    // As any other widget, a Table can be wrapped in a Block.
    .block(Block::default().title("Systems").borders(Borders::ALL))
    // The selected row and its content can also be styled.
    .highlight_style(Style::new().reversed())
    // ...and potentially show a symbol in front of the selection.
    .highlight_symbol(">>");

  f.render_widget(sensor_table, area);
}

/// Draws valve states as listed in tui_data.valves
/// See update_information for how this data is gathered
fn draw_valves(f: &mut Frame, area: Rect, tui_data: &TuiData) {
  //  Get valve states from TUI
  let full_valves: &StringLookupVector<FullValveDatapoint> = &tui_data.valves;

  // Make rows
  let mut rows: Vec<Row> = Vec::<Row>::with_capacity(full_valves.len());
  for pair in full_valves.iter() {
    let name = &pair.name;
    let datapoint = &pair.value;

    // Get base style used in this row based on the actual (derived) state of
    // the valve
    let normal_style = get_full_row_style(datapoint.state.actual);
    let name_style = get_valve_name_style(datapoint.state.actual);

    // Determine rolling change of voltage and current via value - rolling
    // average of value as calculated by update_information. And color code the
    // change based on it's magnitude and sign (increasing / decreasing). Color
    // coding is based on fixed thresholds set for voltage and current
    // independently.
    let d_v = datapoint.voltage - datapoint.rolling_voltage_average;
    let d_v_style: Style;
    if d_v.abs() < 0.1 {
      d_v_style = normal_style;
    } else if d_v > 0.0 {
      d_v_style = normal_style.fg(Color::Green);
    } else {
      d_v_style = normal_style.fg(Color::Red);
    }

    let d_i: f64 = datapoint.current - datapoint.rolling_current_average;
    let d_i_style: Style;
    if d_i.abs() < 0.025 {
      d_i_style = normal_style;
    } else if d_i > 0.0 {
      d_i_style = normal_style.fg(Color::Green);
    } else {
      d_i_style = normal_style.fg(Color::Red);
    }

    let voltage_rows = if datapoint.knows_voltage {
      [
        Cell::from(
          Span::from(format!("{:.2}", datapoint.voltage))
            .into_right_aligned_line(),
        ), // Voltage
        Cell::from(
          Span::from(format!("{:+.3}", d_v)).into_right_aligned_line(),
        )
        .style(d_v_style),
      ]
    } else {
      [Cell::from(""), Cell::from("")]
    };

    let current_rows = if datapoint.knows_current {
      [
        Cell::from(
          Span::from(format!("{:.3}", datapoint.current))
            .into_right_aligned_line(),
        ), // Current
        Cell::from(
          Span::from(format!("{:+.3}", d_i)).into_right_aligned_line(),
        )
        .style(d_i_style), // Rolling change of current
      ]
    } else {
      [Cell::from(""), Cell::from("")]
    };

    // Make the actual row of info
    rows.push(
      Row::new(vec![
        Cell::from(
          Span::from(name.clone())
            .into_centered_line()
            .style(name_style),
        ), // Name of Valve
        voltage_rows[0].clone(),
        voltage_rows[1].clone(),
        current_rows[0].clone(),
        current_rows[1].clone(),
        // Actual / Derived state of valve
        Cell::from(
          Span::from(format!("{}", datapoint.state.actual))
            .into_centered_line(),
        )
        .style(get_state_style(datapoint.state.actual)),
        // Commanded state of valve
        Cell::from(
          Span::from(format!("{}", datapoint.state.commanded))
            .into_centered_line(),
        )
        .style(get_state_style(datapoint.state.commanded)),
      ])
      .style(normal_style),
    );
  }

  let widths = [
    Constraint::Length(12),
    Constraint::Length(7),
    Constraint::Length(8),
    Constraint::Length(8),
    Constraint::Length(9),
    Constraint::Length(12),
    Constraint::Length(12),
  ];

  let valve_table: Table<'_> = Table::new(rows, widths)
    .style(YJSP_STYLE)
    // It has an optional header, which is simply a Row always visible at
    // the top.
    .header(
      Row::new(vec![
        Span::from("Name").into_centered_line(),
        Span::from("Voltage").into_right_aligned_line(),
        Line::from(""),
        Span::from("Current").into_right_aligned_line(),
        Line::from(""),
        Span::from("Derived").into_centered_line(),
        Span::from("Commanded").into_centered_line(),
      ])
      .style(Style::new().bold())
      // To add space between the header and the rest of the rows, specify the
      // margin
      .bottom_margin(1),
    )
    // As any other widget, a Table can be wrapped in a Block.
    .block(Block::default().title("Valves").borders(Borders::ALL))
    // The selected row and its content can also be styled.
    .highlight_style(Style::new().reversed())
    // ...and potentially show a symbol in front of the selection.
    .highlight_symbol(">>");

  f.render_widget(valve_table, area);
}

/// Draws sensors as listed in tui_data.sensors
/// See update_information for how this data is gathered
fn draw_sensors(f: &mut Frame, area: Rect, tui_data: &TuiData) {
  //  Get sensor measurements from TUI
  let full_sensors: &StringLookupVector<SensorDatapoint> = &tui_data.sensors;

  //  Styles used in table
  let normal_style = YJSP_STYLE;
  let data_style = normal_style.fg(WHITE);

  //  Make rows
  let mut rows: Vec<Row> = Vec::<Row>::with_capacity(full_sensors.len());

  for name_datapoint_pair in full_sensors.iter() {
    let name: &String = &name_datapoint_pair.name;
    let datapoint: &SensorDatapoint = &name_datapoint_pair.value;

    // Determine rolling change of the measurement value via value - rolling
    // average of value as calculated by update_information
    // And color code the change based on it's magnitude and sign
    // (increasing / decreasing)
    let d_v = datapoint.measurement.value - datapoint.rolling_average;
    let d_v_style: Style;

    // As values can have vastly differing units, the color code change is 1%
    // of the value, with a minimum change threshold of 0.01 if the value is
    // less than 1
    let value_magnitude_min: f64 = 1.0;
    let value_magnitude =
      datapoint.rolling_average.abs().max(value_magnitude_min);

    // If the change is > 1% the rolling averages value, then it's considered
    // significant enough to highlight. Since sensors have a bigger potential
    // range, a flat delta threshold is a bad idea as it would require
    // configuration.
    if d_v.abs() / value_magnitude < 0.01 {
      d_v_style = data_style;
    } else if d_v > 0.0 {
      d_v_style = normal_style.fg(Color::Green);
    } else {
      d_v_style = normal_style.fg(Color::Red);
    }

    rows.push(
      Row::new(vec![
        Cell::from(
          Span::from(name.clone())
            .style(normal_style)
            .bold()
            .into_right_aligned_line(),
        ), // Sensor Name
        Cell::from(
          Span::from(format!("{:.3}", datapoint.measurement.value))
            .into_right_aligned_line()
            .style(data_style),
        ), // Measurement value
        Cell::from(
          Span::from(format!("{}", datapoint.measurement.unit))
            .into_left_aligned_line()
            .style(data_style.fg(GREY)),
        ), // Measurement unit
        Cell::from(Span::from(format!("{:+.3}", d_v)).into_left_aligned_line())
          .style(d_v_style), /* Rolling Change of value (see
                              * update_information) */
      ])
      .style(normal_style),
    );
  }

  //  ~Fixed Lengths with some room to expand
  let widths = [
    Constraint::Min(12),
    Constraint::Min(10),
    Constraint::Length(5),
    Constraint::Min(14),
  ];

  //  Make the table itself
  let sensor_table: Table<'_> = Table::new(rows, widths)
    .style(normal_style)
    // It has an optional header, which is simply a Row always visible at the
    // top.
    .header(
      Row::new(vec![
        Span::from("Name").into_right_aligned_line(),
        Span::from("Value").into_right_aligned_line(),
        Span::from("Unit").into_centered_line(),
        Span::from("Rolling Change").into_centered_line(),
      ])
      .style(Style::new().bold())
      // To add space between the header and the rest of the rows, specify the
      // margin
      .bottom_margin(1),
    )
    // As any other widget, a Table can be wrapped in a Block.
    .block(Block::default().title("Sensors").borders(Borders::ALL))
    // The selected row and its content can also be styled.
    .highlight_style(Style::new().reversed())
    // ...and potentially show a symbol in front of the selection.
    .highlight_symbol(">>");

  //  Render
  f.render_widget(sensor_table, area);
}

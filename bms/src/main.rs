pub mod adc;
pub mod command;
pub mod communication;
pub mod state;

fn main() {
  let mut state = state::State::Init;

  loop {
    state = state.next();
  }
}

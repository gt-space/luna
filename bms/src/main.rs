pub mod command;
pub mod communication;
pub mod adc;
pub mod state;

fn main() {
  let mut state = state::State::Init;
  
  loop {
    state = state.next();
  }
}

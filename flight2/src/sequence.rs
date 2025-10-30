use common::comm::{SensorType, Sequence, flight::SequenceDomainCommand};
use std::{collections::HashMap, io, os::unix::net::UnixDatagram, process::{Child, Command}};
use crate::Mappings;

pub(crate) type Sequences = HashMap<String, Child>;

fn run(mappings: &Mappings, sequence: &Sequence) -> io::Result<Child> {
    let mut script = String::from("from common import *;");
    script.push_str("OPEN = ValveState.Open;");
    script.push_str("CLOSED = ValveState.Closed;");
    for mapping in mappings {
        let definition = match mapping.sensor_type {
            SensorType::Valve => format!("{0} = Valve('{0}');", mapping.text_id),
            _ => format!("{0} = Sensor('{0}');", mapping.text_id),
        };

        script.push_str(&definition);
    }
    
    script.push_str(&sequence.script);
    Command::new("python3")
        .args(["-c", &script])
        .spawn()
}

pub(crate) fn execute(mappings: &Mappings, sequence: &Sequence, sequences: &mut Sequences) {
    if let Some(running) = sequences.get_mut(&sequence.name) {
        match running.try_wait() {
            Ok(Some(_)) => {},
            Ok(None) => {
                println!("The '{}' sequence is already running. Stop it before re-attempting execution.", sequence.name);
                return;
            },
            Err(e) => {
                eprintln!("Another '{}' sequence was previously ran, but it's status couldn't be determined: {e}", sequence.name);
                return;
            },
        }
    }
    
    let process = match run(mappings, &sequence) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error in running python3: {e}");
            return;
        }
    };

    sequences.insert(sequence.name.clone(), process);
}

pub(crate) fn kill(sequences: &mut Sequences, name: &String) -> io::Result<()> {
    let sequence = match sequences.get_mut(name) {
        Some(c) => {
            if let Ok(Some(_)) = c.try_wait() {
                println!("A sequence named '{name}' isn't running.");
                return Ok(());
            }

            c
        }
        None => {
            println!("A sequence named '{name}' isn't running.");
            return Ok(());
        }
    };

    sequence.kill()
}

pub(crate) fn pull_commands<'a>(socket: &UnixDatagram) -> Vec<SequenceDomainCommand> {
    let mut buf: [u8; 1024] = [0; 1024];
    let mut commands = Vec::new();

    loop {
        let size = match socket.recv(&mut buf) {
            Ok(s) => s,
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
            Err(e) => {
                eprintln!("Error in receiving from sequence command socket: {e}");
                break;
            }
        };

        let command = match postcard::from_bytes::<SequenceDomainCommand>(&buf[..size]) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Error in deserializing SequenceDomainCommand from sequence: {e}");
                continue;
            }
        };

        commands.push(command);
    }

    commands
}
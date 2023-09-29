use fs_protobuf_rust::compiled::mcfs::command;
use fs_protobuf_rust::compiled::mcfs::board;
use fs_protobuf_rust::compiled::mcfs::procedure;
use quick_protobuf::{serialize_into_vec, deserialize_from_slice};

#[test]
fn sample_procedure() {


    let command = command::Command {
        command: command::mod_Command::OneOfcommand::click_valve(
            command::ClickValve { 
                valve: (Some(board::ChannelIdentifier {board_id: 10, channel_type: board::ChannelType::VALVE, channel: 0})), 
                state: (board::ValveState::VALVE_OPEN)
    })};

    let procedure = procedure::Procedure {
      name: std::borrow::Cow::Borrowed("Test Procedure"),
      stages: vec![procedure::Stage {
        name: std::borrow::Cow::Borrowed("Stage 1"),
        sequence: vec![procedure::SequenceAction {
          command: Some(command),
          time: 1221
        }]
      }]
    };

    let procedure_serialized = serialize_into_vec(&procedure).expect("Cannot serialize procedure");
    let procedure_deserialized: procedure::Procedure = deserialize_from_slice(&procedure_serialized).expect("Cannot deserialize procedure");

  
    assert!(procedure_deserialized.name == procedure.name);
    assert!(procedure_deserialized.stages.len() == procedure.stages.len());

}
use fs_protobuf_rust::compiled::mcfs::command;
use fs_protobuf_rust::compiled::mcfs::core;
use fs_protobuf_rust::compiled::mcfs::device;
use fs_protobuf_rust::compiled::mcfs::status;
use fs_protobuf_rust::compiled::google::protobuf::Timestamp;
use std::borrow::Cow;

#[test]
fn different_submessages() {


    let command = command::Command {
        command: command::mod_Command::OneOfcommand::click_valve(
            command::ClickValve { 
                valve: (Some(device::NodeIdentifier {board_id: 10, channel: device::Channel::VALVE, node_id: 0})), 
                state: (device::ValveState::VALVE_OPEN)
    })};

    let status = status::Status {
        status_message: Cow::Borrowed("Not working, did you check valve?"),
        status: status::mod_Status::OneOfstatus::None
    };
    
    let command_message = core::Message {
        timestamp: Some(Timestamp {seconds: 1, nanos: 100}),
        board_id: 5,
        content: core::mod_Message::OneOfcontent::command(command)
    };

    let status_message = core::Message {
        timestamp: Some(Timestamp {seconds: 1, nanos: 200}),
        board_id: 5,
        content: core::mod_Message::OneOfcontent::status(status)
    };

    assert!(parse_message(command_message));
    assert!(parse_message(status_message));


}

fn parse_message(message: core::Message) -> bool {
    match message.content {
        core::mod_Message::OneOfcontent::command(..) => true,
        core::mod_Message::OneOfcontent::data(..) => true,
        core::mod_Message::OneOfcontent::status(..) => true,
        _ => false
    }
}
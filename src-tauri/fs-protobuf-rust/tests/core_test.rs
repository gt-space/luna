use fs_protobuf_rust::compiled::mcfs::command;
use fs_protobuf_rust::compiled::mcfs::core;
use fs_protobuf_rust::compiled::mcfs::board;
use fs_protobuf_rust::compiled::mcfs::status;
use fs_protobuf_rust::compiled::google::protobuf::Timestamp;
use std::borrow::Cow;

#[test]
fn different_submessages() {


    let command = command::Command {
        command: command::mod_Command::OneOfcommand::click_valve(
            command::ClickValve { 
                valve: (Some(board::ChannelIdentifier {board_id: 10, channel_type: board::ChannelType::VALVE, channel: 0})), 
                state: (board::ValveState::VALVE_OPEN)
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
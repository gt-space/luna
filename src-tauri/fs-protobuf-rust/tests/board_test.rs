use fs_protobuf_rust::compiled::mcfs::board;
use quick_protobuf::{serialize_into_vec, deserialize_from_slice};

#[test]
fn node_identifier_serialization() {
    let node = board::ChannelIdentifier {
        board_id: 10,
        channel_type: board::ChannelType::GPIO,
        channel: 0
    };

    let node_serialized = serialize_into_vec(&node).expect("Cannot serialize `node`");
    let node_deserialized: board::ChannelIdentifier = deserialize_from_slice(&node_serialized).expect("Cannot deserialize node");

    assert_eq!(node_deserialized.channel_type, board::ChannelType::GPIO);
    assert_eq!(node_deserialized.channel, 0);
    
}
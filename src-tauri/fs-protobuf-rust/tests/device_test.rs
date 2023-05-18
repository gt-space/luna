use fs_protobuf_rust::compiled::mcfs::device;
use quick_protobuf::{serialize_into_vec, deserialize_from_slice};

#[test]
fn node_identifier_serialization() {
    let node = device::NodeIdentifier {
        board_id: 10,
        channel: device::Channel::GPIO,
        node_id: 0
    };

    let node_serialized = serialize_into_vec(&node).expect("Cannot serialize `node`");
    let node_deserialized: device::NodeIdentifier = deserialize_from_slice(&node_serialized).expect("Cannot deserialize node");

    assert_eq!(node_deserialized.channel, device::Channel::GPIO);
    assert_eq!(node_deserialized.node_id, 0);
    
}
//! Unit tests for the functions declared in `servo/src/server`.
//!
//! Each source file is pulled into a dedicated module with `include!` so that
//! private helpers can be exercised without changing any source files. The
//! `server` shim below re-exports the already-compiled library items so paths
//! such as `crate::server::telemetry::...` inside the included sources resolve
//! from this integration-test crate.
//!
//! Only functions that can be exercised without context from outside the
//! server directory are tested. The remaining functions are documented here
//! with the additional context they would require:
//!
//! - `database::Database::migrate` / `migrate_to`: need the SQL scripts in
//!   `servo/src/migrations`, which live outside the server directory.
//! - `database::Database::active_mappings`: needs the `NodeMappings` schema
//!   created by those external migrations.
//! - `database::Database::log_vehicle_state`: runs an infinite loop waiting on
//!   telemetry notifications; it can only be observed inside a long-lived
//!   server runtime.
//! - `flight::FlightComputer::send_mappings`, `send_all_abort_configs`, and
//!   `update`: need a connected flight computer plus the database schema from
//!   the external migration scripts.
//! - `flight::FlightComputer::send_abort_stage_config`, `send_sequence`,
//!   `stop_sequence`, `abort`, and `send_trigger`: need a live flight-computer
//!   TCP peer to receive the length-prefixed control messages.
//! - `flight::auto_connect` and `receive_vehicle_state`: infinite loops bound
//!   to the fixed ports 5025 and 7201, requiring real flight hardware and an
//!   external stop signal.
//! - `flight::bind_vehicle_state_socket`: binds the global UDP port 7201,
//!   which cannot be isolated from other tests or running servers.
//! - `mod::Server::serve`: binds the global HTTP port 7200, needs a Tokio
//!   runtime, a shutdown task, and all route state; it is only meaningful as
//!   the top-level server entry point.
//! - `mod::wait_for_display_end`: private shutdown plumbing tied to `serve`;
//!   reaching it requires including `src/server/mod.rs` and its whole module
//!   tree into the test crate.
//! - All route handlers in `routes/command.rs`, `routes/sequence.rs`,
//!   `routes/trigger.rs`, and the database-backed handlers in `routes/data.rs`
//!   and `routes/mappings.rs`: need either a connected flight computer, a
//!   database schema supplied by external migrations, an HTTP/WebSocket
//!   client, or the GUI's `.servo` directory. The pure validation and
//!   formatting helpers in those files are tested below.
//! - `routes/data.rs::export`: additionally needs the `.servo` directory under
//!   the current user's home path for temporary HDF5 exports.
//! - `routes/data.rs::forward_data`: needs a WebSocket client and an active
//!   telemetry stream.
//! - `routes/mappings.rs::refresh_radio_schema`: needs the `NodeMappings`
//!   schema from the external migration scripts.

#[allow(dead_code)]
mod server {
    pub use servo::server::*;
}

use ntest::timeout;
// `server/flight.rs` and `server/database.rs` refer to `super::Shared` and
// `super::telemetry` when included as child modules of this test crate. The
// local `Shared` mirrors the library type but uses the included copy of
// `FlightComputer`, so the private fields in `flight_src` stay usable.
#[allow(dead_code)]
pub use servo::server::Database;
#[allow(dead_code)]
pub mod telemetry {
    pub use servo::server::telemetry::*;
}

#[allow(dead_code)]
pub struct Shared {
    pub database: Database,
    pub flight: std::sync::Arc<(
        tokio::sync::Mutex<Option<flight_src::FlightComputer>>,
        tokio::sync::Notify,
    )>,
    pub ground: std::sync::Arc<(
        tokio::sync::Mutex<Option<flight_src::FlightComputer>>,
        tokio::sync::Notify,
    )>,
    pub telemetry: servo::server::TelemetryState,
    pub radio_schema: std::sync::Arc<tokio::sync::Mutex<servo::server::RadioSchemaCache>>,
}

mod error_src {
    #![allow(dead_code)]

    use ntest::timeout;

    include!("../src/server/error.rs");

    fn sqlite_error() -> rusqlite::Error {
        rusqlite::Connection::open_in_memory()
            .unwrap()
            .prepare("this is not valid sql")
            .unwrap_err()
    }

    #[test]
    fn bad_request_uses_the_bad_request_status() {
        let error = bad_request("missing field");

        match error {
            ServerError::Raw(message, status) => {
                assert_eq!(message, "missing field");
                assert_eq!(status, StatusCode::BAD_REQUEST);
            }
            other => panic!("expected a Raw error, got {other:?}"),
        }
    }

    #[test]
    fn bad_request_stringifies_arbitrary_values() {
        let error = bad_request(42);

        assert!(matches!(error, ServerError::Raw(ref message, _) if message == "42"));
    }

    #[test]
    fn not_found_uses_the_not_found_status() {
        let error = not_found("no such sequence");

        match error {
            ServerError::Raw(message, status) => {
                assert_eq!(message, "no such sequence");
                assert_eq!(status, StatusCode::NOT_FOUND);
            }
            other => panic!("expected a Raw error, got {other:?}"),
        }
    }

    #[test]
    fn not_found_accepts_owned_strings() {
        let error = not_found(String::from("missing"));

        assert!(matches!(error, ServerError::Raw(ref message, _) if message == "missing"));
    }

    #[test]
    fn internal_uses_the_internal_server_error_status() {
        let error = internal("database failure");

        match error {
            ServerError::Raw(message, status) => {
                assert_eq!(message, "database failure");
                assert_eq!(status, StatusCode::INTERNAL_SERVER_ERROR);
            }
            other => panic!("expected a Raw error, got {other:?}"),
        }
    }

    #[test]
    fn internal_preserves_formatted_messages() {
        let error = internal(format!("failed with {}", 7));

        assert!(matches!(error, ServerError::Raw(ref message, _) if message == "failed with 7"));
    }

    #[test]
    fn sqlite_errors_convert_to_the_sql_variant() {
        let error = ServerError::from(sqlite_error());

        assert!(matches!(error, ServerError::Sql(_)));
    }

    #[test]
    fn converted_sqlite_errors_report_an_internal_error() {
        let response = ServerError::from(sqlite_error()).into_response();

        assert_eq!(response.status(), StatusCode::INTERNAL_SERVER_ERROR);
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn sql_errors_render_their_message() {
        let message = sqlite_error().to_string();
        let response = ServerError::Sql(sqlite_error()).into_response();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();

        assert_eq!(body.as_ref(), message.as_bytes());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn raw_errors_render_their_status_and_message() {
        let response = bad_request("bad input").into_response();
        let status = response.status();
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();

        assert_eq!(status, StatusCode::BAD_REQUEST);
        assert_eq!(body.as_ref(), b"bad input");
    }
}

mod telemetry_src {
    #![allow(dead_code)]

    include!("../src/server/telemetry.rs");

    use std::time::Duration;

    use common::comm::Computer;
    use ntest::timeout;

    fn mapping(text_id: &str, board_id: &str, sensor_type: SensorType) -> NodeMapping {
        NodeMapping {
            text_id: text_id.to_owned(),
            board_id: board_id.to_owned(),
            sensor_type,
            channel: 1,
            computer: Computer::Flight,
            max: None,
            min: None,
            calibrated_offset: 0.0,
            powered_threshold: None,
            normally_closed: None,
        }
    }

    #[test]
    fn query_value_matches_the_http_query_values() {
        assert_eq!(TelemetrySource::Umbilical.query_value(), "umbilical");
        assert_eq!(TelemetrySource::Radio.query_value(), "tel");
    }

    #[test]
    fn query_value_is_distinct_for_each_source() {
        assert_ne!(
            TelemetrySource::Umbilical.query_value(),
            TelemetrySource::Radio.query_value()
        );
    }

    #[test]
    fn snapshot_table_matches_the_database_tables() {
        assert_eq!(
            TelemetrySource::Umbilical.snapshot_table(),
            "VehicleSnapshots"
        );
        assert_eq!(TelemetrySource::Radio.snapshot_table(), "RadioTelemetry");
    }

    #[test]
    fn snapshot_table_round_trips_through_query_value() {
        for source in [TelemetrySource::Umbilical, TelemetrySource::Radio] {
            assert!(source.snapshot_table().len() > source.query_value().len());
        }
    }

    #[test]
    fn from_str_accepts_every_source_name() {
        assert_eq!(
            "umbilical".parse::<TelemetrySource>(),
            Ok(TelemetrySource::Umbilical)
        );
        assert_eq!(
            "radio".parse::<TelemetrySource>(),
            Ok(TelemetrySource::Radio)
        );
        assert_eq!("tel".parse::<TelemetrySource>(), Ok(TelemetrySource::Radio));
    }

    #[test]
    fn from_str_rejects_unknown_names() {
        assert!("".parse::<TelemetrySource>().is_err());
        assert!("RADIO".parse::<TelemetrySource>().is_err());
        assert!("ethernet".parse::<TelemetrySource>().is_err());
    }

    #[test]
    fn display_uses_human_readable_names() {
        assert_eq!(TelemetrySource::Umbilical.to_string(), "umbilical");
        assert_eq!(TelemetrySource::Radio.to_string(), "radio");
    }

    #[test]
    fn display_differs_from_the_query_value_for_radio() {
        assert_ne!(
            TelemetrySource::Radio.to_string(),
            TelemetrySource::Radio.query_value()
        );
        assert_eq!(
            TelemetrySource::Umbilical.to_string(),
            TelemetrySource::Umbilical.query_value()
        );
    }

    #[test]
    #[timeout(5000)]
    fn live_telemetry_default_has_no_statistics() {
        let telemetry = LiveTelemetry::default();

        assert!(telemetry.last_vehicle_state.0.blocking_lock().is_none());
        assert!(telemetry.rolling_duration.0.blocking_lock().is_none());
        assert!(telemetry.packet_size.0.blocking_lock().is_none());
    }

    #[test]
    #[timeout(5000)]
    fn live_telemetry_default_has_a_fresh_vehicle_state() {
        let telemetry = LiveTelemetry::default();
        let state = telemetry.vehicle.0.blocking_lock();

        assert!(state.sensor_readings.is_empty());
        assert!(state.valve_states.is_empty());
    }

    #[test]
    fn telemetry_state_new_creates_independent_sources() {
        let state = TelemetryState::new();

        assert!(!std::ptr::eq(&state.umbilical, &state.radio));
        assert!(!Arc::ptr_eq(&state.umbilical.vehicle, &state.radio.vehicle));
    }

    #[test]
    #[timeout(5000)]
    fn telemetry_state_new_starts_empty() {
        let state = TelemetryState::new();

        assert!(state.radio.packet_size.0.blocking_lock().is_none());
        assert!(state.umbilical.packet_size.0.blocking_lock().is_none());
    }

    #[test]
    fn telemetry_state_get_returns_the_requested_source() {
        let state = TelemetryState::new();

        assert!(std::ptr::eq(
            state.get(TelemetrySource::Umbilical),
            &state.umbilical
        ));
    }

    #[test]
    fn telemetry_state_get_is_stable_across_calls() {
        let state = TelemetryState::new();

        let first = state.get(TelemetrySource::Radio);
        let second = state.get(TelemetrySource::Radio);

        assert!(std::ptr::eq(first, second));
        assert!(std::ptr::eq(first, &state.radio));
    }

    #[test]
    fn radio_schema_cache_has_no_schema_before_refresh() {
        let cache = RadioSchemaCache::default();

        assert!(cache.schema().is_none());
        assert!(cache.active_mappings.is_empty());
    }

    #[test]
    fn radio_schema_cache_refresh_stores_the_mappings_and_schema() {
        let mut cache = RadioSchemaCache::default();
        let mappings = vec![mapping("VLV01", "sam-21", SensorType::Valve)];

        cache.refresh(mappings.clone());

        assert_eq!(cache.active_mappings, mappings);
        assert!(cache.schema().is_some());
        assert_eq!(cache.schema().unwrap().valve_keys(), ["VLV01"]);
    }

    #[test]
    fn radio_schema_cache_schema_reflects_the_latest_refresh() {
        let mut cache = RadioSchemaCache::default();

        cache.refresh(vec![mapping("VLV01", "sam-21", SensorType::Valve)]);
        assert_eq!(cache.schema().unwrap().valve_keys(), ["VLV01"]);

        cache.refresh(vec![mapping("VLV02", "sam-31", SensorType::Valve)]);
        assert_eq!(cache.schema().unwrap().valve_keys(), ["VLV02"]);
    }

    #[test]
    fn radio_schema_cache_schema_is_none_only_before_the_first_refresh() {
        let mut cache = RadioSchemaCache::default();
        assert!(cache.schema().is_none());

        cache.refresh(Vec::new());
        assert!(cache.schema().is_some());
    }

    #[test]
    fn build_radio_schema_keeps_only_vehicle_mappings() {
        let schema = build_radio_schema(&[
            mapping("VLV01", "sam-21", SensorType::Valve),
            mapping("VLV01_I", "sam-21", SensorType::RailCurrent),
            mapping("VLV01_V", "sam-21", SensorType::RailVoltage),
            mapping("PT01", "sam-21", SensorType::Pt),
            mapping("GROUND_PT", "sam-01", SensorType::Pt),
        ]);

        assert_eq!(schema.valve_keys(), ["VLV01"]);
        assert_eq!(schema.sensor_keys(), ["PT01"]);
    }

    #[test]
    fn build_radio_schema_handles_no_mappings() {
        let schema = build_radio_schema(&[]);

        assert!(schema.valve_keys().is_empty());
        assert!(schema.sensor_keys().is_empty());
    }

    #[test]
    fn should_omit_radio_sensor_omits_valve_helpers() {
        let valve_keys = HashSet::from([String::from("VLV01")]);

        assert!(should_omit_radio_sensor("VLV01_V", &valve_keys));
        assert!(should_omit_radio_sensor("VLV01_I", &valve_keys));
    }

    #[test]
    fn should_omit_radio_sensor_keeps_unrelated_and_plain_sensors() {
        let valve_keys = HashSet::from([String::from("VLV01")]);

        assert!(!should_omit_radio_sensor("VLV02_V", &valve_keys));
        assert!(!should_omit_radio_sensor("VLV01", &valve_keys));
        assert!(!should_omit_radio_sensor("PT01", &valve_keys));
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn update_live_telemetry_replaces_state_and_statistics() {
        let telemetry = LiveTelemetry::default();
        let mut state = VehicleState::new();
        state.sensor_readings.insert(
            String::from("PT01"),
            common::comm::Measurement {
                value: 3.5,
                unit: common::comm::sam::Unit::Psi,
            },
        );

        update_live_telemetry(&telemetry, state.clone(), 512).await;

        assert_eq!(*telemetry.vehicle.0.lock().await, state);
        assert_eq!(*telemetry.packet_size.0.lock().await, Some(512));
        assert!(telemetry.last_vehicle_state.0.lock().await.is_some());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn update_live_telemetry_builds_a_positive_rolling_duration() {
        let telemetry = LiveTelemetry::default();

        assert!(telemetry.rolling_duration.0.lock().await.is_none());

        update_live_telemetry(&telemetry, VehicleState::new(), 1).await;
        tokio::time::sleep(Duration::from_millis(5)).await;
        update_live_telemetry(&telemetry, VehicleState::new(), 2).await;

        let rolling = *telemetry.rolling_duration.0.lock().await;
        assert!(rolling.is_some());
        assert!(rolling.unwrap() > 0.0);
    }
}

mod flight_src {
    #![allow(dead_code)]

    include!("../src/server/flight.rs");

    use std::{io::ErrorKind, time::Duration};

    use ntest::timeout;

    #[repr(align(8))]
    struct AlignedControl([u8; 64]);

    async fn connected_flight() -> (FlightComputer, TcpListener) {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let stream = TcpStream::connect(address).await.unwrap();
        let database = Database::volatile().unwrap();

        (FlightComputer { database, stream }, listener)
    }

    async fn read_prefixed_message(peer: &mut TcpStream) -> Vec<u8> {
        let mut length = [0u8; 2];
        peer.read_exact(&mut length).await.unwrap();
        let mut bytes = vec![0; u16::from_be_bytes(length) as usize];
        peer.read_exact(&mut bytes).await.unwrap();
        bytes
    }

    #[test]
    fn cmsg_align_rounds_up_to_word_boundaries() {
        assert_eq!(cmsg_align(0), 0);
        assert_eq!(cmsg_align(1), size_of::<usize>());
        assert_eq!(cmsg_align(size_of::<usize>()), size_of::<usize>());
        assert_eq!(cmsg_align(size_of::<usize>() + 1), 2 * size_of::<usize>());
    }

    #[test]
    fn cmsg_align_is_idempotent() {
        for length in [1, 7, 16, 33] {
            let aligned = cmsg_align(length);
            assert_eq!(cmsg_align(aligned), aligned);
            assert!(aligned >= length);
        }
    }

    #[test]
    fn cmsg_data_offset_is_the_aligned_header_size() {
        assert_eq!(cmsg_data_offset(), cmsg_align(size_of::<libc::cmsghdr>()));
    }

    #[test]
    fn cmsg_data_offset_is_word_aligned() {
        assert_eq!(cmsg_data_offset() % size_of::<usize>(), 0);
    }

    #[test]
    fn control_with_radio_dscp_selects_radio() {
        let mut control = AlignedControl([0u8; 64]);
        let header = control.0.as_mut_ptr().cast::<libc::cmsghdr>();

        unsafe {
            std::ptr::write(
                header,
                libc::cmsghdr {
                    cmsg_len: (cmsg_data_offset() + 1) as _,
                    cmsg_level: libc::IPPROTO_IP,
                    cmsg_type: libc::IP_TOS,
                },
            );
            *control.0.as_mut_ptr().add(cmsg_data_offset()) = RADIO_TELEMETRY_DSCP << 2;
        }

        assert_eq!(
            telemetry_source_from_control(
                control.0.as_mut_ptr().cast(),
                cmsg_align(cmsg_data_offset() + 1),
            ),
            TelemetrySource::Radio
        );
    }

    #[test]
    fn control_without_radio_dscp_selects_umbilical() {
        let empty = AlignedControl([0u8; 64]);
        assert_eq!(
            telemetry_source_from_control(empty.0.as_ptr().cast_mut().cast(), 0),
            TelemetrySource::Umbilical
        );

        let mut control = AlignedControl([0u8; 64]);
        unsafe {
            std::ptr::write(
                control.0.as_mut_ptr().cast::<libc::cmsghdr>(),
                libc::cmsghdr {
                    cmsg_len: (cmsg_data_offset() + 1) as _,
                    cmsg_level: libc::IPPROTO_IP,
                    cmsg_type: libc::IP_TOS,
                },
            );
            *control.0.as_mut_ptr().add(cmsg_data_offset()) = 0;
        }
        assert_eq!(
            telemetry_source_from_control(
                control.0.as_mut_ptr().cast(),
                cmsg_align(cmsg_data_offset() + 1),
            ),
            TelemetrySource::Umbilical
        );
    }

    #[test]
    fn sockaddr_to_socket_addr_reads_ipv4_addresses() {
        let mut storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };
        storage.ss_family = libc::AF_INET as _;
        let address = unsafe { &mut *(&mut storage as *mut _ as *mut libc::sockaddr_in) };
        address.sin_addr.s_addr = u32::from_ne_bytes([127, 0, 0, 1]);
        address.sin_port = u16::from_ne_bytes([0x1F, 0x90]);

        let converted = sockaddr_to_socket_addr(&storage, size_of::<libc::sockaddr_in>() as _)
            .unwrap()
            .to_string();

        assert_eq!(converted, "127.0.0.1:8080");
    }

    #[test]
    fn sockaddr_to_socket_addr_reads_ipv6_addresses() {
        let mut storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };
        storage.ss_family = libc::AF_INET6 as _;
        let address = unsafe { &mut *(&mut storage as *mut _ as *mut libc::sockaddr_in6) };
        address.sin6_addr.s6_addr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
        address.sin6_port = u16::from_ne_bytes([0x1F, 0x90]);

        let converted = sockaddr_to_socket_addr(&storage, size_of::<libc::sockaddr_in6>() as _)
            .unwrap()
            .to_string();

        assert_eq!(converted, "[::1]:8080");
    }

    #[test]
    fn sockaddr_to_socket_addr_rejects_unknown_families() {
        let storage: libc::sockaddr_storage = unsafe { std::mem::zeroed() };

        let error = sockaddr_to_socket_addr(&storage, size_of::<libc::sockaddr_storage>() as _)
            .expect_err("an unspecified family should not convert");

        assert_eq!(error.kind(), ErrorKind::InvalidData);
    }

    #[test]
    fn format_radio_decode_error_marks_schema_mismatches() {
        let schema_error =
            VehicleStateCompaqError::Schema(common::comm::VehicleStateSchemaError::TooManyValves);
        let message = format_radio_decode_error(schema_error);

        assert!(message.starts_with("radio schema mismatch:"));
        assert!(message.contains("TooManyValves"));

        let message = format_radio_decode_error(VehicleStateCompaqError::PolicyDesynchronized);
        assert!(message.starts_with("radio schema mismatch:"));
    }

    #[test]
    fn format_radio_decode_error_leaves_other_errors_alone() {
        let error = VehicleStateCompaqError::Serialize(postcard::Error::SerializeBufferFull);
        let expected = format!("{error:?}");

        assert_eq!(format_radio_decode_error(error), expected);
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn check_closed_is_false_for_an_open_connection() {
        let (flight, listener) = connected_flight().await;
        let (_peer, _) = listener.accept().await.unwrap();

        assert!(!flight.check_closed());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn check_closed_is_true_after_the_peer_disconnects() {
        let (flight, listener) = connected_flight().await;
        let (peer, _) = listener.accept().await.unwrap();
        drop(peer);
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(flight.check_closed());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn get_ip_returns_the_peer_address() {
        let (flight, listener) = connected_flight().await;
        let (_peer, _) = listener.accept().await.unwrap();

        assert_eq!(flight.get_ip().await.unwrap().to_string(), "127.0.0.1");
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn get_port_returns_the_peer_port() {
        let (flight, listener) = connected_flight().await;
        let (_peer, _) = listener.accept().await.unwrap();

        assert_eq!(
            flight.get_port().await.unwrap(),
            listener.local_addr().unwrap().port()
        );
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn send_bytes_writes_a_big_endian_length_prefix() {
        let (mut flight, listener) = connected_flight().await;
        let (mut peer, _) = listener.accept().await.unwrap();
        let payload = b"hello world";

        flight.send_bytes(payload).await.unwrap();

        assert_eq!(read_prefixed_message(&mut peer).await, payload);
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn send_bytes_rejects_messages_larger_than_u16() {
        let (mut flight, _listener) = connected_flight().await;
        let payload = vec![0u8; u16::MAX as usize + 1];

        let error = flight.send_bytes(&payload).await.unwrap_err();

        assert_eq!(error.kind(), ErrorKind::InvalidInput);
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn recv_vehicle_state_packet_reads_a_datagram_from_the_source() {
        let socket = StdUdpSocket::bind("127.0.0.1:0").unwrap();
        let remote = socket.local_addr().unwrap();
        let sender = StdUdpSocket::bind("127.0.0.1:0").unwrap();
        sender.send_to(b"telemetry", remote).unwrap();
        let mut frame = [0u8; 128];
        let mut control = [0u8; 128];

        let (size, source, address) =
            recv_vehicle_state_packet(&socket, &mut frame, &mut control).unwrap();

        assert_eq!(size, b"telemetry".len());
        assert_eq!(source, TelemetrySource::Umbilical);
        assert_eq!(address, sender.local_addr().unwrap());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn recv_vehicle_state_packet_reads_empty_datagrams() {
        let socket = StdUdpSocket::bind("127.0.0.1:0").unwrap();
        let remote = socket.local_addr().unwrap();
        let sender = StdUdpSocket::bind("127.0.0.1:0").unwrap();
        sender.send_to(&[], remote).unwrap();
        let mut frame = [0u8; 128];
        let mut control = [0u8; 128];

        let (size, source, _) = recv_vehicle_state_packet(&socket, &mut frame, &mut control)
            .expect("an empty datagram should still be received");

        assert_eq!(size, 0);
        assert_eq!(source, TelemetrySource::Umbilical);
    }
}

mod database_src {
    #![allow(dead_code)]

    use ntest::timeout;

    include!("../src/server/database.rs");

    #[test]
    #[timeout(5000)]
    fn volatile_opens_an_in_memory_database() {
        let database = Database::volatile().unwrap();

        let value: i32 = database
            .connection
            .blocking_lock()
            .query_row("SELECT 42", [], |row| row.get(0))
            .unwrap();
        assert_eq!(value, 42);
    }

    #[test]
    #[timeout(5000)]
    fn volatile_databases_are_independent() {
        let first = Database::volatile().unwrap();
        let second = Database::volatile().unwrap();

        first
            .connection
            .blocking_lock()
            .execute_batch("CREATE TABLE Marker (value INTEGER)")
            .unwrap();

        assert!(second
            .connection
            .blocking_lock()
            .prepare("SELECT value FROM Marker")
            .is_err());
    }

    #[test]
    #[timeout(5000)]
    fn open_creates_a_database_file() {
        let path =
            std::env::temp_dir().join(format!("servo-test-db-{}.sqlite", std::process::id()));
        let _ = std::fs::remove_file(&path);

        let database = Database::open(&path).unwrap();
        database
            .connection
            .blocking_lock()
            .execute_batch("CREATE TABLE Marker (value INTEGER)")
            .unwrap();

        assert!(path.exists());
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn open_reports_errors_for_unwritable_paths() {
        let path = std::env::temp_dir()
            .join(format!("servo-test-missing-dir-{}", std::process::id()))
            .join("database.sqlite");
        let _ = std::fs::remove_dir_all(path.parent().unwrap());

        let error = Database::open(&path).expect_err("a missing parent directory should fail");

        assert!(matches!(error, rusqlite::Error::SqliteFailure(_, _)));
    }
}

mod admin_src {
    #![allow(dead_code)]

    use ntest::timeout;

    include!("../src/server/routes/admin.rs");

    async fn shared_with_data() -> Shared {
        let shared = servo::server::Server::new(None).unwrap().shared;
        shared
            .database
            .connection
            .lock()
            .await
            .execute_batch(
                "CREATE TABLE Things (id INTEGER, name TEXT);
                 INSERT INTO Things VALUES (1, 'alpha'), (2, 'beta');",
            )
            .unwrap();
        shared
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn execute_sql_returns_column_names_and_rows() {
        let shared = shared_with_data().await;
        let request = ExecuteSqlRequest {
            raw_sql: String::from("SELECT id, name FROM Things ORDER BY id"),
        };

        let response = execute_sql(State(shared), Json(request)).await.unwrap();

        assert_eq!(response.column_names, ["id", "name"]);
        assert_eq!(
            response.rows,
            vec![
                vec![serde_json::json!(1), serde_json::json!("alpha")],
                vec![serde_json::json!(2), serde_json::json!("beta")],
            ]
        );
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn execute_sql_reports_invalid_statements() {
        let shared = shared_with_data().await;
        let request = ExecuteSqlRequest {
            raw_sql: String::from("SELECT * FROM TableThatDoesNotExist"),
        };

        assert!(execute_sql(State(shared), Json(request)).await.is_err());
    }
}

mod data_src {
    #![allow(dead_code)]

    include!("../src/server/routes/data.rs");

    use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

    use common::comm::{sam::Unit, Measurement};
    use ntest::timeout;

    static HDF5_FILE_INDEX: AtomicUsize = AtomicUsize::new(0);

    fn hdf5_path() -> std::path::PathBuf {
        let index = HDF5_FILE_INDEX.fetch_add(1, AtomicOrdering::Relaxed);
        std::env::temp_dir().join(format!(
            "servo-test-export-{}-{index}.hdf5",
            std::process::id()
        ))
    }

    #[test]
    fn telemetry_source_defaults_to_umbilical() {
        let query = TelemetrySourceQuery::default();

        assert_eq!(
            query.telemetry_source().unwrap(),
            TelemetrySource::Umbilical
        );
    }

    #[test]
    fn telemetry_source_accepts_radio_aliases_and_rejects_unknown_values() {
        for value in ["radio", "tel"] {
            let query = TelemetrySourceQuery {
                source: Some(value.to_owned()),
            };
            assert_eq!(query.telemetry_source().unwrap(), TelemetrySource::Radio);
        }

        let query = TelemetrySourceQuery {
            source: Some(String::from("ethernet")),
        };
        assert!(query.telemetry_source().is_err());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn telemetry_source_stats_are_empty_without_updates() {
        let telemetry = LiveTelemetry::default();
        let stats = telemetry_source_stats(&telemetry).await;

        assert!(stats.time_since_update_ms.is_none());
        assert!(stats.update_rate_hz.is_none());
        assert!(stats.packet_size_bytes.is_none());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn telemetry_source_stats_reflect_received_packets() {
        let telemetry = LiveTelemetry::default();
        servo::server::telemetry::update_live_telemetry(&telemetry, VehicleState::new(), 256).await;
        tokio::time::sleep(Duration::from_millis(5)).await;

        let stats = telemetry_source_stats(&telemetry).await;

        assert_eq!(stats.packet_size_bytes, Some(256));
        assert!(stats.time_since_update_ms.is_some());
        assert!(stats.update_rate_hz.is_some());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn telemetry_stats_reports_both_sources() {
        let shared = servo::server::Server::new(None).unwrap().shared;
        servo::server::telemetry::update_live_telemetry(
            &shared.telemetry.radio,
            VehicleState::new(),
            64,
        )
        .await;

        let response = telemetry_stats(State(shared)).await;
        let value = serde_json::to_value(&*response).unwrap();

        assert_eq!(value["tel"]["packet_size_bytes"], serde_json::json!(64));
        assert!(value["umbilical"]["packet_size_bytes"].is_null());
    }

    #[tokio::test]
    #[timeout(5000)]
    async fn telemetry_stats_is_all_none_on_a_fresh_server() {
        let shared = servo::server::Server::new(None).unwrap().shared;

        let response = telemetry_stats(State(shared)).await;
        let value = serde_json::to_value(&*response).unwrap();

        assert!(value["tel"]["packet_size_bytes"].is_null());
        assert!(value["umbilical"]["packet_size_bytes"].is_null());
    }

    #[test]
    fn make_hdf5_file_writes_the_requested_datasets() {
        let path = hdf5_path();
        let sensor_names = vec![String::from("PT01")];
        let valve_names = vec![String::from("VLV01")];
        let mut state = VehicleState::new();
        state.sensor_readings.insert(
            String::from("PT01"),
            Measurement {
                value: 12.5,
                unit: Unit::Psi,
            },
        );

        make_hdf5_file(&sensor_names, &valve_names, &[(1.0, state)], &path).unwrap();

        let file = hdf5::File::open(&path).unwrap();
        let timestamps = file
            .group("metadata")
            .unwrap()
            .dataset("timestamps")
            .unwrap()
            .read_raw::<f64>()
            .unwrap();
        let readings = file
            .group("sensors")
            .unwrap()
            .group("PT01")
            .unwrap()
            .dataset("readings")
            .unwrap()
            .read_raw::<f64>()
            .unwrap();
        drop(file);
        std::fs::remove_file(&path).unwrap();

        assert_eq!(timestamps, vec![1.0]);
        assert_eq!(readings, vec![12.5]);
    }

    #[test]
    #[timeout(5000)]
    fn make_hdf5_file_uses_a_sentinel_for_missing_sensor_readings() {
        let path = hdf5_path();
        let sensor_names = vec![String::from("MISSING")];
        let valve_names = Vec::<String>::new();

        make_hdf5_file(
            &sensor_names,
            &valve_names,
            &[(2.0, VehicleState::new())],
            &path,
        )
        .unwrap();

        let file = hdf5::File::open(&path).unwrap();
        let readings = file
            .group("sensors")
            .unwrap()
            .group("MISSING")
            .unwrap()
            .dataset("readings")
            .unwrap()
            .read_raw::<f64>()
            .unwrap();
        let units = file
            .group("sensors")
            .unwrap()
            .group("MISSING")
            .unwrap()
            .dataset("units")
            .unwrap()
            .read_raw::<i8>()
            .unwrap();
        drop(file);
        std::fs::remove_file(&path).unwrap();

        assert_eq!(readings, vec![-6942069420.0]);
        assert_eq!(units, vec![-69]);
    }
}

mod mappings_src {
    #![allow(dead_code)]

    include!("../src/server/routes/mappings.rs");

    use axum::http::StatusCode;
    use common::comm::{Computer, SensorType};

    fn mapping(text_id: &str) -> NodeMapping {
        NodeMapping {
            text_id: text_id.to_owned(),
            board_id: String::from("sam-21"),
            sensor_type: SensorType::Pt,
            channel: 1,
            computer: Computer::Flight,
            max: None,
            min: None,
            calibrated_offset: 0.0,
            powered_threshold: None,
            normally_closed: None,
        }
    }

    #[test]
    fn is_python_keyword_recognizes_builtins() {
        assert!(is_python_keyword("len"));
        assert!(is_python_keyword("print"));
        assert!(is_python_keyword("zip"));
    }

    #[test]
    fn is_python_keyword_ignores_case_and_suffixes() {
        assert!(!is_python_keyword("LEn"));
        assert!(!is_python_keyword("len_1"));
        assert!(!is_python_keyword("my_len"));
    }

    #[test]
    fn validate_mapping_identifier_accepts_python_identifiers() {
        for name in ["PT01", "_private", "go_2_da_store", "input_V"] {
            assert!(
                validate_mapping_identifier(&mapping(name)).is_ok(),
                "{name} should be accepted"
            );
        }
    }

    #[test]
    fn validate_mapping_identifier_rejects_invalid_names() {
        for name in ["", "1PT", "etc-1", "has space", "len"] {
            let result = validate_mapping_identifier(&mapping(name));
            assert!(result.is_err(), "{name} should be rejected");
            assert!(matches!(
                result.unwrap_err(),
                crate::server::error::ServerError::Raw(_, StatusCode::BAD_REQUEST)
            ));
        }
    }

    #[test]
    fn validate_mappings_accepts_unique_valid_names() {
        let mappings = vec![mapping("PT01"), mapping("PT02"), mapping("_VALVE")];

        assert!(validate_mappings(&mappings).is_ok());
    }

    #[test]
    fn validate_mappings_rejects_duplicate_names() {
        let mappings = vec![mapping("PT01"), mapping("PT01")];

        let error = validate_mappings(&mappings).unwrap_err();

        assert!(matches!(
            error,
            crate::server::error::ServerError::Raw(_, StatusCode::BAD_REQUEST)
        ));
    }
}

#[test]
#[timeout(5000)]
fn server_new_without_a_path_creates_empty_shared_state() {
    let server = servo::server::Server::new(None).unwrap();

    assert!(server.shared.flight.0.blocking_lock().is_none());
    assert!(server.shared.ground.0.blocking_lock().is_none());
    assert!(server
        .shared
        .database
        .connection
        .blocking_lock()
        .execute_batch("SELECT 1")
        .is_ok());
}

#[test]
#[timeout(5000)]
fn server_new_with_a_path_opens_that_database() {
    let path =
        std::env::temp_dir().join(format!("servo-test-server-{}.sqlite", std::process::id()));
    let _ = std::fs::remove_file(&path);

    let server = servo::server::Server::new(Some(&path)).unwrap();
    server
        .shared
        .database
        .connection
        .blocking_lock()
        .execute_batch("CREATE TABLE Marker (value INTEGER)")
        .unwrap();

    assert!(path.exists());
    std::fs::remove_file(&path).unwrap();
}

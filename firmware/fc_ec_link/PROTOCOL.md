# CTV FC <-> EC link protocol (version 4)

The flight computer (FC) and the engine controller (EC) talk over UDP on the
vehicle Ethernet. This document is the byte-level reference for that link, plus
the behaviour each side expects of the other. It describes the code as it is.
If the code and this document disagree, the code wins, and this document needs
fixing:

| Side | Wire format | Link logic |
|---|---|---|
| FC (Rust) | `common/src/comm/ec_link.rs` | `flight2/src/ec_link.rs` |
| EC (C99) | `firmware/fc_ec_link/fc_ec_link.h/.c` | `firmware/fc_ec_link/ec_side.h/.c` |

Every offset in the byte tables below came from the real C encoder
(`fcec_encode`): each field was set on its own and the changed bytes were
recorded. The test vector in section 10 was produced the same way, and it
matches the Rust test `demand_matches_spec_bytes`.

Contents:

1. Transport
2. Encoding rules and CRC
3. Header (48 bytes)
4. Messages FC -> EC
5. Messages EC -> FC
6. Enums and bit masks
7. EC state machine
8. Validation order on the EC
9. Timing
10. Test vector
11. EC integration guide
12. Not implemented yet

## 1. Transport

| Item | Value |
|---|---|
| Protocol | UDP over IPv4, one message per datagram |
| EC listens on | UDP 8378 (`FCEC_EC_PORT`) |
| FC sends from and listens on | UDP 4574 (`FCEC_FC_PORT`) |
| EC replies to | the source address and port of the FC packet (so 4574) |
| EC IP in code | 192.168.1.50 (`FCEC_EC_IP_*`, `ec_link::EC_ADDRESS`), a placeholder |
| EC IP proposed | 192.168.1.138, hostname `ctv-ec`, on the avionics network allocation page. Not final until the EC is flashed with it. The bench package already sets `FC_EC_ADDRESS=192.168.1.138:8378`; the shared constants stay at .50 for now. |
| Max datagram | 512 bytes (`FCEC_MAX_DATAGRAM_LEN`). The largest message (Status) is 440. |
| Min datagram | 52 bytes (header + CRC, no payload) |

No fragmentation, no TCP and no broadcast. Each side keeps the newest data only,
so a lost datagram is never resent as is (transactions are retried, see 9).

## 2. Encoding rules and CRC

- **Layout:** `header (48) | payload (fixed length per kind) | CRC-32 (4)`.
- **Endianness:** all integers and floats are little-endian.
- **No padding:** fields sit back to back. Each one is written and read
  separately, never by copying a C struct or a Rust type in memory. Fields
  after Status offset 261 are not 2- or 4-byte aligned, so read them byte by
  byte (the reader in `fc_ec_link.c` does).
- **Floats:** IEEE 754 binary32 (`f32` / `float`).
- **bool:** a single bit in a flags byte, never a whole byte (see the header
  `flags` and the Status `flags`).
- **Reserved bytes:** are sent as 0 and ignored when received.
- **Enums:** the decoder rejects values with no meaning (`FCEC_ERR_INVALID_ENUM`).
- **CRC:** CRC-32/ISO-HDLC, the zlib/Ethernet CRC-32 (reflected polynomial
  0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF). It covers the header and
  the payload, i.e. every byte before the CRC, and is stored little-endian as
  the last 4 bytes. Check value: `crc32("123456789") = 0xCBF43926`.

### STM32 hardware CRC

The STM32H7 CRC unit computes the same CRC with these settings (HAL names):

| Setting | Value |
|---|---|
| `DefaultPolynomialUse` | `DEFAULT_POLYNOMIAL_ENABLE` (0x04C11DB7) |
| `DefaultInitValueUse` | `DEFAULT_INIT_VALUE_ENABLE` (0xFFFFFFFF) |
| `InputDataInversionMode` | `CRC_INPUTDATA_INVERSION_BYTE` |
| `OutputDataInversionMode` | `CRC_OUTPUTDATA_INVERSION_ENABLE` |
| `InputDataFormat` | `CRC_INPUTDATA_FORMAT_BYTES` |
| After the unit | XOR the result with 0xFFFFFFFF |

With no inversion and no final XOR the unit computes CRC-32/MPEG-2 instead.
Every packet would then fail the CRC check without an error you can see, so at
boot check the unit against `"123456789" -> 0xCBF43926`. Section 11 shows how
to plug it in.

## 3. Header (48 bytes, every message)

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u8[2] | magic | `"EC"` = `0x45 0x43` |
| 2 | u8 | version | `4` (`FCEC_VERSION`). Anything else is dropped. |
| 3 | u8 | kind | message kind (6.1) |
| 4 | u16 | payload_len | payload bytes after the header; must equal the kind's fixed length |
| 6 | u8 | source | `1` = FC, `2` = EC. Must match the kind: kinds < 0x80 come from the FC, the rest from the EC. |
| 7 | u8 | flags | bit 0 = `duplicate` (on a TxnResult: this repeats an earlier answer). Bits 1-7 reserved, 0. |
| 8 | u32 | ec_boot_id | random per EC boot, nonzero. The EC sends its own; the FC copies the newest one it has seen (0 before the first HelloResult). |
| 12 | u32 | session_id | chosen by the FC in Hello, nonzero. 0 = no session. |
| 16 | u32 | seq | per sender, +1 for every message sent in the session. Wraps. A Demand's command ID is its `seq`. |
| 20 | u32 | txn_id | transactions (Hello, Arm, Disarm, Terminate, ClearSafing) and their results; otherwise 0 |
| 24 | u32 | bundle_tag | first 4 bytes of the config bundle SHA-256, for logs only. Sent as 0 today. |
| 28 | u16 | valid_for_ms | FC -> EC: the message expires `valid_for_ms` after `ec_time_ref_us`. The FC sends 50. The EC sends 0. |
| 30 | u8[2] | reserved | 0 |
| 32 | u64 | sent_time_us | sender's monotonic clock, in µs |
| 40 | u64 | ec_time_ref_us | FC -> EC: `sent_time_us` of the newest EC packet the FC has received. The EC sends 0. |

Header fixed bytes for an FC Heartbeat: `45 43 04 11 00 00 01 00`.

## 4. Messages FC -> EC

| Kind | Name | Payload | Total | EC answers with |
|---|---|---:|---:|---|
| 0x01 | Hello | 48 | 100 | HelloResult |
| 0x02 | Arm | 36 | 88 | TxnResult |
| 0x03 | Disarm | 0 | 52 | TxnResult |
| 0x04 | Terminate | 4 | 56 | TxnResult |
| 0x05 | ClearSafing | 0 | 52 | TxnResult |
| 0x10 | Demand | 32 | 84 | nothing (the outcome shows up in Status) |
| 0x11 | Heartbeat | 0 | 52 | nothing |

Payload offsets below count from the start of the payload. Add 48 for the
offset in the datagram.

### Hello (0x01), 48 bytes

Opens a session. The FC repeats it every 100 ms, with a new `txn_id` each time,
until a HelloResult accepts it.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u32 | fc_boot_id | random per FC boot |
| 4 | u32 | run_id | run identifier (a placeholder until the FC has a run manager) |
| 8 | u16 | demand_period_ms | how often the FC means to send demands. The EC uses it for the demand timeout. 0 = default 10. |
| 10 | u16 | heartbeat_period_ms | how often the FC sends heartbeats (50) |
| 12 | u8[4] | reserved | 0 |
| 16 | u8[32] | expected_bundle_sha256 | SHA-256 of the config bundle the FC expects; all zeros = none. Not checked yet. |

### Arm (0x02), 36 bytes

DISARMED -> ARMED. Must be fresh (section 8).

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u32 | run_id | same as in Hello |
| 4 | u8[32] | expected_bundle_sha256 | all zeros = none. Not checked yet. |

### Disarm (0x03), ClearSafing (0x05), Heartbeat (0x11): no payload

- Disarm: ARMED -> DISARMED. It doesn't have to be fresh, since it only takes
  authority away.
- ClearSafing: SAFING -> DISARMED. Must be fresh.
- Heartbeat: keeps the link timer alive and does nothing else.

### Terminate (0x04), 4 bytes

Safes the EC and latches it. The EC obeys any well-formed Terminate, even
without a session or with the wrong boot ID.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u16 | reason | why the FC terminated (6.3). The FC sends 17 (`FC_TERMINATE`). The EC records `FC_TERMINATE` as its safing reason whatever this says. |
| 2 | u8[2] | reserved | 0 |

### Demand (0x10), 32 bytes

The newest actuator command. Its command ID is the header `seq`. Must be fresh.

| Offset | Type | Field | Units / range |
|---:|---|---|---|
| 0 | f32 | tvc_extension_mm[0] | TVC A1 rod extension, mm, 0 to 26.9 (full stroke of the 1.06" PA-12-T) |
| 4 | f32 | tvc_extension_mm[1] | TVC A2 rod extension, mm, 0 to 26.9 |
| 8 | f32 | rcs_duty[0] | RCS fan M1 duty, 0 to 1 (duty, not speed) |
| 12 | f32 | rcs_duty[1] | RCS fan M2 duty, 0 to 1 |
| 16 | f32 | rcs_duty[2] | RCS fan M3 duty, 0 to 1 |
| 20 | f32 | rcs_duty[3] | RCS fan M4 duty, 0 to 1 |
| 24 | f32 | main_throttle | main ESC throttle, 0 to 1, one signal shared by both ESCs. 0 = minimum-throttle pulse. |
| 28 | u16 | enable_mask | channels this demand drives (6.6) |
| 30 | u8[2] | reserved | 0 |

Every value must be finite and in range, or the whole demand is rejected.
Channels missing from `enable_mask` don't follow the demand: fans and the ESC go
to 0, and the TVC holds where it is.

## 5. Messages EC -> FC

| Kind | Name | Payload | Total | When |
|---|---|---:|---:|---|
| 0x80 | TxnResult | 8 | 60 | answer to Arm, Disarm, Terminate, ClearSafing |
| 0x81 | HelloResult | 52 | 104 | answer to Hello |
| 0x90 | Status | 388 | 440 | every 10 ms, to the FC's address and port |

The EC never sends `valid_for_ms` or `ec_time_ref_us` (both 0). In answers,
`txn_id` copies the request's. In Status it is 0. `session_id` is the
current session, or 0 if there isn't one.

### TxnResult (0x80), 8 bytes

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u8 | request_kind | kind being answered (0x02, 0x03, 0x04 or 0x05) |
| 1 | u8 | outcome | 0 accepted, 1 rejected |
| 2 | u16 | reason | 0 when accepted, else why (6.3) |
| 4 | u8 | ec_state | EC state after handling it (6.2) |
| 5 | u8[3] | reserved | 0 |

A repeated `txn_id` gets the stored answer again with header flag `duplicate`
set, and changes nothing.

### HelloResult (0x81), 52 bytes

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u8 | outcome | 0 accepted, 1 rejected |
| 1 | u8 | ec_state | 6.2 |
| 2 | u16 | reason | 6.3 |
| 4 | u16 | capabilities | channels the EC can drive (6.6). Default 0x7F. |
| 6 | u16 | max_demand_age_ms | freshness limit (50) |
| 8 | u32 | firmware_version | set by the firmware |
| 12 | u32 | calibration_id | set by the firmware |
| 16 | u16 | link_timeout_ms | link timeout (350) |
| 18 | u8[2] | reserved | 0 |
| 20 | u8[32] | active_bundle_sha256 | all zeros: no config on the EC yet |

The FC learns `ec_boot_id` from this message's header.

### Status (0x90), 388 bytes

Sent every 10 ms. The first 92 bytes are link state; the rest are board
measurements.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u8 | ec_state | 6.2 |
| 1 | u8 | flags | bit 0 `output_authority` (ARMED), bit 1 `session_valid`, bit 2 `safing_latched` (SAFING), bit 3 `demand_fresh` (a demand was accepted within 3 demand periods). Bits 4-7 reserved. |
| 2 | u16 | safing_reason | why the EC safed (6.3); 0 when not safing |
| 4 | u32 | last_received_seq | newest `seq` from a valid FC message in this session |
| 8 | u32 | last_accepted_demand_seq | newest demand that passed validation (stored even while disarmed) |
| 12 | u32 | last_applied_demand_seq | newest demand written to the outputs. "Applied" means commanded, not reached. |
| 16 | u64 | last_applied_time_us | EC time of that write |
| 24 | u16 | last_reject_reason | reason of the most recent rejection (6.3) |
| 26 | u16 | limited_mask | channels clamped by EC limits in the last applied demand. Always 0 for now. |
| 28 | u16 | crc_errors | dropped before decoding: datagram too short or too long, bad magic, version or CRC |
| 30 | u16 | rejected | everything else that was refused: unknown kind, wrong payload length or source, a bad enum, an EC -> FC kind, and the session/state/range checks |
| 32 | u16 | stale | rejected as too old (also counted in `rejected`) |
| 34 | u16 | old_or_duplicate | demands no newer than the last accepted one |
| 36 | u16 | txn_queue_overflow | reserved, 0 (no transaction queue yet) |
| 38 | u16 | diag_overflow | reserved, 0 |
| 40 | u16 | deadline_miss | the firmware bumps this when it misses its Status or loop deadline |
| 42 | u16 | motion_coalesced | firmware PA-12 scheduler counter |
| 44 | u16 | motion_rejected | firmware PA-12 scheduler counter |
| 46 | u8 | reset_cause | MCU reset cause, set by the firmware |
| 47 | u8 | reserved | 0 |
| 48 | u32 | firmware_version | |
| 52 | u32 | active_config_id | the EC's `calibration_id` |
| 56 | u32 | driver_faults | driver fault bitmap, set by the firmware |
| 60 | u32 | watchdog_status | watchdog bitmap, set by the firmware |
| 64 | f32 | applied.tvc_extension_mm[0] | what the outputs are set to now, after EC logic. Same units as Demand. |
| 68 | f32 | applied.tvc_extension_mm[1] | |
| 72 | f32 | applied.rcs_duty[0] | |
| 76 | f32 | applied.rcs_duty[1] | |
| 80 | f32 | applied.rcs_duty[2] | |
| 84 | f32 | applied.rcs_duty[3] | |
| 88 | f32 | applied.main_throttle | |
| 92 | u32 | sample_id | +1 per published measurement batch |
| 96 | u64 | sample_time_us | EC time of the batch |
| 104 | u32 | validity_mask | bit per value (table below); a value means nothing unless its bit is set |
| 108 | u64[17] | measurement_sample_time_us | per-value acquisition time, µs, in value order |
| 244 | u8[17] | measurement_quality | per-value quality (6.4), in value order |
| 261 | u16[13] | adc_raw_code | raw ADC3 codes in **board order** (below), not value order |
| 287 | f32[2] | tvc_position_mm | PA-12 A1, A2 measured position, mm |
| 295 | u16[2] | tvc_position_counts | the same positions as raw PA-12 counts |
| 299 | u16[2] | tvc_position_age_ms | age of each position reading, ms |
| 303 | u16[2] | pa12_fault_flags | PA-12 fault bits, as reported by the actuator |
| 307 | u32[2] | pa12_command_request_id | ID of the last goal sent to each PA-12 |
| 315 | u32[2] | pa12_feedback_request_id | ID of the last position readback from each PA-12 |
| 323 | u8[2] | pa12_command_state | 6.5 |
| 325 | u8[2] | pa12_feedback_state | 6.5 |
| 327 | f32[4] | rail_voltage_v | input, 12 V, 4.4 V, 3.3 V rails, V |
| 343 | f32[4] | rail_current_a | the same rails, A |
| 359 | f32[4] | rcs_current_a | RCS fans M1 to M4, A |
| 375 | f32 | motor_rail_voltage_v | P4V4-MOTOR rail, V |
| 379 | f32[2] | board_temp_c | TMP112 U3, U4, °C |
| 387 | u8 | temperature_status | `FCEC_TEMP_*` bits (6.7) |

Array elements are back to back. For example, `measurement_sample_time_us[i]`
starts at 108 + 8i, and `adc_raw_code[i]` at 261 + 2i.

**Value order** (bit `i` of `validity_mask`, and index `i` of
`measurement_sample_time_us` and `measurement_quality`):

| Index / bit | Value | Index / bit | Value |
|---:|---|---:|---|
| 0 | TVC position A1 | 9 | current, 3.3 V rail |
| 1 | TVC position A2 | 10 | RCS current M1 |
| 2 | voltage, input rail | 11 | RCS current M2 |
| 3 | voltage, 12 V rail | 12 | RCS current M3 |
| 4 | voltage, 4.4 V rail | 13 | RCS current M4 |
| 5 | voltage, 3.3 V rail | 14 | voltage, motor rail |
| 6 | current, input rail | 15 | temperature U3 |
| 7 | current, 12 V rail | 16 | temperature U4 |
| 8 | current, 4.4 V rail | | |

**Board order** of `adc_raw_code` (ADC3 scan order, `measurement.h`): vin, iin,
v12, i12, v4v4, i4v4, v3v3, i3v3, im1, im2, im3, im4, vmotor.

Signal paths and conversions are in `MEASUREMENTS.md`.

## 6. Enums and bit masks

### 6.1 Message kind (u8)

| Value | Kind | Direction |
|---:|---|---|
| 0x01 | Hello | FC -> EC |
| 0x02 | Arm | FC -> EC |
| 0x03 | Disarm | FC -> EC |
| 0x04 | Terminate | FC -> EC |
| 0x05 | ClearSafing | FC -> EC |
| 0x10 | Demand | FC -> EC |
| 0x11 | Heartbeat | FC -> EC |
| 0x80 | TxnResult | EC -> FC |
| 0x81 | HelloResult | EC -> FC |
| 0x90 | Status | EC -> FC |

### 6.2 EC state (u8)

| Value | State | Meaning |
|---:|---|---|
| 0 | BOOT_SAFE | just booted, drivers not up; no session, no output authority |
| 1 | DISARMED | outputs safe; demands are stored but not applied |
| 2 | ARMED | demands drive the outputs |
| 3 | SAFING | safe outputs, TVC centered; latched until ClearSafing |

### 6.3 Reason (u16)

| Value | Reason | Used by the EC today |
|---:|---|---|
| 0 | NONE | |
| 1 | BAD_SESSION | Hello with session 0; any other message outside the current session |
| 2 | BAD_BOOT_ID | message with the wrong `ec_boot_id` |
| 3 | WRONG_STATE | new-session Hello while ARMED; ClearSafing when not SAFING |
| 4 | CONFIG_MISMATCH | reserved (bundle check not implemented) |
| 5 | NOT_READY | Hello or Arm in BOOT_SAFE |
| 6 | SAFING_LATCHED | Arm in SAFING |
| 7 | UNSUPPORTED_VERSION | reserved (a wrong version is dropped without an answer) |
| 8 | UNSUPPORTED_CHANNEL | demand enables a channel not in `capabilities` |
| 9 | BAD_LENGTH | reserved (a bad length is dropped without an answer) |
| 10 | OUT_OF_RANGE | demand value outside its range |
| 11 | NON_FINITE | demand value NaN or infinite |
| 12 | STALE | failed freshness, or a `txn_id` older than the newest seen |
| 13 | BUSY | reserved (no transaction queue yet) |
| 14 | DEMAND_TIMEOUT | safing reason: no demand for 3 periods while ARMED |
| 15 | LINK_TIMEOUT | safing reason: no valid FC message for 350 ms |
| 16 | LOCAL_FAULT | reserved for faults the EC detects itself (no API yet, see 12) |
| 17 | FC_TERMINATE | safing reason: Terminate received |

### 6.4 Measurement quality (u8, per value)

| Value | Quality | | Value | Quality |
|---:|---|---|---:|---|
| 0 | OK | | 6 | UNSETTLED |
| 1 | RATIOMETRIC | | 7 | NOT_SAMPLED |
| 2 | STALE | | 8 | ADC_FAULT |
| 3 | SATURATED_HIGH | | 9 | UNCONFIGURED |
| 4 | AT_FLOOR | | 10 | DEVICE_FAULT |
| 5 | OUT_OF_RANGE | | 11 | TRANSPORT_FAULT |

Values above 11 make the whole Status fail to decode.

### 6.5 PA-12 states (u8, per actuator)

`pa12_command_state`: state of the last goal sent to the actuator.

| Value | State |
|---:|---|
| 0 | NONE |
| 1 | QUEUED |
| 2 | SENT_UNCONFIRMED: sent. The PA-12 doesn't ACK goals in its default mode, so this is not proof it was accepted. |
| 3 | TX_FAILED: UART transmit failed after retries |

`pa12_feedback_state`: result of the separate position readback.

| Value | State |
|---:|---|
| 0 | NONE |
| 1 | PENDING |
| 2 | VALID_REPLY |
| 3 | VALID_REPLY_WITH_FAULT (see `pa12_fault_flags`) |
| 4 | TIMEOUT |
| 5 | INVALID_REPLY |
| 6 | TRANSPORT_ERROR |

### 6.6 Channel bits (u16: `enable_mask`, `capabilities`, `limited_mask`)

| Bit | Mask | Channel |
|---:|---|---|
| 0 | 0x0001 | TVC A1 (PA-12) |
| 1 | 0x0002 | TVC A2 (PA-12) |
| 2 | 0x0004 | RCS fan M1 |
| 3 | 0x0008 | RCS fan M2 |
| 4 | 0x0010 | RCS fan M3 |
| 5 | 0x0020 | RCS fan M4 |
| 6 | 0x0040 | main ESC |
| | 0x007F | all |

### 6.7 `temperature_status` bits (`temperature.h`)

| Bit | Meaning |
|---:|---|
| 0 | U3 device failed |
| 1 | U4 device failed |
| 2 | I2C bus failed |
| 3 | ALERT pin active |
| 4 | U3 over temperature |
| 5 | U4 over temperature |
| 6 | ALERT pin read failed |

Whether each temperature is valid is in `validity_mask` bits 15-16, not here.

## 7. EC state machine

```
power-on --fcec_ec_init()--> BOOT_SAFE
BOOT_SAFE --fcec_ec_boot_done()--> DISARMED
DISARMED --Hello (new session)--> DISARMED + session
DISARMED + session --Arm (fresh)--> ARMED
ARMED --Disarm--> DISARMED
ARMED --demand timeout--> SAFING (DEMAND_TIMEOUT)
DISARMED/ARMED/SAFING + session --link timeout--> SAFING (LINK_TIMEOUT), session dropped
any --Terminate--> SAFING (FC_TERMINATE)
SAFING + session --ClearSafing (fresh)--> DISARMED
```

- **Boot:** a new random `ec_boot_id` on every boot, and the EC starts in
  BOOT_SAFE. It never restores ARMED. A reboot ends the session; the FC
  notices the new boot ID in Status and sends Hello again.
- **Hello:**
  - The same session again: accepted, nothing reset.
  - A new nonzero session: accepted in DISARMED or SAFING. Rejected
    WRONG_STATE while ARMED, NOT_READY in BOOT_SAFE.
  - Session 0: rejected BAD_SESSION.
  - Starting a session resets the EC's `seq`, the demand tracking and the
    transaction history.
  - Hello doesn't arm anything. The boot ID isn't checked on Hello, since the
    FC doesn't know it yet.
- **Arm:** acts only in DISARMED. If already ARMED it is accepted and changes
  nothing. In SAFING it is rejected SAFING_LATCHED. Starts the demand timer: a fresh demand must
  arrive within 3 demand periods. A demand stored while disarmed is **not**
  applied on arming; the next Demand is.
- **Disarm:** ARMED -> DISARMED. Fans and ESC go safe; the TVC holds where it
  is (it isn't centered). In any other state it is accepted and changes nothing.
- **SAFING:** latched. The first reason is kept. Only ClearSafing clears it,
  and that needs a valid session. After a link timeout the FC therefore has to
  send Hello first, then ClearSafing. Demands are still checked and stored in
  SAFING but never applied.
- **Terminate:** accepted from anyone, in any state, and answered with an
  accepted TxnResult. Terminate is not FC abort-to-land; abort is just the FC
  changing its Demands.

### Safe outputs

| Output | BOOT_SAFE, DISARMED | SAFING |
|---|---|---|
| RCS fans | duty 0 | duty 0 |
| Main ESC | throttle 0 = keep sending the **minimum-throttle pulse** (don't stop the signal) | same |
| PA-12 TVC | send no goals: the actuators hold their last goal | goal = center (`tvc_center_mm`, default 13.45 mm = half stroke) |

`fcec_ec_drive_tvc()` says whether to send PA-12 goals at all (only in ARMED and
SAFING). If the EC has lost serial contact with a PA-12 it can't center it. The
actuator holds its last goal, and the fault shows in `pa12_feedback_state` and
`pa12_fault_flags`.

## 8. Validation order on the EC

`fcec_ec_receive()` checks in this order and stops at the first failure.

1. **Decode** (`fcec_decode`):
   1. length ≥ 52 and ≤ 512
   2. magic `"EC"`
   3. version 4
   4. CRC
   5. known kind
   6. `payload_len` equal to the kind's length and to the bytes received
   7. `source` matching the kind
   8. every enum field valid

   Failures 1-4 bump `crc_errors`, the rest bump `rejected`. **No reply**,
   since the header can't be trusted, so a version mismatch looks like a
   silent EC.
2. **EC -> FC kind** received by the EC: `rejected`, no reply.
3. **Terminate:** obeyed now (it skips steps 4-6).
4. **Hello:** handled now (7, Hello).
5. **Boot ID**, then **session**: they must match, or the message is rejected
   BAD_BOOT_ID / BAD_SESSION.
6. **Transactions** (Arm, Disarm, ClearSafing):
   - A `txn_id` in the last-8 history gets the stored answer, flagged
     `duplicate`.
   - A `txn_id` not newer than the newest seen is rejected STALE.
7. **Freshness** (Demand, Arm, ClearSafing): rejected STALE unless all hold:
   - `ec_time_ref_us ≤ now`
   - `now − ec_time_ref_us ≤ max_demand_age_ms` (50 ms)
   - `now ≤ ec_time_ref_us + valid_for_ms`

   `now − ec_time_ref_us` bounds how old the FC's information is (it includes
   the round trip), without synced clocks.
8. The message counts as **valid**: the link timer resets and
   `last_received_seq` updates.
9. **Per kind:**
   - **Demand:**
     - Values must be finite and in range (NON_FINITE / OUT_OF_RANGE).
     - Channels must be supported (UNSUPPORTED_CHANNEL).
     - Then `seq` must be newer than the last accepted demand; otherwise
       `old_or_duplicate` goes up and the demand is dropped.
     - When accepted, it is stored, and applied only if ARMED.
   - **Transactions:** the state checks in 7, then the answer is remembered
     and sent.
   - **Heartbeat:** nothing more.

Rejected transactions and Hellos get a reply with the reason. Rejected Demands
and Heartbeats get none; look at `last_reject_reason` and the counters in
Status.

**Sequence wrap:** `seq` and `txn_id` are u32. "a is newer than b" means
`(a − b) mod 2³²` is between 1 and 2³¹ − 1 (`fcec_is_newer`).

## 9. Timing

| Item | Value | Where |
|---|---|---|
| FC demand period (declared in Hello) | 10 ms (100 Hz) | `demand_period_ms` |
| FC demand send rate | every 10 ms on this branch. The bench sim-plant branch (`dev/ctv/fc-sim-plant`) sends the same newest demand every 5 ms but still declares 10 ms, for margin against late wakeups. | `flight2/src/ec_link.rs` |
| FC stops sending demands | when the control side hasn't set one for 100 ms (`demand_hold`), so the EC times out instead of flying a frozen demand | FC |
| FC heartbeat | every 50 ms | FC |
| FC Hello retry | every 100 ms until accepted | FC |
| FC transaction retry | same `txn_id` every 50 ms, up to 5 tries, then "no answer" | FC |
| FC message lifetime | `valid_for_ms` = 50 | FC |
| EC Status | every 10 ms | firmware loop |
| EC max demand age | 50 ms | `FCEC_EC_MAX_DEMAND_AGE_MS` |
| EC demand timeout | 3 demand periods = 30 ms at 100 Hz, ARMED only. Reset only by an accepted demand (or by Arm). Heartbeats don't reset it. | `FCEC_EC_DEMAND_TIMEOUT_PERIODS` |
| EC link timeout | 350 ms with no valid FC message in the session -> session dropped + SAFING | `FCEC_EC_LINK_TIMEOUT_MS` |
| FC link timeout | 350 ms with nothing from the EC -> FC drops the session and sends Hello again | FC |
| EC transaction history | last 8 `txn_id`s | `FCEC_EC_TXN_HISTORY` |

The FC also drops the session and says Hello again when a Status shows a new
`ec_boot_id`, `session_valid` clear, or a different `session_id`.

**A normal run:**
1. Hello -> HelloResult (the FC learns the boot ID), and Status starts flowing.
2. Arm -> TxnResult accepted.
3. Demands every 10 ms, heartbeats every 50 ms.
4. Terminate -> TxnResult, and the EC is SAFING.
5. ClearSafing -> TxnResult, and the EC is DISARMED.

## 10. Test vector

A Demand produced by `fcec_encode` and checked against the Rust test
`demand_matches_spec_bytes` (`common/src/comm/ec_link.rs`). Both encoders give
these exact bytes, and `fcec_decode` accepts them.

Inputs:
- **Header:** `ec_boot_id` 0x11223344, `session_id` 0x0000BEEF, `seq` 2,
  `txn_id` 0, `bundle_tag` 7, `valid_for_ms` 50, `sent_time_us` 1 000 000,
  `ec_time_ref_us` 2 000 000.
- **Payload:** `tvc_extension_mm` {13.45, 13.45}, `rcs_duty`
  {0, 0.25, 0.5, 1.0}, `main_throttle` 0.6, `enable_mask` 0x7F.

84 bytes (header 0-47, payload 48-79, CRC 80-83):

```
45 43 04 10 20 00 01 00 44 33 22 11 EF BE 00 00
02 00 00 00 00 00 00 00 07 00 00 00 32 00 00 00
40 42 0F 00 00 00 00 00 80 84 1E 00 00 00 00 00
33 33 57 41 33 33 57 41 00 00 00 00 00 00 80 3E
00 00 00 3F 00 00 80 3F 9A 99 19 3F 7F 00 00 00
69 C3 C6 5D
```

CRC-32 over bytes 0-79 = **0x5DC6C369**, stored little-endian as `69 C3 C6 5D`.
The Rust tests also pin a Heartbeat (`heartbeat_matches_spec_bytes`).

## 11. EC integration guide

### What to compile

For the link, add **only `fc_ec_link.c` and `ec_side.c`** (plus their
headers) to the Cube project.
- They are plain C99, with no HAL, lwIP or malloc, and only `<string.h>` and
  `<math.h>` (`isfinite`).
- `fake_ec.c`, `test_*.c` and the Makefile are for laptops and are not needed
  on the EC.

`measurement.c`, `measurement_stm32h745.c` and `temperature.c` are separate
helpers for filling the Status measurements. Note that `measurement.c` and
`measurement_stm32h745.c` don't compile as committed (see 12).

### Main loop

```c
fcec_ec_t ec;
fcec_ec_init(&ec, boot_id);           // boot_id: random, nonzero, new every boot
ec.firmware_version = ...;            // optional settings, before the first packet
ec.calibration_id   = ...;
ec.tvc_center_mm[0] = ...;            // calibrated PA-12 centers, mm
ec.tvc_center_mm[1] = ...;
/* bring up drivers, Ethernet, PA-12 UARTs */
fcec_ec_boot_done(&ec);               // BOOT_SAFE -> DISARMED

for (;;) {
    uint64_t now = micros();          // 64-bit monotonic µs

    /* for each received UDP datagram (see the lwIP notes): */
    size_t n = fcec_ec_receive(&ec, rx, rx_len, now, reply, sizeof reply);
    if (n > 0) { /* send reply[0..n] to the datagram's source addr:port */ }

    fcec_ec_tick(&ec, now);           // link and demand timers; every loop

    const fcec_outputs_t *out = fcec_ec_outputs(&ec);
    /* RCS duty, ESC throttle (0 = minimum pulse) every loop;
       PA-12 goals only if fcec_ec_drive_tvc(&ec) */

    if (now >= next_status) {         // every 10 ms
        /* fill ec.measurements (and ec.reset_cause, driver_faults,
           watchdog_status, counters.deadline_miss as needed) */
        n = fcec_ec_status(&ec, now, tx, sizeof tx);
        /* send tx[0..n] to the FC's address:port (the last accepted sender) */
        next_status += 10000;
    }
}
```

Keep the loop well under the demand period: a demand only acts once
`fcec_ec_receive` has run, and the 30 ms demand timeout is checked in
`fcec_ec_tick`. Treat the state in `fcec_ec_t` as read-only. The firmware owns
only the settings, `measurements`, `reset_cause`, `driver_faults`,
`watchdog_status` and `counters.deadline_miss`.

### lwIP glue (raw UDP API, `NO_SYS = 1`)

- **Socket:** create one PCB with `udp_new()`, `udp_bind(pcb, IP_ADDR_ANY,
  8378)` and `udp_recv(pcb, on_rx, &ec)`. With `NO_SYS = 1` the receive
  callback runs in whatever context calls the netif input function. Poll
  `ethernetif_input` from the main loop, not from an interrupt, and the
  callback can call `fcec_ec_receive` directly.
- **Copy the datagram out:** the pbuf may be a chain. Use
  `pbuf_copy_partial(p, rx, sizeof rx, 0)` into a contiguous buffer of
  `FCEC_MAX_DATAGRAM_LEN + 1` bytes, and pass `p->tot_len` (capped at the
  buffer size) as the length, so an oversize datagram fails as too long
  instead of being cut down to a valid-looking one. Then `pbuf_free(p)`.
- **Reply to the sender:** pass the callback's `addr` and `port` to
  `udp_sendto`. The FC sends from 4574, so replies land there. Allocate the
  reply with `pbuf_alloc(PBUF_TRANSPORT, n, PBUF_RAM)`, copy the bytes in,
  send, then free it.
- **Status destination:** remember the address and port of the last FC
  datagram that `fcec_ec_receive` accepted (`ec.last_rx_us` changed or it
  returned a reply), the way `fake_ec.c` does. Send Status only once you have
  one.
- **IP:** static, from flash (CTV has no ACU). 192.168.1.138 is proposed; the
  code placeholder is 192.168.1.50.
- **STM32H7:** the Ethernet DMA descriptors and RX buffers need to sit in
  non-cacheable RAM (the usual D2 SRAM + MPU setup in ST's H7 lwIP examples),
  or received data can be stale.

### Clock

`now_us` must be a **64-bit monotonic microsecond clock** that never jumps.
For example, a 32-bit timer at 1 MHz extended with an overflow count, or the
DWT cycle counter scaled and extended. All EC times on the wire (`sent_time_us`,
`last_applied_time_us`, `sample_time_us`) come from it, and freshness compares
the FC's echo of them against it.

### Boot ID

Take it from the RNG peripheral (`HAL_RNG_GenerateRandomNumber`), and retry if
it returns 0. It must be different on every boot, or the FC can't tell the EC
rebooted.

### Hardware CRC (`FCEC_EXTERNAL_CRC32`)

Define `FCEC_EXTERNAL_CRC32` for `fc_ec_link.c` so the software CRC isn't built,
and provide:

```c
uint32_t fcec_crc32(const uint8_t *data, size_t len) {
    /* hcrc set up as in section 2 (BYTE input inversion, output inversion,
       default polynomial and init, byte input format) */
    return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, (uint32_t)len) ^ 0xFFFFFFFFu;
}
```

`HAL_CRC_Calculate` restarts from the init value on every call. At boot,
check `fcec_crc32("123456789", 9) == 0xCBF43926` and stay in BOOT_SAFE if it
fails. Don't call it from an interrupt while the main loop might be using the
CRC unit.

### References

- `fake_ec.c`: a complete EC loop on a POSIX socket. It shows reply-to-sender,
  Status every 10 ms to the last FC address, and `deadline_miss`.
- `test_ec_side.c`: every state transition, rejection and timeout as a test.
  Use it to check a port, or run it on the target.
- `test_fc_ec_link.c`: encode/decode, CRC and the byte layout.

On a laptop, `make test` builds and runs both test programs, and `make fake_ec`
builds an EC to run against flight2.

## 12. Not implemented yet

- **Config bundle check:** Hello/Arm carry `expected_bundle_sha256` but the EC
  doesn't compare it. `active_bundle_sha256` is all zeros, and
  CONFIG_MISMATCH is never sent.
- **Transaction queue and BUSY:** transactions are handled immediately, and
  `txn_queue_overflow` and `diag_overflow` stay 0.
- **EC limits tighter than the physical range:** none yet, so `limited_mask`
  is always 0.
- **LOCAL_FAULT safing:** `ec_side.h` has no call for the firmware to safe the
  EC on a fault it finds itself. One is needed before flight.
- **Receive budget:** nothing caps how many datagrams are handled per loop.
- **Run ID:** the FC sends a placeholder until it has a run manager.
- **Measurement helpers don't compile:**
  - `measurement.h` declares its own `fcec_measurement_quality_t`, which
    clashes with the one in `fc_ec_link.h`.
  - `measurement_stm32h745.c` defines `fcec_measurement_stm32h745_configure`
    with 3 parameters; its header declares 4.
  - The link itself (`fc_ec_link.c`, `ec_side.c`) isn't affected.
- **`MEASUREMENTS.md` is out of date:** it says the wire version is 3 and that
  Status has no per-value timestamps or quality. Version 4 has both (section 5).

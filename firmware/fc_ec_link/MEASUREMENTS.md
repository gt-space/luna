# EC measurement sensor inventory


This is the Step 1 inventory for the CTV Engine Controller schematic revision 1.0 (dated 2026-10-14). It records the board routing and nominal analog paths described by the supplied inventory guide, arranged for use by the EC measurement implementation.


**Evidence labels:** schematic-derived values are treated as nominal hardware facts from the guide. `†` marks values the guide says were recalled from STM32H745, TMP112, or INA180 datasheets that were not supplied; verify them against the exact fitted parts before release. `TBD` means the firmware or operating limits still need configuration. Sample rates below are implementation targets, not proof of the current firmware behavior. The ADC3 external channel mapping and PC2_C switch setting are now encoded in `measurement_stm32h745.c`; configured ADC timing and the board startup integration remain application responsibilities.


## Shared analog setup


- MCU: STM32H745XIH6. The 13 external analog measurements use ADC3; the guide's ADC channel IDs and alternate functions carry `†` and need checking against the MCU datasheet and firmware configuration.
- ADC resolution is up to 16 bits on this MCU family `†`, but configured bit depth, oversampling, trigger, DMA behavior, and sample time are TBD.
- VREF+ is VDDA, nominally P3V3 through FB1; VREF- is ground. This is not a precision independent reference. Read VREFINT during the scan and derive VDDA `†`; use that value for voltage conversion instead of assuming 3.3 V.
- Nominal conversion: `adc_pin_V = raw_code * measured_VDDA / 2^N` (confirm the driver convention and endpoint scaling). Apply per-channel calibration as `corrected_V = scale_cal * (adc_pin_V - offset_cal)`. Voltage channels multiply corrected pin voltage by the divider factor; current channels divide corrected output voltage by their V/A sensitivity.
- PC2_C is routed to VSNS-MOTOR (ball R1); PC3_C (R2) is unconnected. Firmware must enable/check the STM32 SYSCFG analog switch for PC2_C.
- Acquisition policy: trigger the ADC3 regular sequence from a timer at 4 kHz (250 us between scans), use DMA, and publish every 10 ms. Use a 40-sample boxcar for continuously monitored rail currents; motor-current averaging is gated to stable enable-on windows as detailed below. The voltage paths call for at least 1 kHz acquisition; the 4 kHz common scan target satisfies that proposal, subject to validated ADC settling and CPU/DMA budget.
- `measurement_stm32h745.c` provides the routed GPIOF/GPIOH pad and ADC3 channel table, including PC2_C for VSNS-MOTOR and internal VREFINT, and applies it through caller-provided operations. It has no STM32 HAL header dependency. The target backend must implement those operations and own ADC clock, trigger, sampling-time mapping, DMA, and averaging. The host folder does not contain the EC startup/clock project.
- `measurement.c` converts a completed sweep using a calibrated VDDA reference and per-channel engineering-unit scale/offset, limits, floor/saturation thresholds, and monotonic per-channel acquisition timestamps. It marks ADC/reference errors, saturation, current floor, stale timestamps, unconfigured calibration, and engineering range violations invalid. Driver-supplied timestamps must already account for any sample-window/settling policy.
- Board startup should implement the callbacks in `fcec_stm32h745_ops_t` and call `fcec_measurement_stm32h745_configure()`. That function configures only the listed routed pads/channels, closes the PC2 analog switch, and includes VREFINT. Convert the VREFINT result to VDDA with the exact part's factory calibration code, code full-scale, and calibration voltage via `fcec_measurement_derive_vdda()`, then pass VDDA, raw codes, and acquisition times to `fcec_measurement_convert()`. The board callback implementation and ADC trigger/DMA wiring belong in the target startup project, which is not in this repository.
- Local conversion results retain per-channel timestamps and quality. The FC/EC status payload currently carries a batch `sample_time_us` and validity mask, not per-channel timestamps/quality. The wire version is 3; FC and EC firmware must use matching versions.


## Voltage channels


Full-scale and LSB estimates assume 3.3 V reference and 16-bit conversion; they are nominal calculations, not guaranteed accuracy. Divider resistor source impedances are shown to guide ADC sample-time selection.


| Signal / key | Pin, net, ADC3 input | Units | ADC resolution / reference | Divider and nominal range | Analog filter | Expected period | Calibration ID | Validity |
|---|---|---|---|---|---|---|---|---|
| EC input, P22V2 (`vin`) | PF3 / V-P22V2 / INP5 `†` | V | Configured N TBD; VREF+=VDDA, nominal P3V3; VREFINT-derived VDDA `†` | 100 kOhm / 10 kOhm, x11; ~36.3 V FS, ~0.55 mV/LSB estimate | 9.09 kOhm source with 100 nF; tau ~0.91 ms, fc ~175 Hz | Scan >=1 kHz (common 4 kHz proposal); publish 10 ms average | `EC-CAL` / `vin` | Conversion succeeded; not saturated; in configured range; fresh. SMAJ26A clamp begins near 26 V; flag/limit below clamp region |
| 12 V rail (`v12`) | PF5 / V-P12V0 / INP4 `†` | V | Same shared ADC setup | 100 kOhm / 10 kOhm, x11; ~36.3 V FS, ~0.55 mV/LSB estimate | 9.09 kOhm with 100 nF; tau ~0.91 ms, fc ~175 Hz | Same | `EC-CAL` / `v12` | Same; separate configured under/over-voltage flags. 12 V TVS is marked TODO in the schematic guide |
| 4.4 V rail (`v4v4`) | PF7 / P4V4 / INP3 `†` | V | Same shared ADC setup | 10 kOhm / 10 kOhm, x2; ~6.6 V FS, ~0.10 mV/LSB estimate | 5 kOhm with 100 nF; tau ~0.5 ms, fc ~318 Hz | Same | `EC-CAL` / `v4v4` | Same; SMAJ6.5A hard over-limit |
| 3.3 V rail (`v3v3`) | PF9 / P3V3 / INP2 `†` | V | Same shared ADC setup; ratiometric to its own reference | 10 kOhm / 10 kOhm, x2; ~6.6 V FS, ~0.10 mV/LSB estimate | 5 kOhm with 100 nF; tau ~0.5 ms, fc ~318 Hz | Same | `EC-CAL` / `v3v3` | Treat ADC-derived reading as `RATIOMETRIC` sanity check because VDDA and the sensed rail track. Use VREFINT-derived VDDA as the actual P3V3 estimate; SMAJ5.0A hard over-limit |
| Motor rail P4V4-MOTOR (`vmotor`) | PC2_C / VSNS-MOTOR / INP0 `†` | V | Same shared ADC setup; PC2_C analog switch required | 10 kOhm / 10 kOhm, 1%, x2; ~6.6 V FS, ~0.10 mV/LSB estimate | 5 kOhm with 100 nF; tau ~0.5 ms, fc ~318 Hz | Same | `EC-CAL` / `vmotor` | Same; account for droop behind FB201 during motor pulses. Apply undervoltage limit only after configured duration/settling policy |


## Current channels


All eight channels use INA180A3, nominal gain 100 V/V, unidirectional, powered from P3V3. Every output has 1 kOhm series resistance and 100 nF to ground: tau ~100 us and fc ~1.6 kHz. The 16-bit LSB and full-scale figures below assume the stated reference and are estimates.


| Signal / key | Pin, net, ADC3 input; enable | Units | ADC resolution / reference | Shunt / gain and nominal range | Output filter | Expected period | Calibration ID | Validity |
|---|---|---|---|---|---|---|---|---|
| EC input current (`iin`) | PF4 / I-P22V2 / INP9 `†` | A | Configured N TBD; VREF+=VDDA; derive VDDA via VREFINT `†` | 5 mOhm + INA180A3 x100 = 0.5 V/A; ~6.6 A FS, ~101 uA/LSB estimate | 1 kOhm + 100 nF | Scan >=4 kHz proposal; publish 10 ms average | `EC-CAL` / `iin` | Not saturated or at/below calibrated floor; in range; fresh. Check INA180 common-mode headroom near battery maximum (~25 V concern `†`). Measures board input only; ESC bypasses board |
| 12 V rail current (`i12`) | PF6 / I-P12V0 / INP8 `†` | A | Same shared ADC setup | 5 mOhm + x100 = 0.5 V/A; ~6.6 A FS, ~101 uA/LSB estimate | 1 kOhm + 100 nF | Same | `EC-CAL` / `i12` | Same. Sum is total load for both actuators; guide estimates ~30 mA idle and ~0.33 A stall per PA-12-T 12 V unit |
| 4.4 V rail current (`i4v4`) | PF8 / I-P4V4 / INP7 `†` | A | Same shared ADC setup | 10 mOhm, 1% + x100 = 1.0 V/A; ~3.3 A FS, ~50 uA/LSB estimate | 1 kOhm + 100 nF | Same | `EC-CAL` / `i4v4` | Same. Includes LDO input current; motor-rail current may be estimated as `i4v4 - i3v3` and must be labeled derived |
| 3.3 V rail current (`i3v3`) | PF10 / I-P3V3 / INP6 `†` | A | Same shared ADC setup | 25 mOhm, 1% + x100 = 2.5 V/A; ~1.32 A FS, ~20 uA/LSB estimate | 1 kOhm + 100 nF | Same | `EC-CAL` / `i3v3` | Same; set upper limit against validated LDO current limit `†` |
| Motor 1 current (`im1`) | PH2 / ISNS-M1 / INP13 `†`; M1-EN=PD3 | A | Same shared ADC setup | 50 mOhm, 1% + x100 = 5.0 V/A; ~0.66 A FS, ~10 uA/LSB estimate | 1 kOhm + 100 nF | Triggered; sample >=0.5 ms after EN rising edge and before falling edge | `EC-CAL` / `im1` | Valid only while EN is high and after settling. Outside window quality=`NOT_SAMPLED`, never 0 A. Turn-off decay bypasses shunt through D205 |
| Motor 2 current (`im2`) | PH3 / ISNS-M2 / INP14 `†`; M2-EN=PD2 | A | Same shared ADC setup | Same 50 mOhm / INA180A3 front end | 1 kOhm + 100 nF | Same trigger rule | `EC-CAL` / `im2` | Same as `im1` |
| Motor 3 current (`im3`) | PH4 / ISNS-M3 / INP15 `†`; M3-EN=PD1 | A | Same shared ADC setup | Same 50 mOhm / INA180A3 front end | 1 kOhm + 100 nF | Same trigger rule | `EC-CAL` / `im3` | Same as `im1` |
| Motor 4 current (`im4`) | PH5 / ISNS-M4 / INP16 `†`; M4-EN=PD0 | A | Same shared ADC setup | Same 50 mOhm / INA180A3 front end | 1 kOhm + 100 nF | Same trigger rule | `EC-CAL` / `im4` | Same as `im1` |


## Acquisition, triggering, and filtering policy

- Trigger ADC3's regular sequence from a hardware timer at 4 kHz. Keep the complete sequence, including VREFINT, within the 250 us scan period. Transfer samples with DMA and associate each channel with its conversion time from the timer/DMA schedule; do not stamp every channel with the later telemetry-publication time. Validate ADC sample time and scan duration on the configured ADC clock before relying on these rates.
- For `iin`, `i12`, `i4v4`, and `i3v3`, average the 40 acquired samples in each 10 ms telemetry interval with a boxcar (no additional software low-pass filter). This yields a 100 Hz output. The 40-sample boxcar group delay is 4.875 ms relative to the newest sample; publishing at the 10 ms bin edge adds up to one 250 us scan period. Report the midpoint of the sample timestamps as the measurement time and include actual scheduling/transmit delay separately (nominal sample-to-publication age is about 5 ms).
- Each INA180A3 output has a nominal 1 kOhm / 100 nF single-pole RC filter: `tau = 100 us`, `fc ~= 1.59 kHz`, 10-90% step response about 220 us, and 1% settling about 460 us. These calculations describe only the external RC. INA180 bandwidth, ADC input acquisition, layout parasitics, and the motor-current waveform also affect the complete response.
- For `im1` through `im4`, use the same 4 kHz timer scans, but accept a sample only while that motor's enable is high and at least 0.5 ms (five RC time constants) has elapsed since its most recent rising edge. Stop accepting samples at the falling edge. Average only accepted samples in each 10 ms telemetry interval; never include off-time samples as zero and never average across enable-off periods.
- Require at least four accepted samples per motor channel per 10 ms interval for a valid motor-current result. At 4 kHz this requires an enabled window of approximately 1.5 ms or longer. If fewer than four samples are available, mark the interval `NOT_SAMPLED`, leave the value invalid, and preserve the accepted-sample count for diagnostics. The reported motor-current sample time is the midpoint of the accepted sample timestamps; publish its age so the FC can account for the variable delay. For sparse pulses, that age can approach the full 10 ms telemetry interval.
- The 0.5 ms guard is a conservative external-RC settling rule, not a claim that the whole analog path settles in 0.5 ms. If the enable waveform is PWM, this policy measures on-state current only when pulses are long enough; characterize it with the actual PWM frequency/duty range. Shorter pulses are reported as unavailable rather than as zero or as a fabricated average.
- `vin`, `v12`, `v4v4`, `v3v3`, and `vmotor` may use the same 4 kHz scan and 10 ms boxcar for ordinary telemetry. Preserve the existing path-change settling validity rule for divider channels. Treat `v3v3` as `RATIOMETRIC` because it tracks the ADC reference.


## Digital feedback

`temperature.c` polls TMP112 U3 (`0x40`) and U4 (`0x41`) independently using an adapter-supplied bounded I2C transaction timeout. It samples shared active-low PH9 before I2C reads, reports each device's read failure separately from controller/timeout bus errors, reads each device's extended-mode bit before decoding its temperature, and compares each valid reading to a configured overtemperature limit. `fcec_temperature_publish()` writes the temperatures, bits 15/16 validity, and temperature status into an `fcec_measurements_t` snapshot. The target must provide the timeout-enforcing I2C/GPIO adapter and a configured thermal limit; no EC startup or I2C adapter for this STM32H745 board is present in this repository.


| Signal / key | Pin or bus | Units / resolution | Conversion / filter | Expected period | Calibration ID | Validity |
|---|---|---|---|---|---|---|
| TMP112 U3 (`t_u3`) | I2C3: PH7 SCL, PH8 SDA; address 0x40 (ADD0=GND); 4.7 kOhm pull-ups | deg C; signed 12-bit, 0.0625 deg C/LSB `†` (13-bit extended mode `†`) | Device conversion; no added software filter specified | Default conversion 4 Hz / 250 ms `†`; republish on 10 ms telemetry with original conversion timestamp | `EC-CAL` / `t_u3` offset | Bounded transaction ACK; config readback agrees; fresh conversion. Report overtemperature separately from bus/device failure |
| TMP112 U4 (`t_u4`) | Same bus; address 0x41 (ADD0=P3V3) | Same | Same | Same | `EC-CAL` / `t_u4` offset | Per-device checks, independent of U3 |
| Shared TMP112 ALERT | PH9; active-low open-drain; two 4.7 kOhm pull-ups in parallel | Boolean alert | Sample before I2C reads, since reads may clear interrupt-mode alert | Sample during each bounded poll | None | Active flag is separate and does not identify which sensor asserted it; per-device overtemperature flags use valid readings and configured limit |
| PA-12 actuator 1 (`pa12_1`) | UART7 TX=PA15, RX=PA8, DIR=PB8; SN74LVC2G241 U201; J401 pin 5 DATA-A1 | mm; actuator count and fault flags | Position register 0x8C, 16-bit little-endian (firmware assumption); nominal `mm = counts * 26.9 / 3686` for 1.06 in stroke. No software filter; retain raw counts | Plan ~20 ms per actuator; poll the two separate UARTs in parallel. Default 57600 8N1 and 5 ms after write / 10 ms after read are driver assumptions | `EC-CAL` / `pa12_1` plus stroke config | Validate header, size, checksum, matching ID and 2-byte data; inspect all seven reported errors; count within configured stroke; fresh. Report communication failure separately from actuator fault |
| PA-12 actuator 2 (`pa12_2`) | UART5 TX=PB6, RX=PB5, DIR=PB9; U202; J401 pin 4 DATA-A2 | Same | Same | Same, on independent UART | `EC-CAL` / `pa12_2` plus stroke config | Same per-device validation |

The `firmware/pa12` scheduler owns one transaction per independent UART7/UART5 port, uses nonblocking caller-driven TX/reply completion, and can start work on the other port while a reply is pending. It prioritizes the latest motion/safing goal ahead of diagnostics and configuration; unsent motion goals coalesce to the newest. It enforces the PA-12 quiet intervals (5 ms after writes, 10 ms after reads), permits one configuration operation per second per port, and retries failed transactions at most once after 100 ms. Read replies use a caller-configured finite timeout. Integrate it from the EC event loop using UART DMA/interrupt callbacks; do not wait synchronously for one port before servicing the other.


PA-12 count mapping is nominal only. The supplied guide gives 26.9 mm stroke and 3686 as the assumed long-stroke limit; confirm the installed actuator configuration and calibrate its position map. Manual-based performance figures in the guide are ~0.05 mm accuracy/backlash, 12 or 36 mm/s no-load speed, and ~0.75-2.2 s full-stroke time. Do not report finer effective accuracy just because the raw count step is smaller. The board routes TTL half-duplex only; PA-12-R requires a hardware interface change.

The PA-12 crate exposes `set_position_mm`, `read_position`, `read_faults`, and `apply_safing` (a calibrated park-position goal, which is ordinary motion). Motion APIs require a `PositionCalibration` built from two measured extension/count points plus the actuator's configured hard count limits. Non-finite and out-of-range extension commands return errors; converted counts are rounded and clamped to those hard count limits. No nominal calibration is silently substituted. `read_faults` requests the moving-status register, whose feedback packet also carries hardware error flags; decode those with `Feedback.error`.

The scheduler's `latch_safing(a1_force_off, a2_force_off)` is the authoritative safing path: it accepts only Force-OFF packets, discards unsent motion, queues Force-OFF at the highest priority on both ports, and rejects subsequent ordinary goals until an explicit `clear_safing_latch()`. Since a PA-12 goal write re-enables force, park goals must not be used as a substitute for that latch. ID/PID/calibration and other setup writes belong in `queue_setup`, which accepts register writes only while latched and spaces transactions by at least 10 seconds per port. Setup writes are not automatically retried because a lost UART completion is ambiguous for nonvolatile writes. `queue_motion` accepts only goal-position packets.

The version 3 status payload has per-actuator fields for `pa12_command_request_id`, command state, `pa12_feedback_request_id`, feedback state, measured position, fault flags, and position age. The EC runtime must populate them from scheduler transmissions/completions and parsed feedback. Request IDs are host-side per-port counters, not IDs echoed by the actuator. In the default PA-12 mode a goal write has no acknowledgement: command state `SENT_UNCONFIRMED` means only that UART transmission completed. A later valid readback proves the device replied and supplies measured position/fault flags; it does not prove the actuator accepted or reached the goal. Feedback status distinguishes pending, valid, valid-with-fault, timeout, invalid reply, and transport error. Position age is relative to the status batch time; clock mapping must be applied before combining it with FC timestamps.

No gimbal tracking residual is published or calculated here. The FC should derive one only after the position-to-angle kinematics, calibration identity, and timestamp/clock mapping are defined.


## Shared quality and calibration rules

Temperature status byte bits: bit 0 U3 read failed; bit 1 U4 read failed; bit 2 I2C controller/transaction failure or timeout; bit 3 shared ALERT pin active; bits 4 and 5 U3/U4 overtemperature; bit 6 ALERT GPIO read failure. A per-device read failure clears only that device's temperature validity bit. A bus error is reported independently, so a failed bus does not masquerade as a measured overtemperature. The shared alert line cannot identify which sensor asserted it; per-device threshold flags provide attribution when readings are valid. The recorded timestamp is the host read time for the TMP112's latest conversion, so its age is bounded by the configured conversion period, not the I2C transaction time.


Every measurement should carry engineering value, units, raw code/count when available, quality, sample time, sample ID, and age. Suggested quality values: `OK`, `STALE`, `SATURATED_HIGH`, `AT_FLOOR`, `OUT_OF_RANGE`, `UNSETTLED`, `NOT_SAMPLED`, `ADC_FAULT`, `UNCONFIGURED`, `RATIOMETRIC`.


- Never turn invalid, stale, saturated, unconfigured, or unsettled data into numeric zero. For unidirectional current sensing, a floor code means at/below measurable minimum, not necessarily 0 A; set the floor from verified INA180 behavior and zero-current calibration `†`.
- Mark stale when age exceeds 3x the channel's expected period (initial policy proposal). Use the underlying conversion/acquisition timestamp, not the 10 ms publication time, for slower TMP112 conversions.
- Mark saturation for near-full-scale codes, ADC overrun, or conversion failure. Apply engineering range and under/over-voltage thresholds separately from ADC limits.
- Mark unsettled during the voltage divider settling period after mux/path change (about 4.5 ms for 22 V/12 V, 2.5 ms for the 4.4 V/3.3 V/motor paths) and for at least 0.5 ms after motor enable switching.
- Nominal hardware ratios remain separate from calibration coefficients. Proposed ID format: `EC-CAL-R1.0-00`; set 00 means nominal scale=1, offset=0 and VREFINT-derived VDDA. Store scale, offset, units, and channel mapping per key. Any change to mapping or coefficients increments the set number. Confirm this matches the EC/FC protocol identity field.
- Resistor tolerance, amplifier offset/gain error, reference error, and ADC configuration affect accuracy. Uncalibrated values are nominal estimates only.


## Step 1 closeout: remaining confirmations


The inventory fields are populated from the supplied guide, including proposed periods and validity rules. Before using it as a release configuration, resolve these explicit open points:


1. Configured ADC resolution, sample time, oversampling, scan trigger, and DMA. Board code has the ADC3 channel allowlist and PC2_C SYSCFG switch setting; the startup project must call the setup and include VREFINT in its scan.
2. Factory VREFINT calibration word address/value and calibration voltage for the exact MCU revision, then pass its derived VDDA to conversion.
3. Final calibration coefficients/ID format, acceptable rail/current windows, stale thresholds, and motor enable pulse timing.
4. Exact installed TMP112 mode/rate/thresholds and verified INA180A3 limits, including input common-mode headroom and current floor.
5. PA-12 IDs, configured stroke limits, protocol assumptions, TTL levels at 3.3 V, and final per-actuator polling/position timeout.
6. Confirm protection details, especially the unfinished 12 V SMAJ15A TVS and the maximum battery voltage seen by `iin`.




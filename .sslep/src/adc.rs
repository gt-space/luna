
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use common::comm::{
    gpio::PinValue::Low,
    sslep::{
        SslepChannelType,
        SslepDataPoint,
        SslepSensorDataPoint,
    },
    ADCFamily,
    ADCKind,
    SSLEPADC,
};

// Match SAM's maximum DRDY wait: 1 ms per attempted reading.
// Tune this only after verifying first-conversion and Pi scheduling timing.
const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

const POWER_ADC_CHANNELS: u8 = 6;
const CURRENT_LOOP_ADC_CHANNELS: u8 = 3;

// The ADCs must be constructed in this order:
// 0 = Power ADC
// 1 = Current Loop ADC
fn validate_adcs(adcs: &[Box<dyn ADCFamily>]) {
    assert_eq!(adcs.len(), 2, "SSLEP expects two ADCs");

    assert!(
        matches!(
            adcs[0].kind(),
            ADCKind::Sslep(SSLEPADC::Power)
        ),
        "ADC 0 must be the Power ADC"
    );

    assert!(
        matches!(
            adcs[1].kind(),
            ADCKind::Sslep(SSLEPADC::CurrentLoop)
        ),
        "ADC 1 must be the Current Loop ADC"
    );
}

// Initialize both ADS124S06 ADCs.
pub fn init_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
    validate_adcs(adcs);

    for adc in adcs.iter_mut() {
        // Initially select AIN0 on each ADC.
        adc.set_positive_input_channel(0).unwrap();
        adc.set_negative_input_channel_to_aincom()
            .unwrap();

        // PGA configuration.
        adc.set_pga_gain(1).unwrap();
        adc.disable_pga().unwrap();

        // Conversion settings.
        adc.disable_global_chop().unwrap();
        adc.enable_internal_clock_disable_external()
            .unwrap();
        adc.enable_continious_conversion_mode()
            .unwrap();
        adc.enable_low_latency_filter().unwrap();
        adc.set_data_rate(1000.0).unwrap();

        // Internal 2.5V reference.
        adc.set_ref_input_internal_2v5_ref()
            .unwrap();
        adc.enable_internal_voltage_reference_on_pwr_down()
            .unwrap();

        // Disable unused features.
        adc.disable_idac1().unwrap();
        adc.disable_idac2().unwrap();
        adc.disable_vbias().unwrap();
        adc.disable_crc_byte().unwrap();
        adc.disable_status_byte().unwrap();

        println!("SSLEP ADC {:?} initialized", adc.kind());
    }
}

// Start continuous ADC conversion.
pub fn start_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
    validate_adcs(adcs);

    for (index, adc) in adcs.iter_mut().enumerate() {
        if let Err(e) = adc.spi_start_conversion() {
            eprintln!("ADC {index} failed to start: {e:?}");
        }
    }
}

// Poll DRDY as in SAM: read immediately on LOW, otherwise give up
// after 1 ms and skip the channel. The wait is not a fixed 1 ms delay.
fn wait_for_drdy(adc: &dyn ADCFamily) -> bool {
    let start = Instant::now();

    loop {
        match adc.check_drdy() {
            Some(Low) => return true,
            Some(_) if start.elapsed() >= ADC_DRDY_TIMEOUT => return false,
            Some(_) => {} // SAM busy-polls until ready or timed out.
            // Unlike SAM's 700 us fallback, don't read without a DRDY pin:
            // a read in that situation could be stale or invalid.
            None => return false,
        }
    }
}

// Read all ADC channels and generate telemetry.
pub fn poll_adcs(
    adcs: &mut [Box<dyn ADCFamily>],
) -> Vec<SslepDataPoint> {
    validate_adcs(adcs);

    let mut datapoints = Vec::with_capacity(9);

    // Like SAM: visit a channel number, then poll each ADC that has
    // that channel. SSLEP has 6 power channels and 3 current-loop channels.
    for channel in 0..POWER_ADC_CHANNELS {
        for (adc_index, adc) in adcs.iter_mut().enumerate() {
            let channel_count = if adc_index == 0 {
                POWER_ADC_CHANNELS
            } else {
                CURRENT_LOOP_ADC_CHANNELS
            };

            if channel >= channel_count {
                continue;
            }

            // Stop conversion before switching channels.
            if let Err(e) = adc.spi_stop_conversion() {
                eprintln!(
                    "ADC {adc_index} stop failed: {e:?}"
                );
                continue;
            }

            // Select the required input channel.
            if let Err(e) =
                adc.set_positive_input_channel(channel)
            {
                eprintln!(
                    "ADC {adc_index} AIN{channel} \
                     selection failed: {e:?}"
                );
                continue;
            }

            // Start a new conversion.
            if let Err(e) = adc.spi_start_conversion() {
                eprintln!(
                    "ADC {adc_index} AIN{channel} \
                     start failed: {e:?}"
                );
                continue;
            }

            // Wait for the ADC data-ready signal.
            if !wait_for_drdy(adc.as_ref()) {
                eprintln!(
                    "ADC {adc_index} AIN{channel} \
                     data not ready"
                );
                continue;
            }

            // Read digital ADC counts.
            let raw = match adc.read_counts() {
                Ok(value) => value,
                Err(e) => {
                    eprintln!(
                        "ADC {adc_index} AIN{channel} \
                         read failed: {e:?}"
                    );
                    continue;
                }
            };

            // Convert counts to ADC input voltage.
            let voltage = adc.calc_diff_measurement(raw);

            if !voltage.is_finite() {
                eprintln!(
                    "ADC {adc_index} AIN{channel}: \
                     invalid voltage"
                );
                continue;
            }

            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs_f64();

            // Ensure both ADCs have unique telemetry IDs.
            let telemetry_channel = if adc_index == 0 {
                channel as u32
            } else {
                6 + channel as u32
            };

            datapoints.push(
                SslepDataPoint::Sensor(
                    SslepSensorDataPoint {
                        value: voltage,
                        timestamp,
                        channel: telemetry_channel,
                        channel_type:
                            SslepChannelType::AdcVoltage,
                    }
                )
            );
        }
    }

    datapoints
}

// Stop both ADCs during shutdown or abort.
pub fn reset_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
    validate_adcs(adcs);

    for (index, adc) in adcs.iter_mut().enumerate() {
        if let Err(e) = adc.spi_stop_conversion() {
            eprintln!(
                "ADC {index} failed to stop: {e:?}"
            );
        }

        if let Err(e) = adc.set_positive_input_channel(0) {
            eprintln!(
                "ADC {index} failed to reset AIN0: {e:?}"
            );
        }
    }
}

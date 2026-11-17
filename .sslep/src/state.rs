
use std::{
    io::ErrorKind,
    net::{SocketAddr, UdpSocket},
    time::Instant,
};

use common::comm::{
    gpio::{GpioPin, PinMode},
    sslep::SslepControlMessage,
    ADCFamily,
    ADCKind,
    SSLEPADC,
};

// Verify this is the crate with the correct
// 24-bit ADS124S06 implementation.
use ads124s06::ADC;

use crate::{
    adc::{
        init_adcs,
        poll_adcs,
        reset_adcs,
        start_adcs,
    },
    command::{
        execute,
        init_gpio,
        safe_shutdown,
        CommandOutcome,
    },
    communication::{
        check_heartbeat,
        establish_flight_computer_connection,
        send_data,
    },
    pins::{
        GPIO_CONTROLLER,
        SPI_INFO,
        SpiDevice,
    },

    tmp112::Tmp112,
};

// Four possible SSLEP states.
pub enum State {
    Init,
    Connect(ConnectData),
    MainLoop(MainLoopData),
    Abort(AbortData),
}

#[derive(Clone, Copy)]
pub struct AbortInfo {
    pub received_abort: bool,
    pub last_heard_from_fc: Instant,
    pub time_aborted: Option<Instant>,
}

pub struct ConnectData {
    pub adcs: Vec<Box<dyn ADCFamily>>,
    pub abort_info: AbortInfo,
}

pub struct MainLoopData {
    adcs: Vec<Box<dyn ADCFamily>>,
    my_data_socket: UdpSocket,
    my_command_socket: UdpSocket,
    fc_address: SocketAddr,
    hostname: String,
    last_heard_from_fc: Instant,
}

pub struct AbortData {
    adcs: Vec<Box<dyn ADCFamily>>,
    pub abort_info: AbortInfo,
}

// Handle transitions between SSLEP states.
impl State {
    pub fn next(self) -> Self {
        match self {
            State::Init => init(),

            State::Connect(data) => connect(data),

            State::MainLoop(data) => main_loop(data),

            State::Abort(data) => abort(data),
        }
    }
}

// Initialize SSLEP hardware.
fn init() -> State {
    // Request inactive GPIO outputs.
    init_gpio().expect(
        "SSLEP GPIO initialization failed"
    );

    let mut adcs: Vec<Box<dyn ADCFamily>> =
        Vec::with_capacity(2);

    // SSLEP contains two ADS124S06 ADC chips.
    // Keep this order consistent with adc.rs.
    let devices = [
        (
            SpiDevice::PowerAdc,
            ADCKind::Sslep(SSLEPADC::Power),
        ),
        (
            SpiDevice::CurrentLoopAdc,
            ADCKind::Sslep(SSLEPADC::CurrentLoop),
        ),
    ];

    for (device, kind) in devices {
        let info = SPI_INFO
            .get(&device)
            .expect("Missing ADC SPI configuration");

        let bus = info
            .spi_device
            .expect("Missing SPI device path");

        let drdy = info
            .drdy
            .expect("Missing ADC DRDY pin mapping");

        // Create Raspberry Pi DRDY GPIO input.
        let mut drdy_pin =
            GPIO_CONTROLLER.get_pin(drdy.pin_num);

        drdy_pin.mode(PinMode::Input);

        let boxed_drdy: Box<dyn GpioPin> =
            Box::new(drdy_pin);

        // Create an ADC using the existing driver.
        // Linux SPI manages CE0/CE1 chip selects.
        let adc = ADC::new_with_gpio_pins(
            bus,
            Some(boxed_drdy),
            None,
            kind,
        )
        .expect("Failed to construct SSLEP ADC");

        adcs.push(Box::new(adc));
    }

    // Configure ADC registers.
    init_adcs(&mut adcs);

    State::Connect(ConnectData {
        adcs,
        abort_info: AbortInfo {
            received_abort: false,
            last_heard_from_fc: Instant::now(),
            time_aborted: None,
        },
    })
}

// Establish connection with the Flight Computer.
fn connect(mut data: ConnectData) -> State {
    // communication.rs must match this signature.
    let (
        data_socket,
        command_socket,
        fc_address,
        hostname,
        abort_info,
    ) = establish_flight_computer_connection(
        &mut data
    );

    data_socket
        .set_nonblocking(true)
        .expect("Failed to configure data socket");

    command_socket
        .set_nonblocking(true)
        .expect("Failed to configure command socket");

    // Start continuous ADC conversions.
    start_adcs(&mut data.adcs);

    State::MainLoop(MainLoopData {
        adcs: data.adcs,
        my_data_socket: data_socket,
        my_command_socket: command_socket,
        fc_address,
        hostname,
        last_heard_from_fc: abort_info.last_heard_from_fc,
    })
}

// Receive and execute up to 10 commands.
// Returns true if SSLEP must abort.
fn process_commands(
    command_socket: &UdpSocket,
    fc_address: &SocketAddr,
) -> bool {
    let mut buffer = [0u8; 1024];

    for _ in 0..10 {
        let (size, sender) =
            match command_socket.recv_from(&mut buffer) {
                Ok(packet) => packet,

                Err(err)
                    if err.kind() == ErrorKind::WouldBlock =>
                {
                    break;
                }

                Err(err) => {
                    eprintln!(
                        "SSLEP command socket error: {err}"
                    );
                    return true;
                }
            };

        // Preliminary sender check only.
        // IP matching is not authentication.
        if sender.ip() != fc_address.ip() {
            eprintln!(
                "Unexpected SSLEP command sender: {sender}"
            );
            continue;
        }

        // Deserialize the received command.
        let command =
            match postcard::from_bytes::<SslepControlMessage>(
                &buffer[..size]
            ) {
                Ok(command) => command,

                Err(err) => {
                    eprintln!(
                        "Invalid SSLEP command: {err}"
                    );
                    continue;
                }
            };

        // Execute the command directly.
        match execute(command) {
            Ok(CommandOutcome::Continue) => {}

            Ok(CommandOutcome::AbortRequested) => {
                return true;
            }

            Err(err) => {
                eprintln!(
                    "SSLEP command failed: {err:?}"
                );
                return true;
            }
        }
    }

    false
}

// Main operation loop.
fn main_loop(mut data: MainLoopData) -> State {
    // Check whether the Flight Computer is alive.
    // The heartbeat function must validate its sender.
    let (updated_time, should_abort) =
        check_heartbeat(
            &data.my_data_socket,
            &data.my_command_socket,
            data.last_heard_from_fc,
        );

    data.last_heard_from_fc = updated_time;

    // Abort if heartbeat timeout occurs.
    if should_abort {
        return enter_abort(data);
    }

    // Process commands received from FC.
    if process_commands(
        &data.my_command_socket,
        &data.fc_address,
    ) {
        return enter_abort(data);
    }

    // Read Power ADC and Current Loop ADC.
    let datapoints = poll_adcs(&mut data.adcs);

    // Send measurements to Flight Computer.
    send_data(
        &data.my_data_socket,
        &data.fc_address,
        data.hostname.clone(),
        datapoints,
    );

    // TODO: Add TMP112 board temperature data.
    // TODO: Add MAX6675 thermocouple data.

    State::MainLoop(data)
}

// Enter the abort state.
fn enter_abort(mut data: MainLoopData) -> State {
    // Request the assumed inactive GPIO levels.
    if let Err(err) = safe_shutdown() {
        eprintln!(
            "CRITICAL: SSLEP shutdown failed: {err:?}"
        );
    }

    // Stop ADC conversions.
    reset_adcs(&mut data.adcs);

    State::Abort(AbortData {
        adcs: data.adcs,
        abort_info: AbortInfo {
            received_abort: true,
            last_heard_from_fc:
                data.last_heard_from_fc,
            time_aborted: Some(Instant::now()),
        },
    })
}

// Remain in abort until an explicitly verified
// recovery procedure is implemented.
fn abort(data: AbortData) -> State {
    State::Abort(data)
}

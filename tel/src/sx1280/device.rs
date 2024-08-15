// Represents commands that can be sent to the SX1280
enum Command {
    GetStatus,
    WriteRegister,
    ReadRegister,
    WriteBuffer,
    ReadBuffer,
    SetSleep,
    SetStandby,
    SetFs,
    SetTx,
    SetRx,
    SetRxDutyCycle,
    SetCAD,
    SetTxContinuousWave,
    SetTxContinuousPreamble,
    SetPacketType,
    GetPacketType,
    SetRfFrequency,
    SetTxParams,
    SetCadParams,
    SetBufferBaseAddress,
    SetModulationParams,
    SetPacketParams,
    GetRxBufferStatus,
    GetPacketStatus,
    GetRssilnst,
    SetDioIrqParams,
    GetIrqStatus,
    ClearIrqStatus,
    SetRegulatorMode,
    SetSaveContext,
    SetAutoFS,
    SetAutoTx,
    SetLongPreamble,
    SetUartSpeed,
    SetRangingRole,
    SetAdvancedRanging,
}

impl Command {
    fn opcode(&self) -> u8 {
        match *self {
            GetStatus => 0xc0,
            WriteRegister => 0x18,
            ReadRegister => 0x19,
            WriteBuffer => 0x1A,
            ReadBuffer => 0x1B,
            SetSleep => 0x84,
            SetStandby => 0x80,
            SetFs => 0xC1,
            SetTx => 0x83,
            SetRx => 0x82,
            SetRxDutyCycle => 0x94,
            SetCAD => 0xC5,
            SetTxContinuousWave => 0xD1,
            SetTxContinuousPreamble => 0xD2,
            SetPacketType => 0x8A,
            GetPacketType => 0x03,
            SetRfFrequency => 0x86,
            SetTxParams => 0x8E,
            SetCadParams => 0x88,
            SetBufferBaseAddress => 0x8F,
            SetModulationParams => 0x8B,
            SetPacketParams => 0x8C,
            GetRxBufferStatus => 0x17,
            GetPacketStatus => 0x1D,
            GetRssilnst => 0x1F,
            SetDioIrqParams => 0x8D,
            GetIrqStatus => 0x15,
            ClearIrqStatus => 0x97,
            SetRegulatorMode => 0x96,
            SetSaveContext => 0xD5,
            SetAutoFS => 0x9E,
            SetAutoTx => 0x98,
            SetLongPreamble => 0x9B,
            SetUartSpeed => 0x9D,
            SetRangingRole => 0xA3,
            SetAdvancedRanging => 0x9A,
        }
    }
}

enum Register {
    RxGain,
    ManualGainSetting,
    LNAGainValue,
    LNAGainControl,
    SynchPeakAttenuation,
    PayloadLength,
    LoRaHeaderMode,
    RangingRequestAddress3,
    RangingRequestAddress2,
    RangingRequestAddress1,
    RangingRequestAddress0,
    RangingDeviceAddress3,
    RangingDeviceAddress2,
    RangingDeviceAddress1,
    RangingDeviceAddress0,
    RangingFilterWindowSize,
    ResetRangingFilter,
    RangingResultMUX,
    SFAdditionalConfiguration,

    

}

const REG_ID_RXGAIN: u16 = 0x981;

struct SX1280_registers {
    rx_gain: u8,
    manual_gain_setting: u8,
    lna_gain_value: u8,
    lna_gain_control: u8,
    sync_peak_attenuation: u8,
    payload_length: u8,
    lora_header_mode: u8,
    ranging_request_addr: [u8; 4],
    ranging_device_addr: [u8; 4],
    ranging_filter_window_size: u8,
    reset_ranging_filter: u8,
    ranging_result_mux: u8,
    sf_additional_configuration: u8,
    ranging_calibration_byte: [u8; 3],
    ranging_id_check_length: u8,
    frequency_error_correction: u8,
    lora_sync_word: [u8; 2],
    fei_byte: [u8; 3],
    ranging_result_byte: [u8; 3],
    ranging_rssi: u8,
    freeze_ranging_result: u8,
    packet_preamble_settings: u8,
    whitening_initial_value: u8,
    crc_polynomial_definition: u16,
    crc_polynomial_seed: u32,
    crc_initial_value: u16,
    sync_address_control: u8,
    sync_address_1: u64,
    sync_address_2: u64,
    sync_address_3: u64,
}

struct SX1280 {
    regs: SX1280_registers,
}

impl SX1280 {

    fn WriteRegister(addr : u16, )

    fn SetModulationParams(modParam1: u8, modParam2: u8, modParam3: u8) -> i16 {
        let data: [u8, 3] = [modParam1, modParam2, modParam3];
        // return(self::mod->SPIwriteStream(SX128X_CMD_SET_MODULATION_PARAMS, data, 3));
        // self::mod::SPIwriteStream(SX1280_CMD_SET_MODULATION_PARAMS, data, 3)
        // (write - master out/slave in pin 2, read - master in/slave out pin 3, chip_select pin 4 -> 3 chip selects)
        return 0
    }
    
}

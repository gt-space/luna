use libm::{fabs};

// error codes
const ERR_INVALID_BANDWIDTH: i16   = -8;
const ERR_INVALID_SPREADING_FACTOR = -9
const ERR_INVALID_CODING_RATE      = -10
const ERR_INVALID_BIT_RANGE        = -11
const ERR_INVALID_FREQUENCY        = -12
const ERR_INVALID_OUTPUT_POWER     = -13
const ERR_INVALID_PREAMBLE_LENGTH  = -18
const ERR_WRONG_MODEM: i16         = -20;

// SX1280 physical layer properties
const SX1280_FREQUENCY_STEP_SIZE = 198.3642578;
const SX1280_MAX_PACKET_LENGTH = 255;
const SX1280_CRYSTAL_FREQ = 52.0;
const SX1280_DIV_EXPONENT = 18;

// PacketType Definition
const PACKET_TYPE_GFSK    = 0x00
const PACKET_TYPE_LORA    = 0x01
const PACKET_TYPE_RANGING = 0x02
const PACKET_TYPE_FLRC    = 0x03
const PACKET_TYPE_BLE     = 0x04

// SX1280_CMD_SET_MODULATION_PARAMS
const SX128X_LORA_SF_5  = 0x50                          //  7     0   LoRa spreading factor: 5
const SX128X_LORA_SF_6  = 0x60                          //  7     0                          6
const SX128X_LORA_SF_7  = 0x70                          //  7     0                          7
const SX128X_LORA_SF_8  = 0x80                          //  7     0                          8
const SX128X_LORA_SF_9  = 0x90                          //  7     0                          9
const SX128X_LORA_SF_10 = 0xA0                          //  7     0                          10
const SX128X_LORA_SF_12 = 0xC0                          //  7     0                          12
const SX128X_LORA_SF_11 = 0xB0                          //  7     0                          11
const SX128X_LORA_BW_1625_00 = 0x0A                     //  7     0   LoRa bandwidth: 1625.0 kHz
const SX128X_LORA_BW_812_50  = 0x18                     //  7     0                   812.5 kHz
const SX128X_LORA_BW_406_25  = 0x26                     //  7     0                   406.25 kHz
const SX128X_LORA_BW_203_125 = 0x34                     //  7     0                   203.125 kHz

bandwidth: i16,
bandwidthkhz: f32,
spreading_factor: u8,
coding_rate_lora: u8,
coding_rate_flrc: u8,
preamble_length_lora,
bit_rate,
shaping,
power: i8,
    
fn CheckRange(var: f32, min: f32, max: f32, err: i16) {
    if !(var >= min && var <= max) {
        return err
    }
}

fn Assert(state_var) {
  if state_var != ERR_NONE {
    return state_var;
  }
}

fn SetFrequency(freq: f32) -> i16 {
    CheckRange(freq, 2400.0, 2500.0, ERR_INVALID_FREQUENCY);

    // calculate raw value
    let frf: u32 = (freq * ((1 as u32) << SX1280_DIV_EXPONENT)) / SX1280_CRYSTAL_FREQ;

    return SetRfFrequency(frf);
}

fn SetBandwidth(bw: f32) -> i16 {
    // check active modem
    let modem: u8 = GetPacketType();

    if modem == PACKET_TYPE_LORA {
        // check range for LoRa
        CheckRange(bw, 203.125, 1625.0, ERR_INVALID_BANDWIDTH);
    } else if modem == PACKET_TYPE_RANGING {
        // check range for ranging
        CheckRange(bw, 406.25, 1625.0, ERR_INVALID_BANDWIDTH);
    } else {
        return ERR_WRONG_MODEM;
    }
  
    if fabs(bw - 203.125) <= 0.001 {
        self::bandwidth = SX128X_LORA_BW_203_125;
    } else if fabs(bw - 406.25) <= 0.001 {
        self::bandwidth = SX128X_LORA_BW_406_25;
    } else if fabs(bw - 812.5) <= 0.001 {
        self::bandwidth = SX128X_LORA_BW_812_50;
    } else if fabs(bw - 1625.0) <= 0.001 {
        self::bandwidth = SX128X_LORA_BW_1625_00;
    } else {
        return ERR_INVALID_BANDWIDTH;
    }
  
    // update modulation parameters
    self::bandwidthkhz = bw;
    return SetModulationParams(self::spreading_factor, self::bandwidth, self::coding_rate_lora);
}

fn SetSpreadingFactor(sf: u8) -> i16 {
    // check active modem
    let modem: u8 = GetPacketType();
    if modem == PACKET_TYPE_LORA {
        // check range for LoRa
        CheckRange(sf, 5, 12, ERR_INVALID_SPREADING_FACTOR);
    } else if modem == PACKET_TYPE_RANGING {
        // check range for ranging
        CheckRange(sf, 5, 10, ERR_INVALID_SPREADING_FACTOR);
    } else {
      return ERR_WRONG_MODEM;
    }
  
    // update modulation parameters
    self::spreading_factor = sf << 4;
    let state: i16 = SetModulationParams(self::spreading_factor, self::bandwidth, self::coding_rate_lora);
    Assert(state);
  
    // update mystery register in LoRa mode - SX1280 datasheet v3.0 section 13.4.1
    if modem == X128X_PACKET_TYPE_LORA {
      let data: u8 = 0;
      if (self::spreading_factor == SX128X_LORA_SF_5) || (self::spreading_factor == SX128X_LORA_SF_6) {
        data = 0x1E;
      } else if (self::spreading_factor == SX128X_LORA_SF_7) || (self::spreading_factor == SX128X_LORA_SF_8) {
        data = 0x37;
      } else {
        data = 0x32;
      }
      const SX128X_REG_LORA_SF_CONFIG = 0x0925;
      state = WriteRegister(SX128X_REG_LORA_SF_CONFIG, &[data], 1);
    }
  
    return state;
}
  
fn SetCodingRate(cr: u8, long_inter_leaving: bool) -> i16 {
    // check active modem
    let modem: u8 = GetPacketType();
  
    // LoRa/ranging
    if (modem == PACKET_TYPE_LORA) || (modem == PACKET_TYPE_RANGING) {
        CheckRange(cr, 5, 8, ERR_INVALID_CODING_RATE);
    
        // update modulation parameters
        if long_inter_leaving && (modem == PACKET_TYPE_LORA) {
            self::coding_rate_lora = cr;
        } else {
            self::coding_rate_lora = cr - 4;
        }
        return SetModulationParams(self::spreading_factor, self::bandwidth, self::coding_rate_lora);
  
    // FLRC
    } else if modem == PACKET_TYPE_FLRC {
        CheckRange(cr, 2, 4, ERR_INVALID_CODING_RATE);
    
        // update modulation parameters
        self::codingrateflrc = (cr - 2) * 2;
        return setModulationParams(self::bit_rate, self::codingrate_flrc, self::shaping);
    }
  
    return ERR_WRONG_MODEM;
}
  
fn SetOutputPower(pwr: i8) -> i16 {
    ChipSelect(pwr, -18, 13, ERR_INVALID_OUTPUT_POWER);
    self::power = pwr + 18;
    return SetTxParams(self::power);
}
  
fn SetPreambleLength(preamble_length: u32) -> i16 {
  let modem: u8 = GetPacketType();
  if (modem == PACKET_TYPE_LORA) || (modem == PACKET_TYPE_RANGING) {
    // LoRa or ranging
    CheckRange(preamble_length, 2, 491520, ERR_INVALID_PREAMBLE_LENGTH);

    // check preamble length is even - no point even trying odd numbers
    if preamble_length % 2 != 0 {
        return ERR_INVALID_PREAMBLE_LENGTH;
    }

    // calculate exponent and mantissa values (use the next longer preamble if there's no exact match)
    let len: u32 = 0;
    for e: u8 in 1..15 {
      for m: u8 in 1..15 {
        len = m * (1 as u32 << e);
        if len >= preamble_length {
          break;
        }
      }
      if len >= preamble_length {
        break;
      }
    }

    // update packet parameters
    self::preamble_length_lora = (e << 4) | m;
    return SetPacketParams(self::preamble_length_lora, self::header_type, self::payload_length, self::crc_lora, self::invert_iq_enabled);

  } else if (modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_FLRC) {
    // GFSK or FLRC
    CheckRange(preamble_length, 4, 32, ERR_INVALID_PREAMBLE_LENGTH);

    // check preamble length is multiple of 4
    if(preamble_length % 4 != 0) {
      return ERR_INVALID_PREAMBLE_LENGTH;
    }

    // update packet parameters
    self::preamble_length_gfsk = ((preamble_length / 4) - 1) << 4;
    return SetPacketParamsGFSK(self::preamble_length_gfsk, self::sync_word_len, self::sync_word_match, self::crc_gfsk, self::whitening);
  }

  return ERR_WRONG_MODEM;
}
  
fn SetBitRate(br: f32) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();

  // GFSK/BLE
  if (modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_BLE) {
    match br as u16 {
      125 => self::bit_rate = SX128X_BLE_GFSK_BR_0_125_BW_0_3,
      250 => self::bit_rate = SX128X_BLE_GFSK_BR_0_250_BW_0_6,
      400 => self::bit_rate = SX128X_BLE_GFSK_BR_0_400_BW_1_2,
      500 => self::bit_rate = SX128X_BLE_GFSK_BR_0_500_BW_1_2,
      800 => self::bit_rate = SX128X_BLE_GFSK_BR_0_800_BW_2_4,
      1000 => self::bit_rate = SX128X_BLE_GFSK_BR_1_000_BW_2_4,
      1600 => self::bit_rate = SX128X_BLE_GFSK_BR_1_600_BW_2_4,
      2000 => self::bit_rate = SX128X_BLE_GFSK_BR_2_000_BW_2_4,
      _ => return ERR_INVALID_BIT_RATE
    }

    // update modulation parameters
    self::bit_rate_kbps = br as u16;
    return SetModulationParams(self::bit_rate, self::mod_index, self::shaping);

  // FLRC
  } else if modem == PACKET_TYPE_FLRC {
    match br as u16 {
      260 => self::bit_rate = SX128X_FLRC_BR_0_260_BW_0_3,
      325 => self::bit_rate = SX128X_FLRC_BR_0_325_BW_0_3,
      520 => self::bit_rate = SX128X_FLRC_BR_0_520_BW_0_6,
      650 => self::bit_rate = SX128X_FLRC_BR_0_650_BW_0_6,
      1000 => self::bit_rate = SX128X_FLRC_BR_1_000_BW_1_2,
      1300 => self::bit_rate = SX128X_FLRC_BR_1_300_BW_1_2,
      _ => return ERR_INVALID_BIT_RATE
    }

    // update modulation parameters
    self::bit_rate_kbps = br as u16;
    return SetModulationParams(self::bit_rate, self::coding_rate_flrc, self::shaping);

  }

  return ERR_WRONG_MODEM;
}

fn SetFrequencyDeviation(freq_dev: f32) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();
  if !((modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_BLE)) {
    return ERR_WRONG_MODEM;
  }

  // set frequency deviation to lowest available setting (required for digimodes)
  let new_freq_dev: f32 = feq_dev;
  if freq_dev < 0.0 {
    new_freq_dev = 62.5;
  }

  CheckRange(freq_dev, 62.5, 1000.0, ERR_INVALID_FREQUENCY_DEVIATION);

  // override for the lowest possible frequency deviation - required for some PhysicalLayer protocols
  if new_freq_dev == 0.0 {
    self::mod_index = SX128X_BLE_GFSK_MOD_IND_0_35;
    self::bit_rate = SX128X_BLE_GFSK_BR_0_125_BW_0_3;
    return SetModulationParams(self::bandwidth, self::mod_index, self::shaping);
  }

  // update modulation parameters
  let mod_index: u8 = ((8.0 * (new_freq_dev / (self::bit_rate_kbps as f32))) - 1.0) as u8;
  if mod_index > SX128X_BLE_GFSK_MOD_IND_4_00 {
    return ERR_INVALID_MODULATION_PARAMETERS;
  }

  // update modulation parameters
  self::mod_index = mod_index;
  return SetModulationParams(self::bit_rate, self::mod_index, self::shaping);
}

fn SetDataShaping(sh: u8) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();
  if !((modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_BLE) || (modem == PACKET_TYPE_FLRC)) {
    return ERR_WRONG_MODEM;
  }

  // set data self::shaping
  match sh {
    SHAPING_NONE => self::shaping = SX128X_BLE_GFSK_BT_OFF,
    SHAPING_0_5 => self::shaping = SX128X_BLE_GFSK_BT_0_5,
    SHAPING_1_0 => self::shaping = SX128X_BLE_GFSK_BT_1_0,
    _ => return ERR_INVALID_DATA_SHAPING,
  }

  // update modulation parameters
  if (modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_BLE) {
    return SetModulationParams(self::bit_rate, self::mod_index, self::shaping);
  } else {
    return SetModulationParams(self::bit_rate, self::coding_rate_flrc, self::shaping);
  }
}

fn SetSyncWord(sync_word: *const u8, len: u8) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();
  if !((modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_FLRC)) {
    return ERR_WRONG_MODEM;
  }

  if modem == PACKET_TYPE_GFSK {
    // GFSK can use up to 5 bytes as sync word
    if len > 5 {
      return ERR_INVALID_SYNC_WORD;
    }

    // calculate sync word length parameter value
    if len > 0 {
      self::sync_word_len = (len - 1)*2;
    }

  } else {
    // FLRC requires 32-bit sync word
    if !((len == 0) || (len == 4)) {
      return ERR_INVALID_SYNC_WORD;
    }

    // save sync word length parameter value
    self::sync_word_len = len;
  }

  // reverse sync word byte order
  let sync_word_buff: [u8; 5] = [0x00, 0x00, 0x00, 0x00, 0x00];
  for i: u8 in 0...len {
    sync_word_buff[4 - i] = sync_word[i];
  }

  // update sync word
  let state: i16 = WriteRegister(SX128X_REG_SYNC_WORD_1_BYTE_4, sync_word_buff, 5);
  Assert(state);

  // update packet parameters
  if self::sync_word_len == 0 {
    self::sync_word_match = SX128X_GFSK_FLRC_SYNC_WORD_OFF;
  } else {
    /// \todo add support for multiple sync words
    self::sync_word_match = SX128X_GFSK_FLRC_SYNC_WORD_1;
  }
  return SetPacketParamsGFSK(self::preamble_length_gfsk, self::sync_word_len, self::sync_word_match, self::header_type, self::payload_length, self::crc_gfsk, self::whitening);
}

fn SetSyncWord(uint8_t syncWord, uint8_t controlBits) -> i16 {
  // check active modem
  if GetPacketType() != PACKET_TYPE_LORA {
    return ERR_WRONG_MODEM;
  }

  // update register
  let data: [u8; 2] = [((sync_word & 0xF0) | ((control_bits & 0xF0) >> 4)) as u8, (((sync_word & 0x0F) << 4) | (control_bits & 0x0F)) as u8];
  return WriteRegister(SX128X_REG_LORA_SYNC_WORD_MSB, data, 2);
}

fn SetCRC(len: u8, initial: u32, polynomial: u16) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();

  let state: i16;
  if (modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_FLRC) {
    // update packet parameters
    if modem == PACKET_TYPE_GFSK {
      if len > 2 {
        return ERR_INVALID_CRC_CONFIGURATION;
      }
    } else {
      if len > 3 {
        return ERR_INVALID_CRC_CONFIGURATION;
      }
    }
    self::crc_gfsk = len << 4;
    state = SetPacketParamsGFSK(self::preamble_length_gfsk, self::sync_word_len, self::sync_word_match, self::crc_gfsk, self::whitening);
    Assert(state);

    // set initial CRC value
    let data: [u8; 2] = [((initial >> 8) & 0xFF) as u8, (initial & 0xFF) as u8];
    state = WriteRegister(SX128X_REG_CRC_INITIAL_MSB, data, 2);
    Assert(state);

    // set CRC polynomial
    data[0] = ((polynomial >> 8) & 0xFF) as u8;
    data[1] = (polynomial & 0xFF) as u8;
    state = WriteRegister(SX128X_REG_CRC_POLYNOMIAL_MSB, data, 2);
    return state;

  } else if modem == PACKET_TYPE_BLE {
    // update packet parameters
    if len == 0 {
      self::crc_ble = SX128X_BLE_CRC_OFF;
    } else if len == 3 {
      self::crc_ble = SX128X_BLE_CRC_3_BYTE;
    } else {
      return ERR_INVALID_CRC_CONFIGURATION;
    }
    state = SetPacketParamsBLE(self::connection_state, self::crc_ble, self::ble_test_payload, self::whitening);
    Assert(state);

    // set initial CRC value
    let data: [u8, 3] = [((initial >> 16) & 0xFF) as u8, ((initial >> 8) & 0xFF) as u8, (initial & 0xFF) as u8];
    state = WriteRegister(SX128X_REG_BLE_CRC_INITIAL_MSB, data, 3);
    return state;

  } else if (modem == PACKET_TYPE_LORA) || (modem == PACKET_TYPE_RANGING) {
    // update packet parameters
    if len == 0 {
      self::crc_lora = SX128X_LORA_CRC_OFF;
    } else if(len == 2) {
      self::crc_lora = SX128X_LORA_CRC_ON;
    } else {
      return ERR_INVALID_CRC_CONFIGURATION;
    }
    state = SetPacketParamsLoRa(self::preamble_length_lora, self::header_type, self::payload_length, self::crc_lora, self::invert_iq_enabled);
    return state;
  }

  return ERR_UNKNOWN;
}

fn SetWhitening(enabled: bool) -> i16 {
  // check active modem
  let modem: u8 = GetPacketType();
  if !((modem == PACKET_TYPE_GFSK) || (modem == PACKET_TYPE_BLE)) {
    return ERR_WRONG_MODEM;
  }

  // update packet parameters
  if enabled {
    self::whitening = SX128X_GFSK_BLE_WHITENING_ON;
  } else {
    self::whitening = SX128X_GFSK_BLE_WHITENING_OFF;
  }

  if modem == PACKET_TYPE_GFSK {
    return SetPacketParamsGFSK(self::preamble_length_gfsk, self::sync_word_len, self::sync_word_match, self::crc_gfsk, self::whitening);
  }
  return SetPacketParamsBLE(self::connection_state, self::crc_ble, self::ble_test_payload, self::whitening);
}

fn SetAccessAddress(addr: u32) -> i16 {
  // check active modem
  if GetPacketType() != PACKET_TYPE_BLE {
    return ERR_WRONG_MODEM;
  }

  // set the address
  let addrBuff: [u8, 4] = [((addr >> 24) & 0xFF) as u8, ((addr >> 16) & 0xFF) as u8, ((addr >> 8) & 0xFF) as u8, (addr & 0xFF) as u8];
  return WriteRegister(SX128X_REG_ACCESS_ADDRESS_BYTE_3, addrBuff, 4);
}

fn SetHighSensitivityMode(enable: bool) -> i16 {
  // read the current registers
  let rx_gain: u8 = 0;
  let state: i16 = ReadRegister(SX128X_REG_GAIN_MODE, &rx_gain, 1);
  Assert(state);

  if enable {
    rx_gain |= 0xC0; // Set bits 6 and 7
  } else {
    rx_gain &= ~0xC0; // Unset bits 6 and 7
  }

  // update all values
  state = WriteRegister(SX128X_REG_GAIN_MODE, &rx_gain, 1);
  return state;
}

fn SetGainControl(gain: u8) -> i16 {
  // read the current registers
  let manual_gain_setting: u8 = 0;
  let state: i16 = ReadRegister(SX128X_REG_MANUAL_GAIN_CONTROL_ENABLE_2, &manual_gain_setting, 1);
  Assert(state);
  let lna_gain_value: u8 = 0;
  state = ReadRegister(SX128X_REG_MANUAL_GAIN_SETTING, &lna_gain_value, 1);
  Assert(state);
  let lna_gain_control: u8 = 0;
  state = ReadRegister(SX128X_REG_MANUAL_GAIN_CONTROL_ENABLE_1, &lna_gain_control, 1);
  Assert(state);

  // set the gain
  if gain > 0 && gain < 14 {
    // Set manual gain
    manual_gain_setting &= ~0x01; // Set bit 0 to 0 (Enable Manual Gain Control)
    lna_gain_value &= 0xF0; // Bits 0, 1, 2 and 3 to 0
    lna_gain_value |= gain; // Set bits 0, 1, 2 and 3 to Manual Gain Setting (1-13)
    lna_gain_control |= 0x80; // Set bit 7 to 1 (Enable Manual Gain Control)
  } else {
    // Set automatic gain if 0 or out of range
    manual_gain_setting |= 0x01; // Set bit 0 to 1 (Enable Automatic Gain Control)
    lna_gain_value &= 0xF0; // Bits 0, 1, 2 and 3 to 0
    lna_gain_value |= 0x0A; // Set bits 0, 1, 2 and 3 to Manual Gain Setting (1-13)
    lna_gain_control &= ~0x80; // Set bit 7 to 0 (Enable Automatic Gain Control)
  }

  // update all values
  state = WriteRegister(SX128X_REG_MANUAL_GAIN_CONTROL_ENABLE_2, &manual_gain_setting, 1);
  Assert(state);
  state = WriteRegister(SX128X_REG_MANUAL_GAIN_SETTING, &lna_gain_value, 1);
  Assert(state);
  state = WriteRegister(SX128X_REG_MANUAL_GAIN_CONTROL_ENABLE_1, &lna_gain_control, 1);
  return state;
}
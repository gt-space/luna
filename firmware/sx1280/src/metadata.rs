// value is the opcode sent over SPI
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Command {
  GetStatus = 0xC0,
  WriteRegister = 0x18,
  ReadRegister = 0x19,
  WriteBuffer = 0x1A,
  ReadBuffer = 0x1B,
  SetSleep = 0x84,
  SetStandby = 0x80,
  SetFs = 0xC1,
  SetTx = 0x83,
  SetRx = 0x82,
  SetRxDutyCycle = 0x94,
  SetCAD = 0xC5,
  SetTxContinuousWave = 0xD1,
  SetTxContinuousPreamble = 0xD2,
  SetPacketType = 0x8A,
  GetPacketType = 0x03,
  SetRfFrequency = 0x86,
  SetTxParams = 0x8E,
  SetCadParams = 0x88,
  SetBufferBaseAddress = 0x8F,
  SetModulationParams = 0x8B,
  SetPacketParams = 0x8C,
  GetRxBufferStatus = 0x17,
  GetPacketStatus = 0x1D,
  GetRssilnst = 0x1F,
  SetDioIrqParams = 0x8D,
  GetIrqStatus = 0x15,
  ClearIrqStatus = 0x97,
  SetRegulatorMode = 0x96,
  SetSaveContext = 0xD5,
  SetAutoFS = 0x9E,
  SetAutoTx = 0x98,
  SetLongPreamble = 0x9B,
  SetUartSpeed = 0x9D,
  SetRangingRole = 0xA3,
  SetAdvancedRanging = 0x9A,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Irq {
  TxDone = 0,
  RxDone = 1,
  SyncWordValid = 2,
  SyncWordError = 3,
  HeaderValid = 4,
  HeaderError = 5,
  CrcError = 6,
  RangingSlaveResponseDone = 7,
  RangingSlaveRequestDiscard = 8,
  RangingMasterResultValid = 9,
  RangingMasterTimeout = 10,
  RangingSlaveRequestValid = 11,
  CadDone = 12,
  CadDetected = 13,
  RxTxTimeout = 14,
  PreambleDetectedOrAdvancedRangingDone = 15
}

#[repr(usize)]
#[derive(Copy, Clone, Debug)]
pub enum Dio {
  Dio1 = 0,
  Dio2 = 1,
  Dio3 = 2
}
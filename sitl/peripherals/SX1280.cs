using Antmicro.Renode.Exception;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Wireless {
  public class SX1280 : ISPIPeripheral {
    enum Command : byte {
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
      SetCad = 0xC5,
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
      GetRssiInst = 0x1F,
      SetDioIrqParams = 0x8D,
      GetIrqStatus = 0x15,
      ClrIrqStatus = 0x97,
      SetRegulatorMode = 0x96,
      SetSaveContext = 0xD5,
      SetAutoFs = 0x9E,
      SetAutoTx = 0x98,
      SetLongPreamble = 0x9B,
      SetUartSpeed = 0x9D,
      SetRangingRole = 0xA3,
      SetAdvancedRanging = 0x9A,
    }

    enum Register : ushort {
      FirmwareVersions = 0x153,
      RxGain = 0x891,
      ManualGainSetting = 0x895,
      LnaGainValue = 0x89E,
      LnaGainControl = 0x89F,
      SynchPeakAttenuation = 0x8C2,
      PayloadLength = 0x901,
      LoraHeaderMode = 0x903,
      RangingRequestAddressByte3 = 0x912,
      RangingRequestAddressByte2 = 0x913,
      RangingRequestAddressByte1 = 0x914,
      RangingRequestAddressByte0 = 0x915,
      RangingDeviceAddressByte3 = 0x916,
      RangingDeviceAddressByte2 = 0x917,
      RangingDeviceAddressByte1 = 0x918,
      RangingDeviceAddressByte0 = 0x919,
      RangingFilterWindowSize = 0x91E,
      ResetRangingFilter = 0x923,
      RangingResultMux = 0x924,
      SfAdditionalConfiguration = 0x925,
      RangingCalibrationByte2 = 0x92B,
      RangingCalibrationByte1 = 0x92C,
      RangingCalibrationByte0 = 0x92D,
      RangingIdCheckLength = 0x931,
      FrequencyErrorCorrection = 0x93C,
      CadDetPeak = 0x942,
      LoraSyncWord = 0x944,
      HeaderCrc = 0x954,
      CodingRate = 0x950,
      FeiByte2 = 0x954,
      FeiByte1 = 0x955,
      FeiByte0 = 0x956,
      RangingResultByte2 = 0x961,
      RangingResultByte1 = 0x962,
      RangingResultByte0 = 0x963,
      RangingRssi = 0x964,
      FreezeRangingResult = 0x97F,
      PacketPreambleSettings = 0x9C1,
      WhiteningInitialValue = 0x9C5,
      CrcPolynomialDefinitionMSB = 0x9C6,
      CrcPolynomialDefinitionLSB = 0x9C7,
      CrcPolynomialSeedByte2 = 0x9C7,
      CrcPolynomialSeedByte1 = 0x9C8,
      CrcPolynomialSeedByte0 = 0x9C9,
      CrcMsbInitialValue = 0x9C8,
      CrcLsbInitialValue = 0x9C9,
      SyncAddressControl = 0x9CD,
      SyncAddress1Byte4 = 0x9CE,
      SyncAddress1Byte3 = 0x9CF,
      SyncAddress1Byte2 = 0x9D0,
      SyncAddress1Byte1 = 0x9D1,
      SyncAddress1Byte0 = 0x9D2,
      SyncAddress2Byte4 = 0x9D3,
      SyncAddress2Byte3 = 0x9D4,
      SyncAddress2Byte2 = 0x9D5,
      SyncAddress2Byte1 = 0x9D6,
      SyncAddress2Byte0 = 0x9D7,
      SyncAddress3Byte4 = 0x9D8,
      SyncAddress3Byte3 = 0x9D9,
      SyncAddress3Byte2 = 0x9DA,
      SyncAddress3Byte1 = 0x9DB,
      SyncAddress3Byte0 = 0x9DC,
    }

    enum CircuitMode : byte {
      Reset,
      Startup,
      StdbyRc = 2,
      StdbyXosc = 3,
      Fs = 4,
      Rx = 5,
      Tx = 6,
      Sleep,
    }

    enum CommandStatus : byte {
      Processed = 1,
      DataAvailable = 2,
      TimeOut = 3,
      ProcessingError = 4,
      ExecuteFailed = 5,
      TxDone = 6,
    }

    private static readonly HashSet<Register> nonReadableRegisters = new() {
      Register.RangingFilterWindowSize,
    };

    private static readonly HashSet<Register> nonWritableRegisters = new() {
      Register.FirmwareVersions,
      Register.RangingFilterWindowSize,
      Register.HeaderCrc,
      Register.CodingRate,
      Register.FeiByte2,
      Register.FeiByte1,
      Register.FeiByte0,
    };

    private const ushort RegisterContiguousStartAddress = 0x891;
    private const ushort RegisterContiguousEndAddress = 0x9DC;
    private const ushort RegisterContiguousAddressSpan =
      RegisterContiguousEndAddress - RegisterContiguousStartAddress + 1;

    private byte[] buffer = new byte[256];
    private byte[] registers = new byte[RegisterContiguousAddressSpan];
    private byte bufferBaseAddress;

    private CircuitMode circuitMode;
    private CommandStatus commandStatus;

    private readonly LimitTimer timer;
    private readonly IMachine machine;

    private class TransferState {
      public Command command;
      public uint index;
      public ushort address;

      public TransferState(Command command) {
        this.command = command;
        index = 0;
        address = 0;
      }
    }

    private TransferState? transfer;

    public SX1280(IMachine machine) {
      this.machine = machine;
      timer = new LimitTimer(machine.ClockSource, frequency);

      transfer = null;

      Reset();
    }

    public void Reset() {
      bufferBaseAddress = 0;
      Array.Clear(buffer);

      // registers = new ByteRegisterCollection(this);
      // registers.DefineRegister(Register.FirmwareVersions);
    }

    public byte Transmit(byte data) {
      if (transfer is null) {
        transfer = new TransferState((Command) data);
        return GetStatus();
      }

      switch (transfer.command) {
        case Command.WriteRegister:
          if (transfer.index <= 2) {
            transfer.address = (transfer.address << 8 | (ushort) data);
            break;
          }

          WriteRegister(transfer.address, data);
          transfer.address++;
          break;
        case Command.ReadRegister:
          if (transfer.index <= 2) {
            transfer.address = (transfer.address << 8 | (ushort) data);
            break;
          } else if (transfer.index == 3) {
            break;
          }

          byte read = ReadRegister(transfer.address);
          transfer.address++;
          return read;
      }

      // The default behavior for non-read commands is to return status.
      return GetStatus();
    }

    public void FinishTransmission() {
      transfer = null;
    }

    public byte? GetStatus() {
      return (byte) ((byte) circuitMode << 5 | (byte) commandStatus << 2);
    }

    public byte? WriteBuffer(byte offset, byte value) {
      return null;
    }

    public byte ReadBuffer(byte offset) {
      return 
    }

    public void WriteRegister(ushort address, byte value) {
      if (
        !Enum.IsDefined(typeof(Register), address)
        || nonWritableRegisters.Contains((Register) address)
      ) {
        return;
      }

      ushort offset = address - RegisterContiguousStartAddress;
      return registers[offset];
    }

    public byte ReadRegister(ushort address) {
      if (
        !Enum.IsDefined(typeof(Register), address)
        || nonReadableRegisters.Contains((Register) address)
      ) {
        if (address == 0x153) {
          return 0xB7;
        } else if (address == 0x154) {
          return 0xA9;
        }

        return 0x00;
      }

      ushort offset = address - RegisterContiguousStartAddress;
      return registers[offset];
    }
  }
}

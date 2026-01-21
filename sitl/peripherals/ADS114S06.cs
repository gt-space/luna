using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class ADS114S06 : ISPIPeripheral, ISensor,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    private readonly LimitTimer timer;

    public GPIO DataReady { get; private set; }
    public GPIO Start { get; private set; }
    public ByteRegisterCollection RegistersCollection { get; private set; }

    public ADS114S06(IMachine machine)
    {
      timer = new LimitTimer(
        machine.ClockSource,
        4096000,
        this,
        "InternalClock",
        limit: 0,
        enabled: false,
        eventEnabled: true
      );

      DataReady = new GPIO();
      Start = new GPIO();
      Start.AddStateChangedHook(OnStartChanged);

      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    private ushort offsetCalibration;
    private ushort gainCalibration;
    private IFlagRegisterField sendCrc;
    private IFlagRegisterField sendStatus;
    private IEnumRegisterField<ConversionMode> conversionMode;

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.ID, resetValue: 0b101)
        .WithReservedBits(3, 7)
        .WithValueField(0, 3, FieldMode.Read, name: "DEV_ID");

      RegistersCollection
        .DefineRegister((long) Register.STATUS, resetValue: 0x80)
        .WithFlag(7, FieldMode.Read | FieldMode.WriteToClear, name: "FL_POR")
        .WithFlag(6, FieldMode.Read, name: "RDY")
        .WithFlag(5, FieldMode.Read, name: "FL_P_RAILP")
        .WithFlag(4, FieldMode.Read, name: "FL_P_RAILN")
        .WithFlag(3, FieldMode.Read, name: "FL_N_RAILP")
        .WithFlag(2, FieldMode.Read, name: "FL_N_RAILN")
        .WithFlag(1, FieldMode.Read, name: "FL_REF_L1")
        .WithFlag(0, FieldMode.Read, name: "FL_REF_L0");

      RegistersCollection
        .DefineRegister((long) Register.INPMUX, resetValue: 0x01)
        .WithValueField(4, 4, FieldMode.Read | FieldMode.Write, name: "MUXP")
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "MUXN");

      RegistersCollection
        .DefineRegister((long) Register.PGA)
        .WithValueField(5, 3, FieldMode.Read | FieldMode.Write, name: "DELAY")
        .WithValueField(3, 2, FieldMode.Read | FieldMode.Write, name: "PGA_EN")
        .WithValueField(0, 3, FieldMode.Read | FieldMode.Write, name: "GAIN");

      RegistersCollection
        .DefineRegister((long) Register.DATARATE, resetValue: 0x14)
        .WithFlag(7, FieldMode.Read | FieldMode.Write, name: "G_CHOP")
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "CLK")
        .WithEnumField(5, 1, out conversionMode, FieldMode.Read | FieldMode.Write, name: "MODE")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "FILTER")
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "DR");

      RegistersCollection
        .DefineRegister((long) Register.REF, resetValue: 0x10)
        .WithValueField(6, 2, FieldMode.Read | FieldMode.Write, name: "FL_REF_EN")
        .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "REFP_BUF")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "REFN_BUF")
        .WithValueField(2, 2, FieldMode.Read | FieldMode.Write, name: "REFSEL")
        .WithValueField(0, 2, FieldMode.Read | FieldMode.Write, name: "REFCON");

      RegistersCollection
        .DefineRegister((long) Register.IDACMAG)
        .WithFlag(7, FieldMode.Read | FieldMode.Write, name: "FL_RAIL_EN")
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "PSW")
        .WithReservedBits(4, 2)
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "IMAG");

      RegistersCollection
        .DefineRegister((long) Register.IDACMUX, resetValue: 0xFF)
        .WithValueField(4, 4, FieldMode.Read | FieldMode.Write, name: "I2MUX")
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "I1MUX");

      RegistersCollection
        .DefineRegister((long) Register.VBIAS)
        .WithFlag(7, FieldMode.Read | FieldMode.Write, name: "VB_LEVEL")
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "VB_AINC")
        .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "VB_AIN5")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "VB_AIN4")
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "VB_AIN3")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "VB_AIN2")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "VB_AIN1")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "VB_AIN0");

      RegistersCollection
        .DefineRegister((long) Register.SYS, resetValue: 0x10)
        .WithValueField(5, 3, FieldMode.Read | FieldMode.Write, name: "SYS_MON")
        .WithValueField(3, 2, FieldMode.Read | FieldMode.Write, name: "CAL_SAMP")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "TIMEOUT")
        .WithFlag(1, out sendCrc, FieldMode.Read | FieldMode.Write, name: "CRC")
        .WithFlag(0, out sendStatus, FieldMode.Read | FieldMode.Write, name: "SENDSTAT");

      RegistersCollection
        .DefineRegister((long) Register.OFCAL0)
        .WithValueField(
          0,
          8,
          FieldMode.Read | FieldMode.Write,
          valueProviderCallback: _ => (ulong) (offsetCalibration & 0xFF),
          changeCallback: (_, low) =>
          {
            ushort high = (ushort) (offsetCalibration & 0xFF00);
            offsetCalibration = (ushort) (high | low);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFCAL1)
        .WithValueField(
          0,
          8,
          FieldMode.Read | FieldMode.Write,
          valueProviderCallback: _ => (ulong) (offsetCalibration >> 8),
          changeCallback: (_, high) =>
          {
            ushort low = (ushort) (offsetCalibration & 0xFF);
            offsetCalibration = (ushort) ((high << 8) | low);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.FSCAL0)
        .WithValueField(
          0,
          8,
          FieldMode.Read | FieldMode.Write,
          valueProviderCallback: _ => (ulong) (gainCalibration & 0xFF),
          changeCallback: (_, low) =>
          {
            ushort high = (ushort) (gainCalibration & 0xFF00);
            gainCalibration = (ushort) (high | low);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.FSCAL1, resetValue: 0x40)
        .WithValueField(
          0,
          8,
          FieldMode.Read | FieldMode.Write,
          valueProviderCallback: _ => (ulong) (gainCalibration >> 8),
          changeCallback: (_, high) =>
          {
            ushort low = (ushort) (gainCalibration & 0xFF);
            gainCalibration = (ushort) ((high << 8) | low);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.GPIODAT)
        .WithValueField(4, 4, FieldMode.Read | FieldMode.Write, name: "DIR")
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "DAT");

      RegistersCollection
        .DefineRegister((long) Register.GPIOCON)
        .WithReservedBits(4, 4)
        .WithValueField(0, 4, FieldMode.Read | FieldMode.Write, name: "CON");
    }

    public void Reset()
    {
      command = null;
      mode = Mode.Reset;

      DataReady.Set(false);

      // Reset all internal registers.
      RegistersCollection.Reset();

      timer.LimitReached += ResetDone;
      timer.Limit = 4096;
      timer.Enabled = true;
    }

    private void ResetDone()
    {
      timer.Enabled = false;
      timer.LimitReached -= ResetDone;
      mode = Mode.Standby;
    }

    public byte Transmit(byte data)
    {
      if (mode == Mode.Reset)
      {
        return 0x00;
      }

      if (command is null && data != 0x00)
      {
        command = data;

        if (mode == Mode.PowerDown && (command & 0xFE) != 0x02)
        {
          return 0x00;
        }

        switch (command)
        {
          case 0x02: // WAKEUP
          case 0x03:
            mode = Mode.Standby;
            break;
          case 0x04: // POWERDOWN
          case 0x05:
            mode = Mode.PowerDown;
            break;
          case 0x06: // RESET
          case 0x07:
            Reset();
            break;
          case 0x08: // START
          case 0x09:
            StartConversion();
            break;
          case 0x0A: // STOP
          case 0x0B:
            mode = Mode.Standby;
            break;
          case 0x16: // SYOCAL
            break;
          case 0x17: // SYGCAL
            break;
          case 0x19: // SFOCAL
            break;
          case 0x12: // RDATA
          case 0x13:
            break;
        }
      }

      if ((command & 0x60) != 0) // RREG or WREG
      {
        if (registerAddress is null)
        {
          registerAddress = (byte) (data & 0x1F);
        }
        else if (registerCount is null)
        {
          registerCount = (byte) (data & 0x1F);
        }
        else if (registerCount > 0)
        {
          registerCount--;

          if ((command & 0x20) != 0) // RREG
          {
            byte value = 0x00;
            RegistersCollection.TryRead((long) registerAddress++, out value);
            return value;
          }
          else // WREG
          {
            RegistersCollection.TryWrite((long) registerAddress++, data);
          }
        }
      }

      return mode == Mode.Conversion
        ? StepConversion()
        : (byte) 0x00;
    }

    public void FinishTransmission()
    {
      command = null;
    }

    private void StartConversion()
    {
      mode = Mode.Conversion;
      conversionState = sendStatus.Value
        ? ConversionState.Status
        : ConversionState.Data1;
    }

    private byte StepConversion()
    {
      switch (conversionState)
      {
        case ConversionState.Status:
          conversionState = ConversionState.Data1;
          return RegistersCollection.Read((long) Register.STATUS);
        case ConversionState.Data1:
          conversionState = ConversionState.Data2;
          return 0x01; // TODO: Replace with emulated values
        case ConversionState.Data2:
          conversionState = sendCrc.Value
            ? ConversionState.Crc
            : ConversionState.Data1;
          return 0x02; // TODO: Replace with emulated values
        case ConversionState.Crc:
          conversionState = sendStatus.Value
            ? ConversionState.Status
            : ConversionState.Data1;
          return crc;
      }

      throw new InvalidOperationException(
        $"Invalid ConversionState: {conversionState}"
      );
    }

    ///////////////////
    // GPIO Handlers //
    ///////////////////

    private void OnStartChanged(bool newState)
    {
      if (newState)
      {
        // The rising edge of the START pin starts a new conversion without
        // completing the current conversion [9.4].
        StartConversion();
      }
    }

    //////////////////
    // SPI Handlers //
    //////////////////

    private enum Register : byte
    {
      ID = 0x00,
      STATUS = 0x01,
      INPMUX = 0x02,
      PGA = 0x03,
      DATARATE = 0x04,
      REF = 0x05,
      IDACMAG = 0x06,
      IDACMUX = 0x07,
      VBIAS = 0x08,
      SYS = 0x09,
      OFCAL0 = 0x0B,
      OFCAL1 = 0x0C,
      FSCAL0 = 0x0E,
      FSCAL1 = 0x0F,
      GPIODAT = 0x10,
      GPIOCON = 0x11
    }

    private enum Mode
    {
      Conversion,
      PowerDown,
      Reset,
      Standby,
    }

    private enum ConversionState
    {
      Status,
      Data1,
      Data2,
      Crc,
    }

    private enum ConversionMode
    {
      Continuous = 0,
      SingleShot = 1,
    }

    private byte? registerAddress = null;
    private byte? registerCount = null;

    private byte crc = 0x00;
    private byte? command;
    private Mode mode;
    private ConversionState conversionState;
  }
}

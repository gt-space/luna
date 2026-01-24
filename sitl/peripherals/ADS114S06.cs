using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class ADS114S06 : ISPIPeripheral, IADC,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    private readonly LimitTimer resetTimer;
    private readonly LimitTimer conversionTimer;

    public GPIO DataReady { get; private set; }
    public GPIO Start { get; private set; }
    public ByteRegisterCollection RegistersCollection { get; private set; }

    public decimal VRefP0;
    public decimal VRefN0;
    public decimal VRefP1;
    public decimal VRefN1;

    public ADS114S06(IMachine machine)
    {
      resetTimer = new LimitTimer(
        machine.ClockSource,
        4_096_000,
        this,
        "ResetTimer",
        limit: 4096,
        enabled: false,
        eventEnabled: true
      );
      resetTimer.LimitReached += OnResetDone;

      conversionTimer = new LimitTimer(
        machine.ClockSource,
        4_096_000,
        this,
        "ConversionTimer",
        limit: 1000,
        enabled: false,
        eventEnabled: true
      );
      conversionTimer.LimitReached += OnConversionFinished;

      DataReady = new GPIO();
      Start = new GPIO();
      Start.AddStateChangedHook(OnStartChanged);

      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    private ushort offsetCalibration;
    private ushort gainCalibration;

    private IValueRegisterField conversionDelay = null!;
    private IValueRegisterField pgaEnable = null!;
    private IValueRegisterField pgaGain = null!;
    private IFlagRegisterField sendCrc = null!;
    private IFlagRegisterField sendStatus = null!;
    private IEnumRegisterField<ConversionMode> conversionMode = null!;
    private IEnumRegisterField<ReferenceInput> referenceInput = null!;
    private IEnumRegisterField<InternalReferenceConfig> internalReferenceConfig = null!;

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.ID, resetValue: 0b101)
        .WithReservedBits(3, 5)
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
        .WithValueField(5, 3, out conversionDelay, FieldMode.Read | FieldMode.Write, name: "DELAY")
        .WithValueField(3, 2, out pgaEnable, FieldMode.Read | FieldMode.Write, name: "PGA_EN")
        .WithValueField(0, 3, out pgaGain, FieldMode.Read | FieldMode.Write, name: "GAIN");

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
        .WithEnumField(2, 2, out referenceInput, FieldMode.Read | FieldMode.Write, name: "REFSEL")
        .WithEnumField(0, 2, out internalReferenceConfig, FieldMode.Read | FieldMode.Write, name: "REFCON");

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

      resetTimer.Enabled = true;
    }

    private void OnResetDone()
    {
      resetTimer.Enabled = false;
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

      conversionTimer.Enabled = true;
    }

    private void OnConversionFinished()
    {
      measurement = VoltageToOutputCode(measurements[channel]);
      channel = (channel + 1) % ChannelCount;
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
          return (byte) (measurement >> 8);
        case ConversionState.Data2:
          conversionState = sendCrc.Value
            ? ConversionState.Crc
            : ConversionState.Data1;
          return (byte) (measurement & 0xFF);
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

    //////////
    // IADC //
    //////////

    // (microVolts)
    private decimal[] measurements = new decimal[ChannelCount];
    private short measurement;

    public void SetADCValue(int channel, uint microvolts)
    {
      measurements[channel] = (decimal) microvolts / 1e6m;
    }

    public uint GetADCValue(int channel)
    {
      return (uint) (measurements[channel] * 1e6m);
    }

    private decimal ReferenceVoltage()
    {
      return referenceInput.Value switch
      {
        ReferenceInput.Ref0 => VRefP0 - VRefN0,
        ReferenceInput.Ref1 => VRefP1 - VRefN1,
        ReferenceInput.Internal => 2.5m,
        _ => 0.0m,
      };
    }

    private short VoltageToOutputCode(decimal voltage)
    {
      decimal gainMultiplier = (decimal) (1 << (int) pgaGain.Value);
      decimal precise = voltage * gainMultiplier * 32768.0m / ReferenceVoltage();
      short saturated = (short) Math.Clamp(precise, short.MinValue, short.MaxValue);
      return saturated;
    }

    private const int ChannelCount = 6;
    public int ADCChannelCount => ChannelCount;

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

    public enum Mode
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

    private enum ReferenceInput
    {
      Ref0 = 0,
      Ref1 = 1,
      Internal = 2,
    }

    private enum InternalReferenceConfig
    {
      Off,
      PowersDown,
      AlwaysOn,
    }

    private byte? registerAddress = null;
    private byte? registerCount = null;

    private byte crc = 0x00;
    private byte? command;
    private int channel;
    public Mode mode;
    private ConversionState conversionState;
  }
}

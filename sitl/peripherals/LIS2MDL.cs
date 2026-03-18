using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;
using Antmicro.Renode.Utilities.RESD;

namespace Antmicro.Renode.Peripherals.Sensors
{
  /// <summary>
  /// LIS2MDL 3-axis Magnetometer Renode peripheral.
  /// </summary>
  /// <remarks>
  /// Implemented:
  /// <list type="bullet">
  ///   <item>
  ///     Reading magnetometer and temperature values directly \
  ///     from the `OUT_.*_[LH]` registers
  ///   </item>
  ///   <item>All registers defined with correct resets</item>
  ///   <item>Embedded register control and switching</item>
  ///   <item>Manual Renode monitor control of environmental conditions</item>
  ///   <item>Block data updates for high-byte sequential readings</item>
  ///   <item>SPI interface</item>
  ///   <item>Full magnetometer parameter configuration</item>
  ///   <item>Per-axis interrupt detection with customizable threshold values</item>
  ///   <item>Multiplexed Interrupt and DRDY GPIO pin</item>
  ///   <item>Software resets and memory wipes using registers</item>
  ///   <item>Environmental cancellation with customizable hard-iron offset values</item>
  ///   <item>Single and continuous measurement modes</item>
  ///   <item>Inversion of MSB and LSB</item>
  /// </list>
  ///
  /// Not yet supported:
  /// <list type="bullet">
  ///   <item>I2C Interface</item>
  ///   <item>Self test mode</item>
  ///   <item>Low-pass filter</item>
  ///   <item>Temperature compensation</item>
  /// </list>
  /// </remarks>
  public class LIS2MDL :
    BasicBytePeripheral,
    ISPIPeripheral
  {
    public LIS2MDL(IMachine machine) : base(machine)
    {
      Interrupt = new GPIO();

      magTimer = new LimitTimer(
        machine.ClockSource,
        autoUpdate: true,
        enabled: true,
        eventEnabled: true,
        frequency: 100,
        localName: "magTimer",
        owner: this
      );
      magTimer.LimitReached += OnMeasure;

      DefineRegisters();
      Reset();
    }

    public override void Reset()
    {
      RegistersCollection.Reset();
      address = null;
      isWrite = false;

      magTimer.Limit = ulong.MaxValue;
      magTimer.Reset();
      operationMode = 3; // idle (matches CFG_REG_A reset value 0x03)

      magSample = new DiscreteSample3D(0, 0, 0);
      tempSample = 0;

      latchedX = null;
      latchedY = null;
      latchedZ = null;

      xExceedsPos = false;
      xExceedsNeg = false;
      yExceedsPos = false;
      yExceedsNeg = false;
      zExceedsPos = false;
      zExceedsNeg = false;
    }

    // The true magnetic field, in gauss, of the chip environment.
    public decimal MagneticFieldX { get; set; }
    public decimal MagneticFieldY { get; set; }
    public decimal MagneticFieldZ { get; set; }

    [OnRESDSample(SampleType.MagneticFluxDensity)]
    private void HandleMagneticSample(MagneticSample sample, TimeInterval _)
    {
      // Convert magnetic flux density samples from nanotesla to gauss.
      MagneticFieldX = (decimal) sample.MagneticFluxDensityX * 1e-5m;
      MagneticFieldY = (decimal) sample.MagneticFluxDensityY * 1e-5m;
      MagneticFieldZ = (decimal) sample.MagneticFluxDensityZ * 1e-5m;
    }

    // The true temperature, in degrees Celsius, of the chip environment.
    public decimal Temperature { get; set; }

    [OnRESDSample(SampleType.Temperature)]
    private void HandleTemperatureSample(
      TemperatureSample sample,
      TimeInterval _
    )
    {
      // Convert temperature samples from milli-C to C.
      Temperature = (decimal) sample.Temperature / 1000m;
    }

    [IrqProvider]
    public GPIO Interrupt { get; }

    public byte Transmit(byte data)
    {
      if (address is null)
      {
        isWrite = (data & 0x80) == 0;
        address = (byte) (data & 0x7F);
        return 0x00;
      }

      if (isWrite)
      {
        WriteByte((long) address++, data);
        return 0x00;
      }
      else
      {
        return ReadByte((long) address++);
      }
    }

    public void FinishTransmission()
    {
      address = null;
    }

    delegate void FieldSetter(ref short field, ulong x);

    protected override void DefineRegisters()
    {
      FieldMode R = FieldMode.Read;
      FieldMode RW = R | FieldMode.Write;

      // Byte getter closures that respect the BLE (byte-level endianness)
      // register field. The setters do not need to account for inversion
      // because that is covered in the getters.
      Func<int, byte> lowByte = x => invertBytes.Value
        ? (byte) (x >> 8)
        : (byte) (x & 0xFF);

      Func<int, byte> highByte = x => invertBytes.Value
        ? (byte) (x & 0xFF)
        : (byte) (x >> 8);

      FieldSetter setLowByte = (ref short field, ulong low) =>
        field = (short) (field & 0xFF00 | (int) low);

      FieldSetter setHighByte = (ref short field, ulong high) =>
        field = (short) (field & 0x00FF | ((int) high << 8));

      Register.WHO_AM_I.Define(this, resetValue: 0x40)
        .WithValueField(0, 8, R, name: "WHO_AM_I");

      Register.OFFSET_X_REG_L.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_X_REG_L",
          valueProviderCallback: _ => lowByte(offset.X),
          writeCallback: (_, low) => setLowByte(ref offset.X, low)
        );

      Register.OFFSET_X_REG_H.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_X_REG_H",
          valueProviderCallback: _ => highByte(offset.X),
          writeCallback: (_, high) => setHighByte(ref offset.X, high)
        );

      Register.OFFSET_Y_REG_L.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_Y_REG_L",
          valueProviderCallback: _ => lowByte(offset.Y),
          writeCallback: (_, low) => setLowByte(ref offset.Y, low)
        );

      Register.OFFSET_Y_REG_H.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_Y_REG_H",
          valueProviderCallback: _ => highByte(offset.Y),
          writeCallback: (_, high) => setHighByte(ref offset.Y, high)
        );

      Register.OFFSET_Z_REG_L.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_Z_REG_L",
          valueProviderCallback: _ => lowByte(offset.Z),
          writeCallback: (_, low) => setLowByte(ref offset.Z, low)
        );

      Register.OFFSET_Z_REG_H.Define(this)
        .WithValueField(
          0, 8, RW, name: "OFFSET_Z_REG_H",
          valueProviderCallback: _ => highByte(offset.Z),
          writeCallback: (_, high) => setHighByte(ref offset.Z, high)
        );

      Register.CFG_REG_A.Define(this, resetValue: 0x03)
        .WithFlag(7, out tempCompensationEnabled, RW, name: "COMP_TEMP_EN")
        .WithFlag(
          6, RW, name: "REBOOT",
          valueProviderCallback: _ => false,
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .WithFlag(
          5, RW, name: "SOFT_RST",
          valueProviderCallback: _ => false,
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .WithFlag(4, out lowPower, RW, name: "LP")
        .WithValueField(
          2, 2, RW, name: "ODR",
          valueProviderCallback: _ => odr,
          writeCallback: (_, value) => {
            odr = value;
            magTimer.Limit = OdrToPeriod(value);
          }
        )
        .WithValueField(
          0, 2, RW, name: "MD",
          writeCallback: (_, mode) => SetMode(mode),
          valueProviderCallback: _ => operationMode
        );

      Register.CFG_REG_B.Define(this)
        .WithReservedBits(6, 2)
        .WithFlag(5, out offsetCancellationOneShot, name: "OFF_CANC_ONE_SHOT")
        .WithReservedBits(4, 1)
        .WithFlag(3, out interruptChecking, name: "INT_on_DataOFF")
        .WithFlag(2, out pulseFrequency, name: "Set_FREQ")
        .WithFlag(1, out offsetCancellation, name: "OFF_CANC")
        .WithFlag(0, out lowpassFilter, name: "LPF");

      Register.CFG_REG_C.Define(this)
        .WithReservedBits(7, 1)
        .WithFlag(
          6, out intOnPin, name: "INT_on_PIN",
          writeCallback: (_, __) => DriveDrdy()
        )
        .WithFlag(5, out I2CDisabled, name: "I2C_DIS")
        .WithFlag(4, out blockDataUpdate, name: "BDU")
        .WithFlag(3, out invertBytes, name: "BLE")
        .WithFlag(2, out enableSDO, name: "4WSPI")
        .WithFlag(1, out selfTestOn, name: "Self_test")
        .WithFlag(
          0, out drdyOnPin, name: "DRDY_on_PIN",
          writeCallback: (_, __) => DriveDrdy()
        );

      Register.INT_CRTL_REG.Define(this, resetValue: 0xE0)
        .WithFlag(7, out enableXIE, name: "XIEN")
        .WithFlag(6, out enableYIE, name: "YIEN")
        .WithFlag(5, out enableZIE, name: "ZIEN")
        .WithReservedBits(3, 2)
        .WithFlag(2, out polarity, name: "IEA")
        .WithFlag(1, out latched, name: "IEL")
        .WithFlag(0, out enableInterrupt, name: "IEN");

      Register.INT_SOURCE_REG.Define(this)
        .WithFlag(7, mode: R, name: "P_TH_S_X",
          valueProviderCallback: _ => enableXIE.Value && xExceedsPos
        )
        .WithFlag(6, mode: R, name: "P_TH_S_Y",
          valueProviderCallback: _ => enableYIE.Value && yExceedsPos
        )
        .WithFlag(5, mode: R, name: "P_TH_S_Z",
          valueProviderCallback: _ => enableZIE.Value && zExceedsPos
        )
        .WithFlag(4, mode: R, name: "N_TH_S_X",
          valueProviderCallback: _ => enableXIE.Value && xExceedsNeg
        )
        .WithFlag(3, mode: R, name: "N_TH_S_Y",
          valueProviderCallback: _ => enableYIE.Value && yExceedsNeg
        )
        .WithFlag(2, mode: R, name: "N_TH_S_Z",
          valueProviderCallback: _ => enableZIE.Value && zExceedsNeg
        )
        .WithFlag(1, out MROI, mode: R, name: "MROI")
        .WithFlag(
          0, mode: R, name: "INT",
          valueProviderCallback: _ =>
          {
            if (!enableInterrupt.Value) return false;

            return (enableXIE.Value && (xExceedsPos || xExceedsNeg))
              || (enableYIE.Value && (yExceedsPos || yExceedsNeg))
              || (enableZIE.Value && (zExceedsPos || zExceedsNeg));
          }
        )
        .WithReadCallback((_, __) =>
        {
          // In latched mode (IEL=1), flags clear on read.
          if (latched.Value)
          {
            xExceedsPos = false;
            xExceedsNeg = false;
            yExceedsPos = false;
            yExceedsNeg = false;
            zExceedsPos = false;
            zExceedsNeg = false;
          }
        });

      Register.INT_THS_L_REG.Define(this)
        .WithValueField(
          0, 8, RW, name: "INT_THS_L_REG",
          valueProviderCallback: _ => lowByte(intThreshold),
          writeCallback: (_, l) =>
          {
            intThreshold = (ushort) ((intThreshold & 0xFF00) | (int) l);
          }
        );

      Register.INT_THS_H_REG.Define(this)
        .WithValueField(
          0, 8, RW, name: "INT_THS_H_REG",
          valueProviderCallback: _ => highByte(intThreshold),
          writeCallback: (_, h) =>
          {
            intThreshold = (ushort) ((intThreshold & 0x00FF) | ((int) h << 8));
          }
        );

      Register.STATUS_REG.Define(this)
        .WithFlag(7, out overZYX, R, name: "Zyxor")
        .WithFlag(6, out overZ, R, name: "zor")
        .WithFlag(5, out overY, R, name: "yor")
        .WithFlag(4, out overX, R, name: "xor")
        .WithFlag(3, out drdyZYX, R, name: "Zyxda")
        .WithFlag(2, out drdyZ, R, name: "zda")
        .WithFlag(1, out drdyY, R, name: "yda")
        .WithFlag(0, out drdyX, R, name: "xda");

      Register.OUTX_L_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_L_REG",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedX = magSample.X;
            }
            return lowByte(magSample.X);
          }
        );

      Register.OUTX_H_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_H_REG",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedX is not null
              ? (short) latchedX
              : magSample.X;
            latchedX = null;
            drdyX.Value = false;
            return highByte(val);
          }
        );

      Register.OUTY_L_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_L_REG",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedY = magSample.Y;
            }
            return lowByte(magSample.Y);
          }
        );

      Register.OUTY_H_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_H_REG",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedY is not null
              ? (short) latchedY
              : magSample.Y;
            latchedY = null;
            drdyY.Value = false;
            return highByte(val);
          }
        );

      Register.OUTZ_L_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_L_REG",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedZ = magSample.Z;
            }
            return lowByte(magSample.Z);
          }
        );

      Register.OUTZ_H_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_H_REG",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedZ is not null
              ? (short) latchedZ
              : magSample.Z;
            latchedZ = null;
            drdyZ.Value = false;
            return highByte(val);
          }
        );

      Register.TEMP_OUT_L_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "TEMP_OUT_L_REG",
          valueProviderCallback: _ => lowByte(tempSample)
        );

      Register.TEMP_OUT_H_REG.Define(this)
        .WithValueField(
          0, 8, R, name: "TEMP_OUT_H_REG",
          valueProviderCallback: _ => highByte(tempSample)
        );
    }

    //////////////////////////
    // Magnetometer section //
    //////////////////////////

    private void OnMeasure()
    {
      if (operationMode != 0) return;
      DoMeasurement();
    }

    private void DoMeasurement()
    {
      // Quantize physical values to register samples.
      const decimal magSensitivity = 1.5m; // mgauss/LSB (datasheet Table 2)
      short rawX = QuantizeMeasurement(MagneticFieldX * 1000m, magSensitivity);
      short rawY = QuantizeMeasurement(MagneticFieldY * 1000m, magSensitivity);
      short rawZ = QuantizeMeasurement(MagneticFieldZ * 1000m, magSensitivity);

      // MROI: set if any axis overflowed the measurement range.
      decimal sx = MagneticFieldX * 1000m / magSensitivity;
      decimal sy = MagneticFieldY * 1000m / magSensitivity;
      decimal sz = MagneticFieldZ * 1000m / magSensitivity;
      MROI.Value = sx < short.MinValue || sx > short.MaxValue
        || sy < short.MinValue || sy > short.MaxValue
        || sz < short.MinValue || sz > short.MaxValue;

      // Apply hard-iron offset (datasheet sec 8.1-8.3).
      short corrX = (short) (rawX + offset.X);
      short corrY = (short) (rawY + offset.Y);
      short corrZ = (short) (rawZ + offset.Z);

      // Check interrupts before or after hard-iron correction per
      // INT_on_DataOFF (datasheet Table 27).
      if (interruptChecking.Value)
      {
        CheckInterrupts(corrX, corrY, corrZ);
      }
      else
      {
        CheckInterrupts(rawX, rawY, rawZ);
      }

      // Set overrun flags if previous data was not read.
      overX.Value = drdyX.Value;
      overY.Value = drdyY.Value;
      overZ.Value = drdyZ.Value;
      overZYX.Value = overX.Value || overY.Value || overZ.Value;

      // Update output registers.
      magSample = new DiscreteSample3D(corrX, corrY, corrZ);

      // Update temperature (refreshes at the magnetometer ODR).
      const decimal tempSensitivity = 8m; // LSB/°C (datasheet sec 8.16)
      decimal clampedTemp = Math.Clamp(Temperature, -40m, 85m);
      tempSample = (short) Math.Clamp(
        (clampedTemp - 25m) * tempSensitivity,
        short.MinValue,
        short.MaxValue
      );

      // Set data ready flags.
      drdyX.Value = true;
      drdyY.Value = true;
      drdyZ.Value = true;
      drdyZYX.Value = true;

      DriveDrdy();
      this.Log(LogLevel.Debug, $"Measurement: mag [{MagneticFieldX}, {MagneticFieldY}, {MagneticFieldZ}] -> [{corrX}, {corrY}, {corrZ}]");
    }

    private void CheckInterrupts(short x, short y, short z)
    {
      // In pulsed mode (IEL=0), flags reflect current measurement only.
      if (!latched.Value)
      {
        xExceedsPos = false;
        xExceedsNeg = false;
        yExceedsPos = false;
        yExceedsNeg = false;
        zExceedsPos = false;
        zExceedsNeg = false;
      }

      if (enableXIE.Value && Math.Abs((int) x) > intThreshold)
      {
        if (x > 0) xExceedsPos = true;
        else xExceedsNeg = true;
      }

      if (enableYIE.Value && Math.Abs((int) y) > intThreshold)
      {
        if (y > 0) yExceedsPos = true;
        else yExceedsNeg = true;
      }

      if (enableZIE.Value && Math.Abs((int) z) > intThreshold)
      {
        if (z > 0) zExceedsPos = true;
        else zExceedsNeg = true;
      }
    }

    private void DriveDrdy()
    {
      bool intTriggered = enableInterrupt.Value
        && ((enableXIE.Value && (xExceedsPos || xExceedsNeg))
            || (enableYIE.Value && (yExceedsPos || yExceedsNeg))
            || (enableZIE.Value && (zExceedsPos || zExceedsNeg)));

      // IEA controls interrupt pin polarity: 1 = active-high, 0 = active-low.
      bool intPin = intOnPin.Value && (polarity.Value == intTriggered);
      bool drdyPin = drdyOnPin.Value && drdyZYX.Value;
      Interrupt.Set(intPin || drdyPin);
    }

    private void SetMode(ulong mode)
    {
      operationMode = mode;
      if (mode == 1) // single
      {
        DoMeasurement();
        operationMode = 3; // return to idle (datasheet Table 25)
      }
    }

    private short QuantizeMeasurement(decimal exact, decimal sensitivity)
    {
      return (short) Math.Clamp(
        exact / sensitivity,
        short.MinValue,
        short.MaxValue
      );
    }

    private readonly struct DiscreteSample3D
    {
      public readonly short X, Y, Z;

      public DiscreteSample3D(short x, short y, short z)
      {
        X = x;
        Y = y;
        Z = z;
      }
    }

    /// <summary>
    /// Convenience struct for holding 3D values instead of splitting into
    /// multiple variables.
    /// </summary>
    private struct Vector3D<T>
    {
      public T X, Y, Z;
    }

    private static ulong OdrToPeriod(ulong odr)
    {
      return odr switch
      {
        0 => 10, // 10 Hz
        1 => 5,  // 20 Hz
        2 => 2,  // 50 Hz
        3 => 1,  // 100 Hz
        _ => throw new ArgumentException("Invalid ODR value."),
      };
    }

    // SPI state
    private byte? address;
    private bool isWrite;

    // Magnetometer timer
    private readonly LimitTimer magTimer;

    // Magnetometer samples and latches (BDU)
    private DiscreteSample3D magSample;
    private short tempSample;
    private short? latchedX;
    private short? latchedY;
    private short? latchedZ;

    // Hard-iron offset
    private Vector3D<short> offset;

    // CFG_REG_A
    private IFlagRegisterField tempCompensationEnabled = null!;
    private IFlagRegisterField lowPower = null!;
    private ulong odr;
    private ulong operationMode;

    // CFG_REG_B
    private IFlagRegisterField offsetCancellationOneShot = null!;
    private IFlagRegisterField interruptChecking = null!;
    private IFlagRegisterField pulseFrequency = null!;
    private IFlagRegisterField offsetCancellation = null!;
    private IFlagRegisterField lowpassFilter = null!;

    // CFG_REG_C
    private IFlagRegisterField intOnPin = null!;
    private IFlagRegisterField I2CDisabled = null!;
    private IFlagRegisterField blockDataUpdate = null!;
    private IFlagRegisterField invertBytes = null!;
    private IFlagRegisterField enableSDO = null!;
    private IFlagRegisterField selfTestOn = null!;
    private IFlagRegisterField drdyOnPin = null!;

    // INT_CRTL_REG
    private IFlagRegisterField enableXIE = null!;
    private IFlagRegisterField enableYIE = null!;
    private IFlagRegisterField enableZIE = null!;
    private IFlagRegisterField polarity = null!;
    private IFlagRegisterField latched = null!;
    private IFlagRegisterField enableInterrupt = null!;

    // INT_SOURCE_REG
    private bool xExceedsPos;
    private bool yExceedsPos;
    private bool zExceedsPos;
    private bool xExceedsNeg;
    private bool yExceedsNeg;
    private bool zExceedsNeg;
    private IFlagRegisterField MROI = null!;

    // INT_THRESHOLD
    private ushort intThreshold;

    // STATUS_REG
    private IFlagRegisterField overZYX = null!;
    private IFlagRegisterField overZ = null!;
    private IFlagRegisterField overY = null!;
    private IFlagRegisterField overX = null!;
    private IFlagRegisterField drdyZYX = null!;
    private IFlagRegisterField drdyZ = null!;
    private IFlagRegisterField drdyY = null!;
    private IFlagRegisterField drdyX = null!;

    private enum Register : byte
    {
      OFFSET_X_REG_L = 0x45,
      OFFSET_X_REG_H = 0x46,
      OFFSET_Y_REG_L = 0x47,
      OFFSET_Y_REG_H = 0x48,
      OFFSET_Z_REG_L = 0x49,
      OFFSET_Z_REG_H = 0x4A,
      WHO_AM_I = 0x4F,
      CFG_REG_A = 0x60,
      CFG_REG_B = 0x61,
      CFG_REG_C = 0x62,
      INT_CRTL_REG = 0x63,
      INT_SOURCE_REG = 0x64,
      INT_THS_L_REG = 0x65,
      INT_THS_H_REG = 0x66,
      STATUS_REG = 0x67,
      OUTX_L_REG = 0x68,
      OUTX_H_REG = 0x69,
      OUTY_L_REG = 0x6A,
      OUTY_H_REG = 0x6B,
      OUTZ_L_REG = 0x6C,
      OUTZ_H_REG = 0x6D,
      TEMP_OUT_L_REG = 0x6E,
      TEMP_OUT_H_REG = 0x6F,
    }
  }
}

using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
  /// <summary>
  ///  LSM6DSR 6-axis IMU (accelerometer + gyroscope) Renode peripheral.
  /// </summary>
  /// <remarks>
  ///  Implemented:
  ///  <list type="bullet">
  ///    <item>WHO_AM_I, CTRL1_XL, CTRL2_G, CTRL3_C</item>
  ///    <item>Gyroscope and accelerometer output registers</item>
  ///    <item>SPI mode 3, read bit 0x80, IF_INC auto-increment</item>
  ///    <item>Chip select framing driven by a GPIO input</item>
  ///    <item>Monitor-settable acceleration and angular rate</item>
  ///  </list>
  ///
  ///  Not implemented: FIFO, interrupts, temperature, embedded functions,
  ///  self-test, and full-scale switching (sensitivities are fixed at the
  ///  +/-8 g and 2000 dps settings the onboarding firmware configures).
  /// </remarks>
  public class LSM6DSR :
    ISPIPeripheral,
    IGPIOReceiver,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    public LSM6DSR(IMachine machine)
    {
      this.machine = machine;
      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    public void Reset()
    {
      RegistersCollection.Reset();
      address = 0;
      reading = false;
      expectingCommand = true;
    }

    /// <summary>
    ///  Chip select. The STM32 drives nCS as a plain GPIO, and Renode's
    ///  STM32SPI never calls FinishTransmission itself, so the transaction
    ///  boundary has to come from the pin. Active low.
    /// </summary>
    public void OnGPIO(int number, bool value)
    {
      if(number != 0)
      {
        this.Log(LogLevel.Warning, "Unexpected GPIO {0}", number);
        return;
      }

      this.Log(LogLevel.Noisy, "nCS -> {0}", value ? "high" : "low");

      if(value)
      {
        FinishTransmission();
      }
    }

    public byte Transmit(byte data)
    {
      if(expectingCommand)
      {
        reading = (data & ReadBit) != 0;
        address = (byte)(data & AddressMask);
        expectingCommand = false;

        this.Log(LogLevel.Noisy, "Command 0x{0:X2}: {1} at 0x{2:X2}",
                 data, reading ? "read" : "write", address);

        // The device answers during the following byte, not this one.
        return 0x00;
      }

      if(!reading)
      {
        RegistersCollection.Write(address, data);
        this.Log(LogLevel.Noisy, "  stored 0x{0:X2} at 0x{1:X2}", data, address);
        Advance();
        return 0x00;
      }

      return ReadAndAdvance();
    }

    private byte ReadAndAdvance()
    {
      var value = RegistersCollection.Read(address);
      this.Log(LogLevel.Noisy, "  returning 0x{0:X2} from 0x{1:X2}", value, address);
      Advance();
      return value;
    }

    private void Advance()
    {
      if(autoIncrement.Value)
      {
        address++;
      }
    }

    public void FinishTransmission()
    {
      expectingCommand = true;
    }

    private void DefineRegisters()
    {
      const FieldMode R = FieldMode.Read;

      Register.WhoAmI.Define(this, resetValue: DeviceId)
        .WithValueField(0, 8, R, name: "WHO_AM_I");

      Register.Ctrl1Xl.Define(this)
        .WithValueField(0, 8, name: "CTRL1_XL");

      Register.Ctrl2G.Define(this)
        .WithValueField(0, 8, name: "CTRL2_G");

      Register.Ctrl3C.Define(this, resetValue: 0x04)
        .WithFlag(0, name: "SW_RESET")
        .WithReservedBits(1, 1)
        .WithFlag(2, out autoIncrement, name: "IF_INC")
        .WithValueField(3, 5, name: "CTRL3_C_REST");

      DefineOutputRegister(Register.OutXLG, () => GyroRaw(AngularRateX), false);
      DefineOutputRegister(Register.OutXHG, () => GyroRaw(AngularRateX), true);
      DefineOutputRegister(Register.OutYLG, () => GyroRaw(AngularRateY), false);
      DefineOutputRegister(Register.OutYHG, () => GyroRaw(AngularRateY), true);
      DefineOutputRegister(Register.OutZLG, () => GyroRaw(AngularRateZ), false);
      DefineOutputRegister(Register.OutZHG, () => GyroRaw(AngularRateZ), true);

      DefineOutputRegister(Register.OutXLXl, () => AccelRaw(AccelerationX), false);
      DefineOutputRegister(Register.OutXHXl, () => AccelRaw(AccelerationX), true);
      DefineOutputRegister(Register.OutYLXl, () => AccelRaw(AccelerationY), false);
      DefineOutputRegister(Register.OutYHXl, () => AccelRaw(AccelerationY), true);
      DefineOutputRegister(Register.OutZLXl, () => AccelRaw(AccelerationZ), false);
      DefineOutputRegister(Register.OutZHXl, () => AccelRaw(AccelerationZ), true);
    }

    private void DefineOutputRegister(Register register, Func<short> source, bool high)
    {
      register.Define(this)
        .WithValueField(0, 8, FieldMode.Read,
                        valueProviderCallback: _ =>
                        {
                          var raw = (ushort)source();
                          return high ? (byte)(raw >> 8) : (byte)(raw & 0xFF);
                        },
                        name: high ? "OUT_H" : "OUT_L");
    }

    // 0.244 mg / LSB at +/-8 g.
    private static short AccelRaw(double g)
    {
      return Saturate(g * 1000.0 / 0.244);
    }

    // 70 mdps / LSB at 2000 dps.
    private static short GyroRaw(double dps)
    {
      return Saturate(dps * 1000.0 / 70.0);
    }

    private static short Saturate(double value)
    {
      return (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, Math.Round(value)));
    }

    // Set from the Renode monitor or a Robot test, e.g.
    //   sysbus.spi1.imu AccelerationZ 1.0
    public double AccelerationX { get; set; }
    public double AccelerationY { get; set; }
    public double AccelerationZ { get; set; }
    public double AngularRateX { get; set; }
    public double AngularRateY { get; set; }
    public double AngularRateZ { get; set; }

    public ByteRegisterCollection RegistersCollection { get; private set; }

    private readonly IMachine machine;

    private IFlagRegisterField autoIncrement = null!;

    private byte address;
    private bool reading;
    private bool expectingCommand;

    private const byte DeviceId = 0x6B;
    private const byte ReadBit = 0x80;
    private const byte AddressMask = 0x7F;

    private enum Register
    {
      WhoAmI = 0x0F,
      Ctrl1Xl = 0x10,
      Ctrl2G = 0x11,
      Ctrl3C = 0x12,
      OutXLG = 0x22,
      OutXHG = 0x23,
      OutYLG = 0x24,
      OutYHG = 0x25,
      OutZLG = 0x26,
      OutZHG = 0x27,
      OutXLXl = 0x28,
      OutXHXl = 0x29,
      OutYLXl = 0x2A,
      OutYHXl = 0x2B,
      OutZLXl = 0x2C,
      OutZHXl = 0x2D,
    }
  }
}

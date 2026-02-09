using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class ASM330LHBG1 :
    ISPIPeripheral,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    public ASM330LHBG1(IMachine machine)
    {
      this.machine = machine;

      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    public void Reset()
    {
      RegistersCollection.Reset();
      expectingAddress = true;
      currentAddress = 0;
      readTransfer = false;
      autoIncrement = false;
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      if (expectingAddress)
      {
        readTransfer = (data & 0x80) != 0;
        currentAddress = (byte)(data & 0x7F);
        autoIncrement = ((data & 0x40) != 0) || (ifInc?.Value ?? false);
        expectingAddress = false;
        return 0xFF;
      }

      if (readTransfer)
      {
        var value = RegistersCollection.Read(currentAddress);
        if (autoIncrement)
        {
          currentAddress++;
        }
        return value;
      }

      RegistersCollection.Write(currentAddress, data);
      if (autoIncrement)
      {
        currentAddress++;
      }
      return 0xFF;
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      expectingAddress = true;
    }

    public void SetGyroscopeRaw(short x, short y, short z)
    {
      WriteVector(x, gyroXLow, gyroXHigh);
      WriteVector(y, gyroYLow, gyroYHigh);
      WriteVector(z, gyroZLow, gyroZHigh);
      gyroDataReady.Value = true;
    }

    public void SetAccelerationRaw(short x, short y, short z)
    {
      WriteVector(x, accelXLow, accelXHigh);
      WriteVector(y, accelYLow, accelYHigh);
      WriteVector(z, accelZLow, accelZHigh);
      accelDataReady.Value = true;
    }

    public void SetTemperatureRaw(short temp)
    {
      WriteVector(temp, tempLow, tempHigh);
      tempDataReady.Value = true;
    }

    public ByteRegisterCollection RegistersCollection { get; private set; }

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long)Register.FUNC_CFG_ACCESS, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "FUNC_CFG_ACCESS");

      RegistersCollection
        .DefineRegister((long)Register.PIN_CTRL, resetValue: 0x3F)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "PIN_CTRL");

      RegistersCollection
        .DefineRegister((long)Register.FIFO_CTRL1, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "FIFO_CTRL1");

      RegistersCollection
        .DefineRegister((long)Register.FIFO_CTRL2, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "FIFO_CTRL2");

      RegistersCollection
        .DefineRegister((long)Register.FIFO_CTRL3, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "FIFO_CTRL3");

      RegistersCollection
        .DefineRegister((long)Register.FIFO_CTRL4, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "FIFO_CTRL4");

      RegistersCollection
        .DefineRegister((long)Register.INT1_CTRL, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "INT1_CTRL");

      RegistersCollection
        .DefineRegister((long)Register.INT2_CTRL, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "INT2_CTRL");

      RegistersCollection
        .DefineRegister((long)Register.WHO_AM_I, resetValue: 0x6B)
        .WithValueField(0, 8, FieldMode.Read, name: "WHO_AM_I");

      RegistersCollection
        .DefineRegister((long)Register.CTRL1_XL, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL1_XL");

      RegistersCollection
        .DefineRegister((long)Register.CTRL2_G, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL2_G");

      RegistersCollection
        .DefineRegister((long)Register.CTRL3_C, resetValue: 0x04)
        .WithFlag(2, out ifInc, FieldMode.Read | FieldMode.Write, name: "IF_INC")
        .WithReservedBits(0, 2)
        .WithReservedBits(3, 5);

      RegistersCollection
        .DefineRegister((long)Register.CTRL4_C, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL4_C");

      RegistersCollection
        .DefineRegister((long)Register.CTRL5_C, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL5_C");

      RegistersCollection
        .DefineRegister((long)Register.CTRL6_C, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL6_C");

      RegistersCollection
        .DefineRegister((long)Register.CTRL7_G, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL7_G");

      RegistersCollection
        .DefineRegister((long)Register.CTRL8_XL, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL8_XL");

      RegistersCollection
        .DefineRegister((long)Register.CTRL9_XL, resetValue: 0xE0)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL9_XL");

      RegistersCollection
        .DefineRegister((long)Register.CTRL10_C, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "CTRL10_C");

      RegistersCollection
        .DefineRegister((long)Register.STATUS_REG, resetValue: 0x00)
        .WithFlag(0, out accelDataReady, FieldMode.Read, name: "XLDA")
        .WithFlag(1, out gyroDataReady, FieldMode.Read, name: "GDA")
        .WithFlag(2, out tempDataReady, FieldMode.Read, name: "TDA")
        .WithReservedBits(3, 5);

      RegistersCollection
        .DefineRegister((long)Register.OUT_TEMP_L, resetValue: 0x00)
        .WithValueField(0, 8, out tempLow, FieldMode.Read, name: "OUT_TEMP_L");

      RegistersCollection
        .DefineRegister((long)Register.OUT_TEMP_H, resetValue: 0x00)
        .WithValueField(0, 8, out tempHigh, FieldMode.Read, name: "OUT_TEMP_H");

      RegistersCollection
        .DefineRegister((long)Register.OUTX_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroXLow, FieldMode.Read, name: "OUTX_L_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTX_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroXHigh, FieldMode.Read, name: "OUTX_H_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTY_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroYLow, FieldMode.Read, name: "OUTY_L_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTY_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroYHigh, FieldMode.Read, name: "OUTY_H_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTZ_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroZLow, FieldMode.Read, name: "OUTZ_L_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTZ_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroZHigh, FieldMode.Read, name: "OUTZ_H_G");

      RegistersCollection
        .DefineRegister((long)Register.OUTX_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelXLow, FieldMode.Read, name: "OUTX_L_A");

      RegistersCollection
        .DefineRegister((long)Register.OUTX_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelXHigh, FieldMode.Read, name: "OUTX_H_A");

      RegistersCollection
        .DefineRegister((long)Register.OUTY_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelYLow, FieldMode.Read, name: "OUTY_L_A");

      RegistersCollection
        .DefineRegister((long)Register.OUTY_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelYHigh, FieldMode.Read, name: "OUTY_H_A");

      RegistersCollection
        .DefineRegister((long)Register.OUTZ_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelZLow, FieldMode.Read, name: "OUTZ_L_A");

      RegistersCollection
        .DefineRegister((long)Register.OUTZ_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelZHigh, FieldMode.Read, name: "OUTZ_H_A");
    }

    private static void WriteVector(short value, IValueRegisterField low, IValueRegisterField high)
    {
      unchecked
      {
        low.Value = (byte)(value & 0xFF);
        high.Value = (byte)((value >> 8) & 0xFF);
      }
    }

    private IMachine machine;
    private bool expectingAddress;
    private bool readTransfer;
    private bool autoIncrement;
    private byte currentAddress;

    private IFlagRegisterField ifInc = null!;
    private IFlagRegisterField accelDataReady = null!;
    private IFlagRegisterField gyroDataReady = null!;
    private IFlagRegisterField tempDataReady = null!;

    private IValueRegisterField tempLow = null!;
    private IValueRegisterField tempHigh = null!;
    private IValueRegisterField gyroXLow = null!;
    private IValueRegisterField gyroXHigh = null!;
    private IValueRegisterField gyroYLow = null!;
    private IValueRegisterField gyroYHigh = null!;
    private IValueRegisterField gyroZLow = null!;
    private IValueRegisterField gyroZHigh = null!;
    private IValueRegisterField accelXLow = null!;
    private IValueRegisterField accelXHigh = null!;
    private IValueRegisterField accelYLow = null!;
    private IValueRegisterField accelYHigh = null!;
    private IValueRegisterField accelZLow = null!;
    private IValueRegisterField accelZHigh = null!;

    private enum Register
    {
      FUNC_CFG_ACCESS = 0x01,
      PIN_CTRL = 0x02,
      FIFO_CTRL1 = 0x07,
      FIFO_CTRL2 = 0x08,
      FIFO_CTRL3 = 0x09,
      FIFO_CTRL4 = 0x0A,
      INT1_CTRL = 0x0D,
      INT2_CTRL = 0x0E,
      WHO_AM_I = 0x0F,
      CTRL1_XL = 0x10,
      CTRL2_G = 0x11,
      CTRL3_C = 0x12,
      CTRL4_C = 0x13,
      CTRL5_C = 0x14,
      CTRL6_C = 0x15,
      CTRL7_G = 0x16,
      CTRL8_XL = 0x17,
      CTRL9_XL = 0x18,
      CTRL10_C = 0x19,
      STATUS_REG = 0x1E,
      OUT_TEMP_L = 0x20,
      OUT_TEMP_H = 0x21,
      OUTX_L_G = 0x22,
      OUTX_H_G = 0x23,
      OUTY_L_G = 0x24,
      OUTY_H_G = 0x25,
      OUTZ_L_G = 0x26,
      OUTZ_H_G = 0x27,
      OUTX_L_A = 0x28,
      OUTX_H_A = 0x29,
      OUTY_L_A = 0x2A,
      OUTY_H_A = 0x2B,
      OUTZ_L_A = 0x2C,
      OUTZ_H_A = 0x2D,
    }
  }
}

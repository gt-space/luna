using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class LIS2MDLTR :
    ISPIPeripheral,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    public LIS2MDLTR(IMachine machine)
    {
      this.machine = machine;

      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    // Called when the sensor is reset, such as a RESET pin being pulled active.
    public void Reset()
    {
      RegistersCollection.Reset();
    }

    //Inverts high and low bytes
    public void Invert()
    {
      byte temp = outX & 0xFF00;
      outX = (byte)((outX >> 8) | temp);
      temp = outY & 0xFF00;
      outY = (byte)((outY >> 8) | temp);
      temp = outZ & 0xFF00;
      outZ = (byte)((outZ >> 8) | temp);
      temp = tempOut & 0xFF00;
      tempOut = (byte)((tempOut >> 8) | temp);
    }

    //If ctrl reg allows, drive the value through GPIO
    public void driveDrdy()
    {
      bool signal = (enableInterrupt.Value && intOnPin.Value) || (drdyZYX.Value && drdyOnPin.Value);

      Interrupt.Set(signal);
    }

    //Checks if the inputted data exceeds the set threshold
    public bool checkExceeds(short data, short axis)
    {
      bool result = Math.abs((int)data) > intThreshold;
      if (result) {
        switch (axis)
          {
            case outZ:
              if (!enableZIE.Value)
              {
                return false;
              }
              (sbyte)data > 0 ? ZExceedsPos.Value = 1 : ZExceedsNeg.Value = 1;
              break;
            case outY:
              if (enableYIE.Value)
              {
                return false;
              }
              (sbyte)data > 0 ? YExceedsPos.Value = 1 : YExceedsNeg.Value = 1;
              break;
            case outX:
              if (enableXIE.Value)
              {
                return false;
              }
              (sbyte)data > 0 ? XExceedsPos.Value = 1 : XExceedsNeg.Value = 1;
              break;
          }
        intOnPin.Value = polarity.Value && enableInterrupt.Value;
      }
      return result;
    }

    //Writes data (should be from actual device, currently dummy data)
    public void writeData(short data, short axis)
    {
      checkExceeds((sbyte)data, axis);
      if (offsetCancellation.Value && !(!offsetCancellationOneShot.Value && isSingle))
      {
        switch (axis)
        {
          case outZ:
            data += offZ;
            break;
          case outY:
            data += offY;
            break;
          case outX:
            data += offX;
            break;
        }
        if (interruptChecking.Value)
        {
          checkExceeds((sbyte)data, axis);
        }
      }
      switch (axis)
      {
        case outZ:
          data == axis.Value ? overZ.Value = 1 : overZ.Value = 0;
          break;
        case outY:
          if (enableYIE.Value)
          data == axis.Value ? overY.Value = 1 : overY.Value = 0;
          break;
        case outX:
          data == axis.Value ? overX.Value = 1 : overX.Value = 0;
          break;
      }
      overZYX.Value = overZ.Value | overY.Value | overX.Value;
      axis.Value = data;
      if (isContinuous)
      {
        drdyOnPin.Value = 1;
        driveDrdy();
      } else
      {
        drdyOnPin.Value = 1;
        driveDrdy();
        setMode((ulong)11);
      }
    }

    //Sets the mode
    public void setMode(ulong value)
    {
      operationMode.Value = value;
      if(value == 01)
      {
        isSingle = true;
        isContinuous = false;
      }
    }

    // Defines registers and their sub-fields according to the datasheet.
    // Make sure that you account for default reset values.
    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.OFFSET_X_REG_L)
        .withValueField(0, 8, FieldMode.Read, name: "offXLSB",
          valueProviderCallback: _ => (byte)(offX & 0xFF),
          writeCallback: (_, newValue) =>
          {
            offX = (short)((offX & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_X_REG_H)
        .withValueField(0, 8, FieldMode.Read, name: "offXMSB",
          valueProviderCallback: _ => (byte)(offX >> 8),
          writeCallback: (_, newValue) =>
          {
            offX = (short)((offX & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Y_REG_L)
        .withValueField(0, 8, FieldMode.Read, name: "offYLSB",
          valueProviderCallback: _ => (byte)(offY & 0xFF),
          writeCallback: (_, newValue) =>
          {
            offY = (short)((offY & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Y_REG_H)
        .withValueField(0, 8, FieldMode.Read, name: "offXMSB",
          valueProviderCallback: _ => (byte)(offY >> 8),
          writeCallback: (_, newValue) =>
          {
            offY = (short)((offY & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
      .DefineRegister((long) Register.OFFSET_Z_REG_L)
      .withValueField(0, 8, FieldMode.Read, name: "offZLSB",
        valueProviderCallback: _ => (byte)(offZ & 0xFF),
        writeCallback: (_, newValue) =>
        {
          offZ = (short)((offZ & 0xFF00) | newValue);
        }
      );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Z_REG_H)
        .withValueField(0, 8, FieldMode.Read, name: "offZMSB",
          valueProviderCallback: _ => (byte)(offZ >> 8),
          writeCallback: (_, newValue) =>
          {
            offZ = (short)((offZ & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_A, resetValue: 0x03)
        .withFlag(7, out tempCompIsOn, name: "COMP_TEMP_EN")
        .withFlag(6, name: "REBOOT",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .withFlag(5, name: "SOFT_RST",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .withFlag(4, out lowPower, name: "LP")
        .withValueField(2, 2, out outputDataRate, name: "ODR")
        .withValueField(0, 2, name: "MD",
          valueProviderCallback: _ => operationMode.Value,
          writeCallback: (_, value) =>
          {
            setMode(value);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_B, resetValue: 0x00)
        .withFlag(4, out offsetCancellationOneShot, name: "OFF_CANC_ONE_SHOT")
        .withFlag(3, out interruptChecking, name: "INT_on_DataOFF")
        .withFlag(2, out pulseFrequency, name: "Set_FREQ")
        .withFlag(1, out offsetCancellation, name: "OFF_CANC")
        .withFlag(0, out lowpassFilter, name: "LPF");

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_C, resetValue: 0x00)
        .withFlag(6, name: "INT_on_PIN",
          valueProviderCallback: _ =>
          {
            intOnPin.Value;
          },
          writeCallback(_, value) =>
          {
            intOnPin.Value = value;
            driveDrdy();
          }
        )
        /*
        If 1, the INTERRUPT signal (INT bit in INT_SOURCE_REG (64h)) is driven to the INT/DRDY pin.
        The INT/DRDY pin is configured in push-pull output mode.
        */
        .withFlag(5, out I2CDisabled, name: "I2C_DIS")
        .withFlag(4, out blockDataUpdate, name: "BDU")
        .withFlag(3, name: "BLE",
          valueProviderCallback: _ => byteInversion.Value,
          writeCallback(_, value) =>
          {
            byteInversion.Value = value;
            if (value)
            {
              Invert();
            }
          }
        )
        .withFlag(2, out enableSDO, name: "4WSPI")
        .withFlag(1, out selfTestOn, name: "Self_test")
        /*
        If 1, the data-ready signal (Zyxda bit in STATUS_REG (67h)) is driven on the INT/DRDY pin.
        The INT/DRDY pin is configured in push-pull output mode
        */
        .withFlag(0, name: "DRDY_on_PIN",
          valueProviderCallback: _ => drdyOnPin.Value,
          writeCallback(_, value) =>
          {
            drdyOnPin.Value = value;
            driveDrdy();
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_CRTL_REG, resetValue: 0xE0)
        .withFlag(7, out enableXIE, name: "XIEN")
        .withFlag(6, out enableYIE, name: "YIEN")
        .withFlag(5, out enableZIE, name: "XIEN")
        /*
        Controls the polarity of the INT bit (INT_SOURCE_REG (64h)) when an interrupt occurs. Default: 0
        If IEA = 0, then INT = 0 signals an interrupt
        If IEA = 1, then INT = 1 signals an interrupt
        */
        .withFlag(2, out polarity, name: "IEA")
        /*
        Controls whether the INT bit (INT_SOURCE_REG (64h)) is latched or pulsed. Default: 0
        If IEL = 0, then INT is pulsed.
        If IEL = 1, then INT is latched.
        Once latched, INT remains in the same state until INT_SOURCE_REG (64h) is read
        */
        .withFlag(1, out latched, name: "IEL")
        /*
        Enables the interrupt. When set, enables the generation of the interrupt. The INT bit is in INT_SOURCE_REG (64h).
        */
        .withFlag(0, out enableInterrupt, name: "IEN");


        /*
        Problem here with latching and interrupt
        */
      RegistersCollection
        .DefineRegister((long) Register.INT_SOURCE_REG)
        .withFlag(7, out XExceedsPos, mode: FieldMode.Read, name: "P_TH_S_X")
        .withFlag(6, out YExceedsPos, mode: FieldMode.Read, name: "P_TH_S_Y")
        .withFlag(5, out ZExceedsPos, mode: FieldMode.Read, name: "P_TH_S_Z")
        .withFlag(4, out XExceedsNeg, mode: FieldMode.Read, name: "N_TH_S_X")
        .withFlag(3, out YExceedsNeg, mode: FieldMode.Read, name: "N_TH_S_Y")
        .withFlag(2, out ZExceedsNeg, mode: FieldMode.Read, name: "N_TH_S_Z")
        .withFlag(1, out MROI, mode: FieldMode.Read, name: "MROI")
        .withFlag(0, mode: FieldMode.Read, name: "INT",
          valueProviderCallback: _ =>
          {
            if (!enableInterrupt.Value)
            {
              0;
            } else if (polarity.Value || (!(value || polarity.Value)))
            {
              isInterrupt = true;
            } else
            {
              //Error handling?
            }
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_THS_L_REG, resetValue: 0x00)
        .withValueField(0, 8, name: "tLSB",
          valueProviderCallback: _ => (byte)(intThreshold & 0xFF),
          writeCallback: (_, newValue) =>
          {
            intThreshold = (ushort)((intThreshold & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_THS_H_REG, resetValue: 0x00)
        .withValueField(0, 8, name: "tMSB",
          valueProviderCallback: _ => (byte)(intThreshold >> 8),
          writeCallback: (_, newValue) =>
          {
            intThreshold = (ushort)((intThreshold & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.STATUS_REG)
        .withValueField(7, out overZYX, FieldMode.Read, name: "Zyxor")
        .withValueField(6, out overZ, FieldMode.Read, name: "zor")
        .withValueField(5, out overY, FieldMode.Read, name: "yor")
        .withValueField(4, out overX, FieldMode.Read, name: "xor")
        .withValueField(3, out drdyZYX, FieldMode.Read, name: "Zyxda")
        .withValueField(2, out drdyZ, FieldMode.Read, name: "zda")
        .withValueField(1, out drdyY, FieldMode.Read, name: "yda")
        .withValueField(0, out drdyX, FieldMode.Read, name: "xda");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outXLSB",
          valueProviderCallback: _ => {
            drdyX.Value = false;
            return (byte)(outX & 0xFF);
          },
          writeCallback: (_, newValue) =>
          {
            outX = (short)((outX & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTX_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outXMSB",
          valueProviderCallback: _ => {
            drdyX.Value = false;
            return (byte)(outX >> 8);
          },
          writeCallback: (_, newValue) =>
          {
            outX = (short)((outX & 0x00FF) | (newValue << 8));
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTY_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outYLSB",
          valueProviderCallback: _ => {
            drdyY.Value = false;
            return (byte)(outY & 0xFF);
            },
          writeCallback: (_, newValue) =>
          {
            outY = (short)((outY & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTY_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outYMSB",
          valueProviderCallback: _ => {
            drdyY.Value = false;
            (byte)(outY >> 8);
          },
          writeCallback: (_, newValue) =>
          {
            outY = (short)((outY & 0x00FF) | (newValue << 8));
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTZ_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outZLSB",
          valueProviderCallback: _ => {
            drdyZ.Value = false;
            (byte)(outZ & 0xFF);
            },
          writeCallback: (_, newValue) =>
          {
            outZ = (short)((outZ & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outZMSB",
          valueProviderCallback: _ => {
            drdyZ.Value = false;
            (byte)(outZ >> 8);
          },
          writeCallback: (_, newValue) =>
          {
            outZ = (short)((outZ & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "tempLSB",
          valueProviderCallback: _ => (byte)(tempOut & 0xFF),
          writeCallback: (_, newValue) =>
          {
            tempOut = (short)((tempOut & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "tempMSB",
          valueProviderCallback: _ => (byte)(tempOut >> 8),
          writeCallback: (_, newValue) =>
          {
            tempOut = (short)((newValue << 8) | (tempOut & 0x00FF));
          }
        );
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      Console.WriteLine($"Received SPI byte: {data}");

      if (isFirstByte)
      {
        isFirstByte = false;
        reading = (data & 0x80) != 0;
        address = (byte)(data & 0x7F);
        Console.WriteLine($"Received first byte: {data}\nReading: {reading}\nAddress: {address}");
        return 0xFF;
      }

      if (!Register.IsDefined(typeof(Register), example1))
      {
        Console.WriteLine($"Invalid address");
      }

      else if (reading)
      {
        Console.WriteLine($"Reading from {address}");
        byte output = Read(data);
        Console.WriteLine($"Reading output is: {output}");
        return output;
      } else
      {
        Console.WriteLine($"Writing: {data}\nAddress: {address}");
        Write(data);
        return 0xFF;
      }
    }

    //Called if RW byte is 1, outputs data from current address
    public byte Read(byte data)
    {
      var buffer = new byte();
      buffer = RegistersCollection.Read((byte)address);
      address = address++;
      if (address == (byte)0x70)
      {
        address = (byte)0x45;
      }
      if (address == (byte)0x4b)
      {
        address = (byte)0x4f;
      }
      if (address = 0x50)
      {
        address = (byte)0x60;
      }
      if (address = 0x67)
      {
        XExceedsPos.Value = false;
        XExceedsNeg.Value = false;
        YExceedsPos.Value = false;
        YExceedsNeg.Value = false;
        ZExceedsPos.Value = false;
        ZExceedsNeg.Value = false;
        MROI.Value = false;
        isInterrupt.Value = false;
      }
      return buffer;
    }

    //Called if RW byte is 0, writes data to current register
    public void Write(byte data)
    {
      RegistersCollection.Write((byte)address, data);
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      Console.WriteLine("SPI transmission finished (CS high)");
      isFirstByte = true;
    }


    //The following uses dummy data to implement offset calculation behavior
    short x = 0;
    short y = 0;
    short z = 0;

    public void dummyData()
    {
      dummyData(
        0b0000000000000001, 0b0000000000000001, 0b0000000000000001);
    }

    //simulates device axis readings
    public void dummyData(short x, short y, short z)
    {
      writeData((sbyte)x, outX);
      drdyX.Value = true;
      writeData((sbyte)y, outY);
      drdyY.Value = true;
      writeData((sbyte)z, outZ);
      drdyZ.Value = true;
      drdyZYX.Value = true;

      // if (offsetCancellation && !(!offsetCancellationOneShot.Value && isSingle))
      // {
      //   outX += offX;
      //   outY += offY;
      //   outZ += offZ;

      //   if (interruptChecking.Value)
      //   {
      //     checkExceeds(outX);
      //     checkExceeds(outY);
      //     checkExceeds(outZ);
      //   }
      // }
    }

    // Maps the register map of the chip onto a virtual register collection.
    public ByteRegisterCollection RegistersCollection { get; private set; }
    //GPIO Object
    public GPIO Interrupt { get; }

    private IMachine machine;

    // Fields derived from fields in the register collection.
    //
    // The `null!` default initialization is applied because they will all be
    // set in the `DefineRegisters` method, which is called by the constructor.

    //Offset:
    private short offX = 0;
    private short offY = 0;
    private short offZ = 0;

    //CFG_REG_A:
    private IFlagRegisterField tempCompIsOn = null!;
    private IFlagRegisterField lowPower = null!;
    private IFlagRegisterField outputDataRate = null!;
    private IFlagRegisterField operationMode = null!;
    private bool isSingle = false;
    private bool isContinuous = true;

    //CFG_REG_B:
    private IFlagRegisterField offsetCancellationOneShot = null!;
    private IFlagRegisterField interruptChecking = null!;
    private IFlagRegisterField pulseFrequency = null!;
    private IFlagRegisterField offsetCancellation = null!;
    private IFlagRegisterField lowpassFilter = null!;

    //CFG_REG_C:
    private IFlagRegisterField intOnPin = null!;
    private IFlagRegisterField I2CDisabled = null!;
    private IFlagRegisterField blockDataUpdate = null!;
    private IFlagRegisterField byteInversion = null!;
    private IFlagRegisterField enableSDO = null!;
    private IFlagRegisterField selfTestOn = null!;
    private IFlagRegisterField drdyOnPin = null!;

    //Interruption Flags
    private IFlagRegisterField polarity = null!;
    private IFlagRegisterField latched = null!;
    private IFlagRegisterField enableInterrupt = null!;

    //INT_CTRL_REG:
    private IFlagRegisterField enableXIE = null!;
    private IFlagRegisterField enableYIE = null!;
    private IFlagRegisterField enableZIE = null!;

    //INT_SOURCE_REG:
    private IFlagRegisterField XExceedsPos = null!;
    private IFlagRegisterField YExceedsPos = null!;
    private IFlagRegisterField ZExceedsPos = null!;
    private IFlagRegisterField XExceedsNeg = null!;
    private IFlagRegisterField YExceedsNeg = null!;
    private IFlagRegisterField ZExceedsNeg = null!;
    private IFlagRegisterField MROI = null!;
    private IFlagRegisterField isInterrupt = null!;

    //INT_THRESHOLD:
    private ushort intThreshold = 0;

    //STATUS_REG
    private IFlagRegisterField overZYX = null!;
    private IFlagRegisterField overZ = null!;
    private IFlagRegisterField overY = null!;
    private IFlagRegisterField overX = null!;
    private IFlagRegisterField drdyZYX = null!;
    private IFlagRegisterField drdyZ = null!;
    private IFlagRegisterField drdyY = null!;
    private IFlagRegisterField drdyX = null!;

    //OUTPUT
    private short outX = 0;
    private short outY = 0;
    private short outZ = 0;
    private short tempOut = 0;

    //SPI
    private bool isFirstByte = true;
    private bool reading = false;
    private byte address = 0;

    // Retrieved directly from the register map in the datasheet of the chip.
      private enum Register
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

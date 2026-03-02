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
      InstantiateRegisters();
      Reset();
      Interrupt = new GPIO();
      dummyData();
    }

    // Called when the sensor is reset, such as a RESET pin being pulled active.
    public void Reset()
    {
      Console.WriteLine($"Received reset command; Resetting...");
      RegistersCollection.Reset();
    }

    //Inverts high and low bytes
    public void Invert()
    {
      Console.WriteLine($"Received invert command; Inverting...");
      short temp = (short)(outX & 0xFF00);
      outX = (short)(outX << 8 | (byte)temp);
      temp = (short)(outY & 0xFF00);
      outY = (short)(outY << 8 | (byte)temp);
      temp = (short)(outZ & 0xFF00);
      outZ = (short)(outZ << 8 | (byte)temp);
      temp = (short)(tempOut & 0xFF00);
      tempOut = (short)(tempOut << 8 | (byte)temp);
    }

    //If ctrl reg allows, drive the value through GPIO
    public void driveDrdy()
    {
      Console.WriteLine($"Received drive DRDY command");
      Console.WriteLine($"Interrupt: {enableInterrupt.Value && intOnPin.Value}\nDRDY: {drdyZYX.Value && drdyOnPin.Value}");
      bool signal = (enableInterrupt.Value && intOnPin.Value) || (drdyZYX.Value && drdyOnPin.Value);
      Interrupt.Set(signal);
      Console.WriteLine($"Driving {signal}");
    }

    //Checks if the inputted data exceeds the set threshold
    public bool checkExceeds(short data, string axis)
    {
      // if (!enableInterrupt.Value)
      // {
      //   Console.WriteLine($"Interrupt Disabled in INT_CTRL_REG");
      //   return false;
      // }
      bool result = Math.Abs((int)data) > intThreshold;
      if (result) {
        Console.WriteLine($"{enableXIE.Value} {enableYIE.Value} {enableZIE.Value}");
        if (axis == "z")
        {
          if (enableZIE.Value)
          {
            if (data > 0)
            {
              ZExceedsPos = true;
            }
            else
            {
              ZExceedsNeg = true;
            }
          }
        }
        else if (axis == "y")
        {
          if (enableYIE.Value)
          {
            if (data > 0)
            {
              YExceedsPos = true;
            }
            else
            {
              YExceedsNeg = true;
            }
          }
        }
        else
        {
          if (enableXIE.Value)
          {
            if (data > 0)
            {
              XExceedsPos = true;
            }
            else
            {
              XExceedsNeg = true;
            }
          }
        }
        intOnPin.Value = polarity.Value && enableInterrupt.Value;
      }
      return result;
    }

    //Writes data (should be from actual device, currently dummy data)
    public void writeData(short data, ref short axis, string axisRef)
    {
      string a = axisRef;
      checkExceeds(data, a);
      if (offsetCancellation.Value && !(!offsetCancellationOneShot.Value && isSingle))
      {
        if (a == "z")
        {
          data += offZ;
        }
        else if (a == "y")
        {
          data += offY;
        }
        else if (a == "x")
        {
          data += offX;
        }

        if (interruptChecking.Value)
        {
          checkExceeds(data, a);
        }
      }

      if (a == "z")
      {
        overZ.Value = data != axis ? true : false;
      }
      else if (a == "y")
      {
        overY.Value = data != axis ? true : false;
      }
      else if (a == "x")
      {
        overX.Value = data != axis ? true : false;
      }

      overZYX.Value = overZ.Value | overY.Value | overX.Value;
      axis = data;
      if (isContinuous)
      {
        drdyOnPin.Value = true;
        driveDrdy();
      } else
      {
        drdyOnPin.Value = true;
        driveDrdy();
        setMode(3);
      }
    }

    //Sets the mode
    public void setMode(ulong value)
    {
      operationMode = value;
      if(value == 1)
      {
        isSingle = true;
        isContinuous = false;
      } else
      {
        isSingle = false;
      }

      if (isContinuous)
      {
        drdyOnPin.Value = true;
        driveDrdy();
      }
      else if (isSingle)
      {
        drdyOnPin.Value = true;
        driveDrdy();
        operationMode = 3;
        setMode(3);
      }
    }

    // Defines registers and their sub-fields according to the datasheet.
    // Make sure that you account for default reset values.
    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.WHO_AM_I, resetValue: 0x40)
        .WithValueField(0, 8, FieldMode.Read, name: "whoAmI");

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_X_REG_L, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read, name: "offXLSB",
          valueProviderCallback: _ => (byte)(offX & 0xFF),
          writeCallback: (_, newValue) =>
          {
            offX = (short)((offX & 0xFF00) | (byte)newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_X_REG_H, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read, name: "offXMSB",
          valueProviderCallback: _ => (byte)(offX >> 8),
          writeCallback: (_, newValue) =>
          {
            offX = (short)((offX & 0x00FF) | ((byte)newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Y_REG_L, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read, name: "offYLSB",
          valueProviderCallback: _ => (byte)(offY & 0xFF),
          writeCallback: (_, newValue) =>
          {
            offY = (short)((offY & 0xFF00) | (byte)newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Y_REG_H, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read, name: "offXMSB",
          valueProviderCallback: _ => (byte)(offY >> 8),
          writeCallback: (_, newValue) =>
          {
            offY = (short)((offY & 0x00FF) | ((byte)newValue << 8));
          }
        );

      RegistersCollection
      .DefineRegister((long) Register.OFFSET_Z_REG_L, resetValue: 0x00)
      .WithValueField(0, 8, FieldMode.Read, name: "offZLSB",
        valueProviderCallback: _ => (byte)(offZ & 0xFF),
        writeCallback: (_, newValue) =>
        {
          offZ = (short)((offZ & 0xFF00) | (byte)newValue);
        }
      );

      RegistersCollection
        .DefineRegister((long) Register.OFFSET_Z_REG_H, resetValue: 0x00)
        .WithValueField(0, 8, FieldMode.Read, name: "offZMSB",
          valueProviderCallback: _ => (byte)(offZ >> 8),
          writeCallback: (_, newValue) =>
          {
            offZ = (short)((offZ & 0x00FF) | ((byte)newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_A, resetValue: 0x03)
        .WithFlag(7, out tempCompIsOn, name: "COMP_TEMP_EN")
        .WithFlag(6, name: "REBOOT",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .WithFlag(5, name: "SOFT_RST",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reset();
            }
          }
        )
        .WithFlag(4, out lowPower, name: "LP")
        .WithValueField(2, 2, name: "ODR")
        .WithValueField(0, 2, name: "MD",
          writeCallback: (_, value) =>
          {
            setMode(value);
          },
          valueProviderCallback: _ => operationMode
        );

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_B, resetValue: 0x00)
        .WithReservedBits(5, 3)
        .WithFlag(4, out offsetCancellationOneShot, name: "OFF_CANC_ONE_SHOT")
        .WithFlag(3, out interruptChecking, name: "INT_on_DataOFF")
        .WithFlag(2, out pulseFrequency, name: "Set_FREQ")
        .WithFlag(1, out offsetCancellation, name: "OFF_CANC")
        .WithFlag(0, out lowpassFilter, name: "LPF");

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_C, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithFlag(6, out intOnPin, name: "INT_on_PIN",
          writeCallback: (_, value) =>
          {
            driveDrdy();
          }
        )
        /*
        If 1, the INTERRUPT signal (INT bit in INT_SOURCE_REG (64h)) is driven to the INT/DRDY pin.
        The INT/DRDY pin is configured in push-pull output mode.
        */
        .WithFlag(5, out I2CDisabled, name: "I2C_DIS")
        .WithFlag(4, out blockDataUpdate, name: "BDU")
        .WithFlag(3, name: "BLE",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Invert();
            }
          }
        )
        .WithFlag(2, out enableSDO, name: "4WSPI")
        .WithFlag(1, out selfTestOn, name: "Self_test")
        /*
        If 1, the data-ready signal (Zyxda bit in STATUS_REG (67h)) is driven on the INT/DRDY pin.
        The INT/DRDY pin is configured in push-pull output mode
        */
        .WithFlag(0, out drdyOnPin, name: "DRDY_on_PIN",
          writeCallback: (_, value) =>
          {
            driveDrdy();
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_CRTL_REG, resetValue: 0xE0)
        .WithFlag(7, out enableXIE, name: "XIEN")
        .WithFlag(6, out enableYIE, name: "YIEN")
        .WithFlag(5, out enableZIE, name: "ZIEN")
        .WithReservedBits(3, 2, 0x00)
        /*
        Controls the polarity of the INT bit (INT_SOURCE_REG (64h)) when an interrupt occurs. Default: 0
        If IEA = 0, then INT = 0 signals an interrupt
        If IEA = 1, then INT = 1 signals an interrupt
        */
        .WithFlag(2, out polarity, name: "IEA")
        /*
        Controls whether the INT bit (INT_SOURCE_REG (64h)) is latched or pulsed. Default: 0
        If IEL = 0, then INT is pulsed.
        If IEL = 1, then INT is latched.
        Once latched, INT remains in the same state until INT_SOURCE_REG (64h) is read
        */
        .WithFlag(1, out latched, name: "IEL")
        /*
        Enables the interrupt. When set, enables the generation of the interrupt. The INT bit is in INT_SOURCE_REG (64h).
        */
        .WithFlag(0, out enableInterrupt, name: "IEN");



      RegistersCollection
        .DefineRegister((long) Register.INT_SOURCE_REG)
        .WithFlag(7, mode: FieldMode.Read, name: "P_TH_S_X",
          valueProviderCallback: _ => enableXIE.Value && XExceedsPos
        )
        .WithFlag(6, mode: FieldMode.Read, name: "P_TH_S_Y",
          valueProviderCallback: _ => enableYIE.Value && YExceedsPos
        )
        .WithFlag(5, mode: FieldMode.Read, name: "P_TH_S_Z",
          valueProviderCallback: _ => enableZIE.Value && ZExceedsPos
        )
        .WithFlag(4, mode: FieldMode.Read, name: "N_TH_S_X",
          valueProviderCallback: _ => enableXIE.Value && XExceedsNeg
        )
        .WithFlag(3, mode: FieldMode.Read, name: "N_TH_S_Y",
          valueProviderCallback: _ => enableYIE.Value && YExceedsNeg
        )
        .WithFlag(2, mode: FieldMode.Read, name: "N_TH_S_Z",
          valueProviderCallback: value => enableZIE.Value && ZExceedsNeg
        )
        .WithFlag(1, out MROI, mode: FieldMode.Read, name: "MROI")
        .WithFlag(0, out isInterrupt, mode: FieldMode.Read, name: "INT",
          valueProviderCallback: _ =>
          {
            if (!enableInterrupt.Value) return false;
            bool triggered = (enableXIE.Value && (XExceedsPos || XExceedsNeg)) ||
                             (enableYIE.Value && (YExceedsPos || YExceedsNeg)) ||
                             (enableZIE.Value && (ZExceedsPos|| ZExceedsNeg));
            return triggered ? polarity.Value : !polarity.Value;
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_THS_L_REG, resetValue: 0x00)
        .WithValueField(0, 8, name: "tLSB",
          valueProviderCallback: _ => (byte)(intThreshold & 0xFF),
          writeCallback: (_, newValue) =>
          {
            intThreshold = (ushort)((intThreshold & 0xFF00) | (byte)newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.INT_THS_H_REG, resetValue: 0x00)
        .WithValueField(0, 8, name: "tMSB",
          valueProviderCallback: _ => (byte)(intThreshold >> 8),
          writeCallback: (_, newValue) =>
          {
            intThreshold = (ushort)((intThreshold & 0x00FF) | (byte)(newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.STATUS_REG)
        .WithFlag(7, out overZYX, FieldMode.Read, name: "Zyxor")
        .WithFlag(6, out overZ, FieldMode.Read, name: "zor")
        .WithFlag(5, out overY, FieldMode.Read, name: "yor")
        .WithFlag(4, out overX, FieldMode.Read, name: "xor")
        .WithFlag(3, out drdyZYX, FieldMode.Read, name: "Zyxda")
        .WithFlag(2, out drdyZ, FieldMode.Read, name: "zda")
        .WithFlag(1, out drdyY, FieldMode.Read, name: "yda")
        .WithFlag(0, out drdyX, FieldMode.Read, name: "xda");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_L_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outXLSB",
          valueProviderCallback: _ => {
            drdyX.Value = false;
            return (byte)(outX & 0xFF);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTX_H_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outXMSB",
          valueProviderCallback: _ => {
            drdyX.Value = false;
            return (byte)(outX >> 8);
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTY_L_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outYLSB",
          valueProviderCallback: _ => {
            drdyY.Value = false;
            return (byte)(outY & 0xFF);
            }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTY_H_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outYMSB",
          valueProviderCallback: _ => {
            drdyY.Value = false;
            return (byte)(outY >> 8);
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTZ_L_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outZLSB",
          valueProviderCallback: _ => {
            drdyZ.Value = false;
            return (byte)(outZ & 0xFF);
            }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_H_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "outZMSB",
          valueProviderCallback: _ => {
            drdyZ.Value = false;
            return (byte)(outZ >> 8);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_L_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "tempLSB",
          valueProviderCallback: _ => (byte)(tempOut & 0xFF)
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_H_REG)
        .WithValueField(0, 8, FieldMode.Read, name: "tempMSB",
          valueProviderCallback: _ => (byte)(tempOut >> 8)
        );
    }

    public void InstantiateRegisters()
    {
      //INT_SOURCE_REG;
      MROI.Value = true;
      isInterrupt.Value = false;

      //STATUS_REG
      overZYX.Value = false;
      overZ.Value = false;
      overY.Value = false;
      overX.Value = false;
      drdyZYX.Value = true;
      drdyZ.Value = true;
      drdyY.Value = true;
      drdyX.Value = true;
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      Console.WriteLine($"Received SPI byte: 0x{data:X2}");
      if (isFirstByte)
      {
        isFirstByte = false;
        reading = (data & 0x80) != 0;
        address = (byte)(data & 0x7F);
        Console.WriteLine($"Received first byte: 0x{data:X2}\nReading: {reading}\nAddress: 0x{address:X2}");
        if (!Enum.IsDefined(typeof(Register), address))
        {
          Console.WriteLine($"Inputted invalid address\nResetting...");
          isFirstByte = true;
          reading = false;
          address = 0;
          return 9;
        }
        return 0xFF;
      }

      if (!Register.IsDefined(typeof(Register), address))
        {
          Console.WriteLine($"Incremented to invalid address");
          return 1;
        }

      if (reading)
      {
        Console.WriteLine($"Reading from 0x{address:X2}");
        byte output = Read(data);
        Console.WriteLine($"Reading output is: 0x{output:X2}");
        return output;
      }
      else
      {
        Console.WriteLine($"Writing: 0x{data:X2}\nAddress: 0x{address:X2}");
        Write(data);
        return 0xEE;
      }
    }

    //Called if RW byte is 1, outputs data from current address
    public byte Read(byte data)
    {
      byte output = RegistersCollection.Read((byte)address);
      if (address == (byte)0x64)
      {
        //"This flag is reset by reading INT_SOURCE_REG (64h)."
        MROI.Value = false;
        if (latched.Value)
        {
            Console.WriteLine($"Latched, so resetting INT_SOURCE_REG thresholds");
            XExceedsPos = false;
            XExceedsNeg = false;
            YExceedsPos = false;
            YExceedsNeg = false;
            ZExceedsPos = false;
            ZExceedsNeg = false;
        }
      }
      address++;
      if (address == (byte)0x70)
      {
        address = (byte)0x45;
      }
      if (address == (byte)0x4b)
      {
        address = (byte)0x4f;
      }
      if (address == (byte)0x50)
      {
        address = (byte)0x60;
      }
      return output;
    }

    //Called if RW byte is 0, writes data to current register
    public void Write(byte data)
    {
      RegistersCollection.Write((byte)address, data);
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      Console.WriteLine("SPI transmission finished (CS high)\n");
      isFirstByte = true;
    }


    //The following uses dummy data to implement offset calculation behavior
    short x = 0;
    short y = 0;
    short z = 0;

    public void dummyData()
    {
      dummyData(
        0b0000000011111111, 0b0000000011111111, 0b0000000011111111);
    }

    //simulates device axis readings
    public void dummyData(short x, short y, short z)
    {
      writeData(x, ref outX, "x");
      drdyX.Value = true;
      writeData(y, ref outY, "y");
      drdyY.Value = true;
      writeData(z, ref outZ, "z");
      MROI.Value = true;
      drdyZ.Value = true;
      drdyZYX.Value = true;
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
    private ulong operationMode = 0;
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
    private bool XExceedsPos = false;
    private bool YExceedsPos = false;
    private bool ZExceedsPos = false;
    private bool XExceedsNeg = false;
    private bool YExceedsNeg = false;
    private bool ZExceedsNeg = false;
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
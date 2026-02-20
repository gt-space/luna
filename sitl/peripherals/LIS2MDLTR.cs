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

    // Called when the sensor is rebooted; different from a reset
    public void Reboot()
    {
      //This "Reboots magnetometer memory content"
    }

    //Inverts high and low bytes
    public void Invert()
    {
      //"an inversion of the low and high bytes of the data occurs"
    }

    public void driveDrdy()
    {
      bool signal = (enableInterrupt.Value && intOnPin.Value) || (drdyZYX.Value && drdyOnPin.Value);

      Interrupt.Set(signal);
    }

    // Defines registers and their sub-fields according to the datasheet.
    // Make sure that you account for default reset values.
    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_A, resetValue: 0x03)
        .withFlag(7, out tempCompIsOn, name: "COMP_TEMP_EN")
        .withFlag(6, name: "REBOOT",
          writeCallback: (_, value) =>
          {
            if (value)
            {
              Reboot();
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
          })
        .withFlag(4, out lowPower, name: "LP")
        .withValueField(2, 2, out outputDataRate, name: "ODR")
        .withValueField(0, 2, out operationMode, name: "MD");

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_B, resetValue: 0x00)
        .withFlag(4, out offsetCancellationOneShot, name: "OFF_CANC_ONE_SHOT")
        .withFlag(3, out interruptChecking, name: "INT_on_DataOFF")
        .withFlag(2, out pulseFrequency, name: "Set_FREQ")
        .withFlag(1, out offsetCancellation, name: "OFF_CANC")
        .withFlag(0, out lowpassFilter, name: "LPF");

      RegistersCollection
        .DefineRegister((long) Register.CFG_REG_C, resetValue: 0x00)
        .withFlag(6, out intOnPin, name: "INT_on_PIN",
          writeCallback(_, _) =>
          {
            driveDrdy();
          }
        )
        /*
        If 1, the INTERRUPT signal (INT bit in INT_SOURCE_REG (64h)) is driven to the INT/DRDY pin.
        The INT/DRDY pin is configured in push-pull output mode.
        */
        .withFlag(5, out I2CDisabled, name: "I2C_DIS")
        .withFlag(4, out blockDataUpdate, name: "BDU")
        .withFlag(3, name:"BLE",
          writeCallback(_, value) =>
          {
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
        .withFlag(0, out drdyOnPin, name: "DRDY_on_PIN",
          writeCallback(_, _) =>
          {
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
        .withFlag(7, out XExceedsPos, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "P_TH_S_X")
        .withFlag(6, out YExceedsPos, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "P_TH_S_Y")
        .withFlag(5, out ZExceedsPos, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "P_TH_S_Z")
        .withFlag(4, out XExceedsNeg, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "N_TH_S_X")
        .withFlag(3, out YExceedsNeg, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "N_TH_S_Y")
        .withFlag(2, out ZExceedsNeg, mode: useReadToClear ? FieldMode.ReadToClear : FieldMode.Read, name: "N_TH_S_Z")
        .withFlag(1, out MROI, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "MROI")
        .withFlag(0, mode: latched ? FieldMode.ReadToClear : FieldMode.Read, name: "INT",
          writeCallback(_, value) =>
          {
            if (!enableInterrupt)
            {
              //I'm assuming this would be prevented.
            } else if ((value && polarity) || (!(value || polarity)))
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
          valueProviderCallback: _ => (byte)(outX & 0xFF),
          writeCallback: (_, newValue) =>
          {
            outX = (ushort)((outX & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTX_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outXMSB",
          valueProviderCallback: _ => (byte)(outX >> 8),
          writeCallback: (_, newValue) =>
          {
            outX = (ushort)((outX & 0x00FF) | (newValue << 8));
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTY_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outYLSB",
          valueProviderCallback: _ => (byte)(outY & 0xFF),
          writeCallback: (_, newValue) =>
          {
            outY = (ushort)((outY & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTY_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outYMSB",
          valueProviderCallback: _ => (byte)(outY >> 8),
          writeCallback: (_, newValue) =>
          {
            outY = (ushort)((outY & 0x00FF) | (newValue << 8));
          }
        );

        RegistersCollection
        .DefineRegister((long) Register.OUTZ_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outZLSB",
          valueProviderCallback: _ => (byte)(outZ & 0xFF),
          writeCallback: (_, newValue) =>
          {
            outZ = (ushort)((outZ & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "outZMSB",
          valueProviderCallback: _ => (byte)(outZ >> 8),
          writeCallback: (_, newValue) =>
          {
            outZ = (ushort)((outZ & 0x00FF) | (newValue << 8));
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_L_REG)
        .withValueField(0, 8, FieldMode.Read, name: "tempLSB",
          valueProviderCallback: _ => (byte)(tempOut & 0xFF),
          writeCallback: (_, newValue) =>
          {
            tempOut = (ushort)((tempOut & 0xFF00) | newValue);
          }
        );

      RegistersCollection
        .DefineRegister((long) Register.TEMP_OUT_H_REG)
        .withValueField(0, 8, FieldMode.Read, name: "tempMSB",
          valueProviderCallback: _ => (byte)(tempOut >> 8),
          writeCallback: (_, newValue) =>
          {
            tempOut = (ushort)((newValue << 8) | (tempOut & 0x00FF));
          }
        );
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      Console.WriteLine($"Received SPI byte: {data}");
      Console.WriteLine("Replying (full-duplex) with 0xFF");
      return 0xFF;
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      Console.WriteLine("SPI transmission finished (CS high)");
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

    //CFG_REG_A:
    private IFlagRegisterField tempCompIsOn = null!;
    private IFlagRegisterField lowPower = null!;
    private IFlagRegisterField outputDataRate = null!;
    private IFlagRegisterField operationMode = null!;

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
    private ushort intThreshold = null!;

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
    private ushort outX = null!;
    private ushort outY = null!;
    private ushort outZ = null!;
    private short tempOut = null!;


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

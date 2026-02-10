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
        .withFlag(6, out null, name: "INT_on_PIN")
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
        .withFlag(0, name: "DRDY_on_PIN");
      /*
      If 1, the data-ready signal (Zyxda bit in STATUS_REG (67h)) is driven on the INT/DRDY pin.
      The INT/DRDY pin is configured in push-pull output mode
      */

      RegistersCollection
      .DefineRegister((long) Register.INT_CRTL_REG, resetValue: 0xE0)
      .withFlag(7, out enableXIE, name: "XIEN")
      .withFlag(6, out enableYIE, name: "YIEN")
      .withFlag(5, out enableZIE, name: "XIEN")
      .withFlag(4, )
      
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
    private IFlagRegisterField I2CDisabled = null!;
    private IFlagRegisterField blockDataUpdate = null!;
    private IFlagRegisterField enableSDO = null!;
    private IFlagRegisterField selfTestOn = null!;

    //INT_CTRL_REG:


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
      INT_THIS_L_REG = 0x65,
      INT_THIS_H_REG = 0x66,
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

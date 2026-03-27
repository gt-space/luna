using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class Example :
    ISPIPeripheral,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    public Example(IMachine machine)
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
    
    // Defines registers and their sub-fields according to the datasheet.
    // Make sure that you account for default reset values.
    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Register.ID, resetValue: 0x0F)
        .WithReservedBits(4, 4)
        .WithValueField(0, 4, FieldMode.Read, name: "IDENTITY");

      RegistersCollection
        .DefineRegister((long) Register.STATUS)
        .WithFlag(7, out isOn, FieldMode.Read, name: "ON")
        .WithFlag(6, out isUp, FieldMode.Read, name: "UP")
        .WithReservedBits(4, 2)
        .WithValueField(0, 4, out count, FieldMode.Read, name: "COUNT");

      RegistersCollection
        .DefineRegister((long) Register.IN)
        .WithValueField(0, 8, out input, FieldMode.Read | FieldMode.Write, name: "INPUT");

      RegistersCollection
        .DefineRegister((long) Register.OUT)
        .WithValueField(0, 8, out output, FieldMode.Read, name: "OUTPUT");
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
    private IFlagRegisterField isOn = null!;
    private IFlagRegisterField isUp = null!;
    private IValueRegisterField count = null!;
    private IValueRegisterField input = null!;
    private IValueRegisterField output = null!;

    // Retrieved directly from the register map in the datasheet of the chip.
    private enum Register
    {
      ID = 0x00,
      STATUS = 0x01,
      IN = 0x02,
      OUT = 0x03,
    }
  }
}

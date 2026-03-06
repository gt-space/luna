using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class MS5611 :
    ISPIPeripheral,
    IProvidesRegisterCollection<WordRegisterCollection>
  {
    private byte? command;
    private int byteIndex;
    private int digVal;
    private bool conversionDone;
    public MS5611(IMachine machine)
    {
      this.machine = machine;
      command = null;
      byteIndex = 0;
      digVal = 0;
      conversionDone = false;

      RegistersCollection = new WordRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    // Called when the sensor is reset, such as a RESET pin being pulled active.
    public void Reset()
    {
      RegistersCollection.Reset();
      command = null;
      byteIndex = 0;
      digVal = 0;
      conversionDone = false;
    }

    // Defines registers and their sub-fields according to the datasheet.
    // Make sure that you account for default reset values.
    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long)Register.C1, resetValue: 40127)
        .WithValueField(0, 16, FieldMode.Read);

      RegistersCollection
      .DefineRegister((long)Register.C2, resetValue: 36924)
      .WithValueField(0, 16, FieldMode.Read);

      RegistersCollection
      .DefineRegister((long)Register.C3, resetValue: 23317)
      .WithValueField(0, 16, FieldMode.Read);

      RegistersCollection
      .DefineRegister((long)Register.C4, resetValue: 23282)
      .WithValueField(0, 16, FieldMode.Read);

      RegistersCollection
      .DefineRegister((long)Register.C5, resetValue: 33464)
      .WithValueField(0, 16, FieldMode.Read);

      RegistersCollection
      .DefineRegister((long)Register.C6, resetValue: 28312)
      .WithValueField(0, 16, FieldMode.Read);
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      if (command == null)
      {
        command = data;
        byte conversion = (byte)(command & 0xF0);
        byteIndex = 0;
        return 0xFF;
      }

      byte response = 0x00;

      if (command == 0x1E) // reset command
      {
        Reset();
      }
      else if ((command & 0xF0) == 0x40) // d1 conversion command
      {
        digVal = 9085466;
        conversionDone = true;
      }
      else if ((command & 0xF0) == 0x50) // d2 conversion command
      {
        digVal = 8569150;
        conversionDone = true;
      }
      else if (command == 0x00) // adc read command
      {
        if (conversionDone)
        {
          response = (byte)((digVal >> (16 - (8 * byteIndex))) & 0xFF);
          if (byteIndex == 2)
          {
            conversionDone = false;
          }
          byteIndex++;
        }
      }
      else if ((command & 0xF0) == 0xA0) // PROM read command
      {
        int cnum = ((byte)command >> 1) & 0x07;  // 0xA2 -> cnum = 1
        long address = cnum * 2;  // 1 * 2 = 0x02

        if (cnum == 0)
        {
          return 0x00;
        }
        else if (cnum == 7)
        {
          ushort[] proms = new ushort[] { 0, RegistersCollection.Read(0x02), RegistersCollection.Read(0x04), RegistersCollection.Read(0x06), RegistersCollection.Read(0x08), RegistersCollection.Read(0x0A), RegistersCollection.Read(0x0C), 0 };

          if (byteIndex == 0)
          {
            response = 0x00;
          }
          else if (byteIndex == 1)
          {
            response = crc4(proms);
          }
        }
        else
        {
          ushort prom = RegistersCollection.Read(address);
          byte msb = (byte)(prom >> 8);
          byte lsb = (byte)(prom & 0xFF);
          if (byteIndex == 0)
          {
            response = msb;
          }
          else if (byteIndex == 1)
          {
            response = lsb;
          }
        }
        byteIndex++;
      }

      return response;
    }

    // Calculates CRC
    private byte crc4(ushort[] n_prom)
    {
      int cnt; // simple counter
      ushort n_rem; // crc reminder
      ushort crc_read; // original value of the crc
      byte n_bit;
      n_rem = 0x00;
      crc_read = n_prom[7]; //save read CRC
      n_prom[7] = (ushort)(0xFF00 & n_prom[7]); //CRC byte is replaced by 0
      for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
      { // choose LSB or MSB
        if (cnt % 2 == 1)
        {
          n_rem ^= (ushort)(n_prom[cnt >> 1] & 0x00FF);
        }
        else
        {
          n_rem ^= (ushort)(n_prom[cnt >> 1] >> 8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
          if ((n_rem & 0x8000) != 0)
          {
            n_rem = (ushort)((n_rem << 1) ^ 0x3000);
          }
          else
          {
            n_rem = (ushort)(n_rem << 1);
          }
        }
      }
      n_rem = (ushort)(0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
      n_prom[7] = crc_read; // restore the crc_read to its original place
      return (byte)(n_rem ^ 0x00);
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      byteIndex = 0;
      command = null;
    }

    // Maps the register map of the chip onto a virtual register collection.
    public WordRegisterCollection RegistersCollection { get; private set; }

    private IMachine machine;

    // Fields derived from fields in the register collection.
    //
    // The `null!` default initialization is applied because they will all be
    // // set in the `DefineRegisters` method, which is called by the constructor.
    // private IFlagRegisterField isOn = null!;
    // private IFlagRegisterField isUp = null!;
    // private IValueRegisterField count = null!;
    // private IValueRegisterField input = null!;
    // private IValueRegisterField output = null!;

    // Retrieved directly from the register map in the datasheet of the chip.
    private enum Register
    {
      C1 = 0x02,
      C2 = 0x04,
      C3 = 0x06,
      C4 = 0x08,
      C5 = 0x0A,
      C6 = 0x0C,
    }
  }
}

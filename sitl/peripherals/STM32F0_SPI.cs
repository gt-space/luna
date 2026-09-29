//
// Derived from Renode's STM32SPI peripheral.
//   Copyright (c) 2010-2024 Antmicro
//   Copyright (c) 2011-2015 Realtime Embedded
//   Licensed under the MIT License (see Renode's licenses/MIT.txt).
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.SPI
{
  /// <summary>
  ///  STM32F0 SPI controller with a packing RX FIFO.
  /// </summary>
  /// <remarks>
  ///  Renode's stock STM32SPI keeps a byte-wide receive FIFO, asserts RXNE as
  ///  soon as it holds anything, and pops exactly one byte per SPI_DR read.
  ///  It parses CR2.FRXTH into a field and then never reads that field.
  ///
  ///  On the F0 that is wrong. HAL_SPI_TransmitReceive clears FRXTH whenever
  ///  it has more than one byte left to receive, waits for RXNE, and then
  ///  reads SPI_DR as a halfword to take *two* bytes out of one access. The
  ///  stock model hands back one, so the HAL consumes one byte fewer than it
  ///  accounts for. The leftovers accumulate in the FIFO and every transfer
  ///  after the first is skewed, which shows up in a driver as reading the
  ///  response to the previous byte.
  ///
  ///  This version honours FRXTH: RXNE waits for two bytes when the threshold
  ///  is 16-bit, and a DR read in that state returns both, first byte in the
  ///  low half.
  /// </remarks>
  public sealed class STM32F0_SPI :
    NullRegistrationPointPeripheralContainer<ISPIPeripheral>,
    IWordPeripheral,
    IDoubleWordPeripheral,
    IBytePeripheral,
    IKnownSize
  {
    public STM32F0_SPI(IMachine machine) : base(machine)
    {
      receiveBuffer = new Queue<byte>();
      IRQ = new GPIO();
      registers = new DoubleWordRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    public byte ReadByte(long offset)
    {
      return (byte)ReadDoubleWord(offset);
    }

    public void WriteByte(long offset, byte value)
    {
      WriteDoubleWord(offset, value);
    }

    public ushort ReadWord(long offset)
    {
      return (ushort)ReadDoubleWord(offset);
    }

    public void WriteWord(long offset, ushort value)
    {
      WriteDoubleWord(offset, value);
    }

    public uint ReadDoubleWord(long offset)
    {
      return registers.Read(offset);
    }

    public void WriteDoubleWord(long offset, uint value)
    {
      registers.Write(offset, value);
    }

    public override void Reset()
    {
      IRQ.Unset();
      lock(receiveBuffer)
      {
        receiveBuffer.Clear();
      }
      registers.Reset();
    }

    public long Size => 0x400;

    public GPIO IRQ { get; }

    private void DefineRegisters()
    {
      Registers.Control1.Define(registers)
        .WithValueField(0, 16, name: "CR1");

      Registers.Control2.Define(registers)
        .WithValueField(0, 8, name: "CR2_LOW")
        .WithValueField(8, 4, name: "DS")
        .WithFlag(12, out byteThreshold, name: "FRXTH")
        .WithReservedBits(13, 19);

      Registers.Status.Define(registers, 2)
        .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => IsReceiveReady(),
                  name: "RXNE")
        .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => true, name: "TXE")
        .WithTaggedFlag("CHSIDE", 2)
        .WithTaggedFlag("UDR", 3)
        .WithTaggedFlag("CRCERR", 4)
        .WithTaggedFlag("MODF", 5)
        .WithFlag(6, out overrun, FieldMode.Read, name: "OVR")
        .WithTaggedFlag("BSY", 7)
        .WithTaggedFlag("FRE", 8)
        .WithValueField(9, 2, FieldMode.Read,
                        valueProviderCallback: _ => FifoLevel(), name: "FRLVL")
        .WithValueField(11, 2, FieldMode.Read, valueProviderCallback: _ => 0,
                        name: "FTLVL")
        .WithReservedBits(13, 19);

      Registers.Data.Define(registers)
        .WithValueField(0, 16, valueProviderCallback: _ => HandleDataRead(),
                        writeCallback: (_, value) => HandleDataWrite((byte)value),
                        name: "DR")
        .WithReservedBits(16, 16);

      Registers.CRCPolynomial.Define(registers, 7)
        .WithValueField(0, 16, name: "CRCPOLY")
        .WithReservedBits(16, 16);

      Registers.ReceivedCRC.Define(registers)
        .WithValueField(0, 16, FieldMode.Read, name: "RXCRC")
        .WithReservedBits(16, 16);

      Registers.TransmittedCRC.Define(registers)
        .WithValueField(0, 16, FieldMode.Read, name: "TXCRC")
        .WithReservedBits(16, 16);

      Registers.I2SConfiguration.Define(registers)
        .WithValueField(0, 16, name: "I2SCFGR")
        .WithReservedBits(16, 16);

      Registers.I2SPrescaler.Define(registers, 10)
        .WithValueField(0, 16, name: "I2SPR")
        .WithReservedBits(16, 16);
    }

    // With a 16-bit threshold the HAL waits for two bytes and then takes both
    // in one access, so RXNE must not fire on a single byte.
    private bool IsReceiveReady()
    {
      lock(receiveBuffer)
      {
        return receiveBuffer.Count >= RequiredBytes;
      }
    }

    private ulong FifoLevel()
    {
      lock(receiveBuffer)
      {
        return (ulong)(receiveBuffer.Count > 3 ? 3 : receiveBuffer.Count);
      }
    }

    private int RequiredBytes => byteThreshold.Value ? 1 : 2;

    private ulong HandleDataRead()
    {
      IRQ.Unset();
      lock(receiveBuffer)
      {
        ulong result = 0;
        for(var i = 0; i < RequiredBytes; i++)
        {
          if(receiveBuffer.Count == 0)
          {
            // The HAL reads DR speculatively to clear flags, so an empty
            // FIFO here is expected and not worth a warning.
            break;
          }
          result |= (ulong)receiveBuffer.Dequeue() << (8 * i);
        }
        return result;
      }
    }

    private void HandleDataWrite(byte value)
    {
      IRQ.Unset();
      lock(receiveBuffer)
      {
        var peripheral = RegisteredPeripheral;
        if(peripheral == null)
        {
          this.Log(LogLevel.Warning, "Transmission with no SPI peripheral attached");
          receiveBuffer.Enqueue(0x00);
          return;
        }

        var response = peripheral.Transmit(value);

        if(receiveBuffer.Count >= FifoCapacity)
        {
          this.Log(LogLevel.Warning, "RX FIFO full, dropping 0x{0:X2}", response);
          overrun.Value = true;
        }
        else
        {
          receiveBuffer.Enqueue(response);
        }

        this.Log(LogLevel.Noisy, "Transmitted 0x{0:X2}, received 0x{1:X2}",
                 value, response);
      }
    }

    private readonly Queue<byte> receiveBuffer;
    private readonly DoubleWordRegisterCollection registers;

    private IFlagRegisterField byteThreshold;
    private IFlagRegisterField overrun;

    private const int FifoCapacity = 4;

    private enum Registers
    {
      Control1 = 0x00,
      Control2 = 0x04,
      Status = 0x08,
      Data = 0x0C,
      CRCPolynomial = 0x10,
      ReceivedCRC = 0x14,
      TransmittedCRC = 0x18,
      I2SConfiguration = 0x1C,
      I2SPrescaler = 0x20,
    }
  }
}

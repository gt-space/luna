using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.SPI
{
  public class OMAP_McSPI :
    SimpleContainer<ISPIPeripheral>,
    IProvidesRegisterCollection<DoubleWordRegisterCollection>,
    IDoubleWordPeripheral
  {
    public OMAP_McSPI(IMachine machine) : base(machine)
    {
      channels = new Channel[4];

      RegistersCollection = new DoubleWordRegisterCollection(this);
      DefineRegisters();
    }

    public override void Reset()
    {
      RegistersCollection.Reset();
    }

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Registers.Revision, resetValue: 0x00300000)
        .WithValueField(30, 2, FieldMode.Read, name: "SCHEME")
        .WithReservedBits(28, 2)
        .WithValueField(16, 12, FieldMode.Read, name: "FUNC")
        .WithValueField(11, 5, FieldMode.Read, name: "R_RTL")
        .WithValueField(8, 3, FieldMode.Read, name: "X_MAJOR")
        .WithValueField(6, 2, FieldMode.Read, name: "CUSTOM")
        .WithValueField(0, 6, FieldMode.Read, name: "Y_MINOR");

      RegistersCollection
        .DefineRegister((long) Registers.SysConfig)
        .WithReservedBits(10, 22)
        .WithValueField(8, 2, FieldMode.Read | FieldMode.Write, name: "CLOCKACTIVITY")
        .WithReservedBits(5, 3)
        .WithValueField(3, 2, FieldMode.Read | FieldMode.Write, name: "SIDLEMODE")
        .WithReservedBits(2, 1)
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "SOFTRESET")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "AUTOIDLE");

      RegistersCollection
        .DefineRegister((long) Registers.SysStatus)
        .WithReservedBits(1, 31)
        .WithFlag(0, FieldMode.Read, name: "RESETDONE");

      RegistersCollection
        .DefineRegister((long) Registers.IrqStatus)
        .WithReservedBits(18, 14)
        .WithFlag(17, FieldMode.Read | FieldMode.Write, name: "EOW")
        .WithReservedBits(15, 2)
        .WithFlag(14, FieldMode.Read | FieldMode.Write, name: "RX3_FULL")
        .WithFlag(13, FieldMode.Read | FieldMode.Write, name: "TX3_UNDERFLOW")
        .WithFlag(12, FieldMode.Read | FieldMode.Write, name: "TX3_EMPTY")
        .WithReservedBits(11, 1)
        .WithFlag(10, FieldMode.Read | FieldMode.Write, name: "RX2_FULL")
        .WithFlag(9, FieldMode.Read | FieldMode.Write, name: "TX2_UNDERFLOW")
        .WithFlag(8, FieldMode.Read | FieldMode.Write, name: "TX2_EMPTY")
        .WithReservedBits(7, 1)
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "RX1_FULL")
        .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "TX1_UNDERFLOW")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "TX1_EMPTY")
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "RX0_OVERFLOW")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "RX0_FULL")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "TX0_UNDERFLOW")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "TX0_EMPTY");

      RegistersCollection
        .DefineRegister((long) Registers.IrqEnable)
        .WithReservedBits(18, 14)
        .WithFlag(17, FieldMode.Read | FieldMode.Write, name: "EOWKE")
        .WithReservedBits(15, 2)
        .WithFlag(14, FieldMode.Read | FieldMode.Write, name: "RX3_FULL__ENABLE")
        .WithFlag(13, FieldMode.Read | FieldMode.Write, name: "TX3_UNDERFLOW__ENABLE")
        .WithFlag(12, FieldMode.Read | FieldMode.Write, name: "TX3_EMPTY__ENABLE")
        .WithReservedBits(11, 1)
        .WithFlag(10, FieldMode.Read | FieldMode.Write, name: "RX2_FULL__ENABLE")
        .WithFlag(9, FieldMode.Read | FieldMode.Write, name: "TX2_UNDERFLOW__ENABLE")
        .WithFlag(8, FieldMode.Read | FieldMode.Write, name: "TX2_EMPTY__ENABLE")
        .WithReservedBits(7, 1)
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "RX1_FULL__ENABLE")
        .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "TX1_UNDERFLOW__ENABLE")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "TX1_EMPTY__ENABLE")
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "RX0_OVERFLOW__ENABLE")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "RX0_FULL__ENABLE")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "TX0_UNDERFLOW__ENABLE")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "TX0_EMPTY__ENABLE");

      RegistersCollection
        .DefineRegister((long) Registers.Syst)
        .WithReservedBits(12, 20)
        .WithFlag(11, FieldMode.Read | FieldMode.Write, name: "SSB")
        .WithFlag(10, FieldMode.Read | FieldMode.Write, name: "SPIENDIR")
        .WithFlag(9, FieldMode.Read | FieldMode.Write, name: "SPIDATDIR1")
        .WithFlag(8, FieldMode.Read | FieldMode.Write, name: "SPIDATDIR0")
        .WithReservedBits(7, 1)
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "SPICLK")
        .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "SPIDAT_1")
        .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "SPIDAT_0")
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "SPIEN_3")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "SPIEN_2")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "SPIEN_1")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "SPIEN_0");

      RegistersCollection
        .DefineRegister((long) Registers.ModulCtrl)
        .WithReservedBits(9, 23)
        .WithFlag(8, FieldMode.Read | FieldMode.Write, name: "FDAA")
        .WithFlag(7, FieldMode.Read | FieldMode.Write, name: "MOA")
        .WithValueField(4, 3, FieldMode.Read | FieldMode.Write, name: "INITDLY")
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "SYSTEM_TEST")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "MS")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "PIN34")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "SINGLE");

      DefineChannelRegisters(
        Registers.Ch0Conf,
        Registers.Ch0Stat,
        Registers.Ch0Ctrl,
        Registers.Tx0,
        Registers.Rx0
      );

      DefineChannelRegisters(
        Registers.Ch1Conf,
        Registers.Ch1Stat,
        Registers.Ch1Ctrl,
        Registers.Tx1,
        Registers.Rx1
      );

      DefineChannelRegisters(
        Registers.Ch2Conf,
        Registers.Ch2Stat,
        Registers.Ch2Ctrl,
        Registers.Tx2,
        Registers.Rx2
      );

      DefineChannelRegisters(
        Registers.Ch3Conf,
        Registers.Ch3Stat,
        Registers.Ch3Ctrl,
        Registers.Tx3,
        Registers.Rx3
      );

      RegistersCollection
        .DefineRegister((long) Registers.XferLevel)
        .WithValueField(16, 16, FieldMode.Read | FieldMode.Write, name: "WCNT")
        .WithValueField(8, 8, FieldMode.Read | FieldMode.Write, name: "AFL")
        .WithValueField(0, 8, FieldMode.Read | FieldMode.Write, name: "AEL");

      RegistersCollection
        .DefineRegister((long) Registers.DafTx)
        .WithValueField(0, 32, FieldMode.Read | FieldMode.Write, name: "DAFTDATA");

      RegistersCollection
        .DefineRegister((long) Registers.DafRx)
        .WithValueField(0, 32, FieldMode.Read, name: "DAFRDATA");
    }

    private void DefineChannelRegisters(
      Registers conf,
      Registers stat,
      Registers ctrl,
      Registers tx,
      Registers rx
    )
    {
      RegistersCollection
        .DefineRegister((long) conf)
        .WithReservedBits(30, 2)
        .WithFlag(29, FieldMode.Read | FieldMode.Write, name: "CLKG")
        .WithFlag(28, FieldMode.Read | FieldMode.Write, name: "FFER")
        .WithFlag(27, FieldMode.Read | FieldMode.Write, name: "FFEW")
        .WithValueField(25, 2, FieldMode.Read | FieldMode.Write, name: "TCS")
        .WithFlag(24, FieldMode.Read | FieldMode.Write, name: "SBPOL")
        .WithFlag(23, FieldMode.Read | FieldMode.Write, name: "SBE")
        .WithValueField(21, 2, FieldMode.Read | FieldMode.Write, name: "SPIENSLV")
        .WithFlag(20, FieldMode.Read | FieldMode.Write, name: "FORCE")
        .WithFlag(19, FieldMode.Read | FieldMode.Write, name: "TURBO")
        .WithFlag(18, FieldMode.Read | FieldMode.Write, name: "IS")
        .WithFlag(17, FieldMode.Read | FieldMode.Write, name: "DPE1")
        .WithFlag(16, FieldMode.Read | FieldMode.Write, name: "DPE0")
        .WithFlag(15, FieldMode.Read | FieldMode.Write, name: "DMAR")
        .WithFlag(14, FieldMode.Read | FieldMode.Write, name: "DMAW")
        .WithValueField(12, 2, FieldMode.Read | FieldMode.Write, name: "TRM")
        .WithValueField(7, 5, FieldMode.Read | FieldMode.Write, name: "WL")
        .WithFlag(6, FieldMode.Read | FieldMode.Write, name: "EPOL")
        .WithValueField(2, 4, FieldMode.Read | FieldMode.Write, name: "CLKD")
        .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "POL")
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "PHA");

      RegistersCollection
        .DefineRegister((long) stat)
        .WithReservedBits(7, 25)
        .WithFlag(6, FieldMode.Read, name: "RXFFF")
        .WithFlag(6, FieldMode.Read, name: "RXFFE")
        .WithFlag(6, FieldMode.Read, name: "TXFFF")
        .WithFlag(6, FieldMode.Read, name: "TXFFE")
        .WithFlag(6, FieldMode.Read, name: "EOT")
        .WithFlag(6, FieldMode.Read, name: "TXS")
        .WithFlag(6, FieldMode.Read, name: "RXS");

      RegistersCollection
        .DefineRegister((long) ctrl)
        .WithReservedBits(16, 16)
        .WithValueField(8, 8, FieldMode.Read | FieldMode.Write, name: "EXTCLK")
        .WithReservedBits(1, 7)
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "EN");

      RegistersCollection
        .DefineRegister((long) tx)
        .WithValueField(0, 32, FieldMode.Read | FieldMode.Write, name: "TDATA");

      RegistersCollection
        .DefineRegister((long) rx)
        .WithValueField(0, 32, FieldMode.Read, name: "RDATA");
    }

    public uint ReadDoubleWord(long offset)
    {
      return RegistersCollection.Read(offset);
    }

    public void WriteDoubleWord(long offset, uint value)
    {
      RegistersCollection.Write(offset, value);
    }

    public DoubleWordRegisterCollection RegistersCollection { get; }
    private Channel[] channels;

    private class Channel
    {
      public Channel()
      {

      }
    }

    public enum Registers
    {
      Revision = 0x000,
      SysConfig = 0x110,
      SysStatus = 0x114,
      IrqStatus = 0x118,
      IrqEnable = 0x11C,
      Syst = 0x124,
      ModulCtrl = 0x128,
      Ch0Conf = 0x12C,
      Ch0Stat = 0x130,
      Ch0Ctrl = 0x134,
      Tx0 = 0x138,
      Rx0 = 0x13C,
      Ch1Conf = 0x140,
      Ch1Stat = 0x144,
      Ch1Ctrl = 0x148,
      Tx1 = 0x14C,
      Rx1 = 0x150,
      Ch2Conf = 0x154,
      Ch2Stat = 0x158,
      Ch2Ctrl = 0x15C,
      Tx2 = 0x160,
      Rx2 = 0x164,
      Ch3Conf = 0x168,
      Ch3Stat = 0x16C,
      Ch3Ctrl = 0x170,
      Tx3 = 0x174,
      Rx3 = 0x178,
      XferLevel = 0x17C,
      DafTx = 0x180,
      DafRx = 0x1A0,
    }
  }
}

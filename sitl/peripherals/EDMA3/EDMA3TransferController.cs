using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
  public class EDMA3TransferController :
    IDoubleWordPeripheral,
    IProvidesRegisterCollection<DoubleWordRegisterCollection>,
    IKnownSize
  {
    public EDMA3TransferController(IMachine machine)
    {
      RegistersCollection = new DoubleWordRegisterCollection(this);
      DefineRegisters();
    }

    public void Reset()
    {
      RegistersCollection.Reset();
    }

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Registers.PID, resetValue: 0x40007C00)
        .WithReservedBits(16, 16)
        .WithValueField(0, 16, FieldMode.Read, name: "PID");

      RegistersCollection
        .DefineRegister((long) Registers.TCCFG, resetValue: 0x00000224)
        .WithReservedBits(10, 22)
        .WithValueField(8, 2, FieldMode.Read, name: "DREGDEPTH")
        .WithReservedBits(6, 2)
        .WithValueField(4, 2, FieldMode.Read, name: "BUSWIDTH")
        .WithReservedBits(3, 1)
        .WithValueField(0, 3, FieldMode.Read, name: "FIFOSIZE");

      RegistersCollection
        .DefineRegister((long) Registers.SysConfig, resetValue: 0x00000028)
        .WithReservedBits(6, 26)
        .WithValueField(4, 2, FieldMode.Read | FieldMode.Write, name: "STANDBYMODE")
        .WithValueField(2, 2, FieldMode.Read | FieldMode.Write, name: "IDLEMODE")
        .WithReservedBits(0, 2);

      RegistersCollection
        .DefineRegister((long) Registers.TCStat, resetValue: 0x00000100)
        .WithReservedBits(14, 18)
        .WithValueField(12, 2, FieldMode.Read, name: "DFSTRTPTR")
        .WithReservedBits(7, 5)
        .WithValueField(4, 3, FieldMode.Read, name: "DSTACTV")
        .WithReservedBits(3, 1)
        .WithFlag(2, FieldMode.Read, name: "WSACTV")
        .WithFlag(1, FieldMode.Read, name: "SRCACTV")
        .WithFlag(0, FieldMode.Read, name: "PROGBUSY");

      RegistersCollection
        .DefineRegister((long) Registers.ErrStat)
        .WithReservedBits(4, 28)
        .WithFlag(3, FieldMode.Read, name: "MMRAERR")
        .WithFlag(2, FieldMode.Read, name: "TRERR")
        .WithReservedBits(1, 1)
        .WithFlag(0, FieldMode.Read, name: "BUSERR");

      RegistersCollection
        .DefineRegister((long) Registers.ErrEn)
        .WithReservedBits(4, 28)
        .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "MMRAERR")
        .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "TRERR")
        .WithReservedBits(1, 1)
        .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "BUSERR");

      RegistersCollection
        .DefineRegister((long) Registers.ErrClr)
        .WithReservedBits(4, 28)
        .WithFlag(3, FieldMode.Write, name: "MMRAERR")
        .WithFlag(2, FieldMode.Write, name: "TRERR")
        .WithReservedBits(1, 1)
        .WithFlag(0, FieldMode.Write, name: "BUSERR");

      RegistersCollection
        .DefineRegister((long) Registers.ErrDet)
        .WithReservedBits(18, 14)
        .WithFlag(17, FieldMode.Read, name: "TCCHEN")
        .WithFlag(16, FieldMode.Read, name: "TCINTEN")
        .WithReservedBits(14, 2)
        .WithValueField(8, 6, FieldMode.Read, name: "TCC")
        .WithReservedBits(4, 4)
        .WithValueField(0, 4, FieldMode.Read, name: "STAT");

      RegistersCollection
        .DefineRegister((long) Registers.ErrCmd)
        .WithReservedBits(1, 31)
        .WithFlag(0, FieldMode.Write, name: "EVAL");

      RegistersCollection
        .DefineRegister((long) Registers.RdRate)
        .WithReservedBits(3, 29)
        .WithValueField(0, 3, FieldMode.Read | FieldMode.Write, name: "RDRATE");

      void DefineOptionsRegister(long offset)
      {
        RegistersCollection
          .DefineRegister(offset)
          .WithReservedBits(23, 9)
          .WithFlag(22, FieldMode.Read, name: "TCCHEN")
          .WithReservedBits(21, 1)
          .WithFlag(20, FieldMode.Read, name: "TCINTEN")
          .WithReservedBits(18, 2)
          .WithValueField(12, 6, FieldMode.Read, name: "TCC")
          .WithReservedBits(11, 1)
          .WithValueField(8, 3, FieldMode.Read, name: "FWID")
          .WithReservedBits(7, 1)
          .WithValueField(4, 3, FieldMode.Read, name: "PRI")
          .WithReservedBits(2, 2)
          .WithFlag(1, FieldMode.Read, name: "DAM")
          .WithFlag(0, FieldMode.Read, name: "SAM");
      }

      DefineOptionsRegister((long) Registers.SAOpt);

      RegistersCollection
        .DefineRegister((long) Registers.SASrc)
        .WithValueField(0, 32, FieldMode.Read, name: "SADDR");

      void DefineCountRegister(long offset)
      {
        RegistersCollection
          .DefineRegister(offset)
          .WithValueField(16, 16, FieldMode.Read, name: "BCNT")
          .WithValueField(0, 16, FieldMode.Read, name: "ACNT");
      }

      DefineCountRegister((long) Registers.SACnt);

      RegistersCollection
        .DefineRegister((long) Registers.SADst)
        .WithReservedBits(0, 32);

      void DefineBIndexRegister(long offset)
      {
        RegistersCollection
          .DefineRegister(offset)
          .WithValueField(16, 16, FieldMode.Read, name: "DBIDX")
          .WithValueField(0, 16, FieldMode.Read, name: "SBIDX");
      }

      DefineBIndexRegister((long) Registers.SABIdx);

      void DefineProxyRegister(long offset)
      {
        RegistersCollection
          .DefineRegister(offset)
          .WithReservedBits(9, 23)
          .WithFlag(8, FieldMode.Read, name: "PRIV")
          .WithReservedBits(4, 4)
          .WithValueField(0, 4, FieldMode.Read, name: "PRIVID");
      }

      DefineProxyRegister((long) Registers.SAMPPrxy);

      RegistersCollection
        .DefineRegister((long) Registers.SACntRld)
        .WithReservedBits(16, 16)
        .WithValueField(0, 16, FieldMode.Read, name: "ACNTRLD");

      RegistersCollection
        .DefineRegister((long) Registers.SASrcBRef)
        .WithValueField(0, 32, FieldMode.Read, name: "SADDRBREF");

      RegistersCollection
        .DefineRegister((long) Registers.SADstBRef)
        .WithReservedBits(0, 32);

      RegistersCollection
        .DefineRegister((long) Registers.DFCntRld)
        .WithReservedBits(16, 16)
        .WithValueField(0, 16, FieldMode.Read, name: "ACNTRLD");

      RegistersCollection
        .DefineRegister((long) Registers.DFSrcBRef)
        .WithReservedBits(0, 32);

      RegistersCollection
        .DefineRegister((long) Registers.DFDstBRef)
        .WithValueField(0, 32, FieldMode.Read, name: "DADDRBREF");

      for (int i = 0; i < 4; i++)
      {
        long offset = i * 0x40;

        DefineOptionsRegister((long) Registers.DFOpt0 + offset);

        RegistersCollection
          .DefineRegister((long) Registers.DFSrc0 + offset)
          .WithReservedBits(0, 32);

        DefineCountRegister((long) Registers.DFCnt0 + offset);

        RegistersCollection
          .DefineRegister((long) Registers.DFDst0 + offset)
          .WithValueField(0, 32, FieldMode.Read, name: "DADDR");

        DefineBIndexRegister((long) Registers.DFBIdx0 + offset);
        DefineProxyRegister((long) Registers.DFMPPrxy0 + offset);
      }
    }

    public uint ReadDoubleWord(long offset)
    {
      return RegistersCollection.Read(offset);
    }

    public void WriteDoubleWord(long offset, uint value)
    {
      RegistersCollection.Write(offset, value);
    }

    public void SetChannelController(EDMA3ChannelController controller)
    {
      this.channelController = controller;
    }

    public DoubleWordRegisterCollection RegistersCollection { get; }
    public long Size => 1 << 20;

    private EDMA3ChannelController? channelController = null;

    public enum Registers
    {
      PID = 0x000,
      TCCFG = 0x004,
      SysConfig = 0x010,
      TCStat = 0x100,
      ErrStat = 0x120,
      ErrEn = 0x124,
      ErrClr = 0x128,
      ErrDet = 0x12C,
      ErrCmd = 0x130,
      RdRate = 0x140,
      SAOpt = 0x240,
      SASrc = 0x244,
      SACnt = 0x248,
      SADst = 0x24C,
      SABIdx = 0x250,
      SAMPPrxy = 0x254,
      SACntRld = 0x258,
      SASrcBRef = 0x25C,
      SADstBRef = 0x260,
      DFCntRld = 0x280,
      DFSrcBRef = 0x284,
      DFDstBRef = 0x288,
      DFOpt0 = 0x300,
      DFSrc0 = 0x304,
      DFCnt0 = 0x308,
      DFDst0 = 0x30C,
      DFBIdx0 = 0x310,
      DFMPPrxy0 = 0x314,
    }
  }
}

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;

using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.DMA
{
  public class EDMA3ChannelController :
    IDoubleWordPeripheral,
    IProvidesRegisterCollection<DoubleWordRegisterCollection>,
    IKnownSize
  {
    public EDMA3ChannelController(
      IMachine machine,
      EDMA3TransferController? tc0 = null,
      EDMA3TransferController? tc1 = null,
      EDMA3TransferController? tc2 = null
    )
    {
      RegistersCollection = new DoubleWordRegisterCollection(this);
      DefineRegisters();

      transferControllers = new List<EDMA3TransferController>();

      if (tc0 != null)
      {
        transferControllers.Add(tc0);
        tc0.SetChannelController(this);
      }

      if (tc1 != null)
      {
        transferControllers.Add(tc1);
        tc1.SetChannelController(this);
      }

      if (tc2 != null)
      {
        transferControllers.Add(tc2);
        tc2.SetChannelController(this);
      }
    }

    public void Reset()
    {
      RegistersCollection.Reset();
    }

    private void DefineRegisters()
    {
      RegistersCollection
        .DefineRegister((long) Registers.PID, resetValue: 0x40014C00)
        .WithValueField(0, 32, FieldMode.Read, name: "PID");

      RegistersCollection
        .DefineRegister((long) Registers.CCCfg, resetValue: 0x00322445)
        .WithReservedBits(26, 6)
        .WithFlag(25, FieldMode.Read, name: "MP_EXIST")
        .WithFlag(24, FieldMode.Read, name: "CHMAP_EXIST")
        .WithReservedBits(22, 2)
        .WithValueField(20, 2, FieldMode.Read, name: "NUM_REGN")
        .WithReservedBits(19, 1)
        .WithValueField(16, 3, FieldMode.Read, name: "NUM_EVQUE")
        .WithReservedBits(15, 1)
        .WithValueField(12, 3, FieldMode.Read, name: "NUM_PAENTRY")
        .WithReservedBits(11, 1)
        .WithValueField(8, 3, FieldMode.Read, name: "NUM_INTCH")
        .WithReservedBits(7, 1)
        .WithValueField(4, 3, FieldMode.Read, name: "NUM_QDMACH")
        .WithReservedBits(3, 1)
        .WithValueField(0, 3, FieldMode.Read, name: "NUM_DMACH");

      RegistersCollection
        .DefineRegister((long) Registers.SysConfig, resetValue: 0x00000008)
        .WithReservedBits(4, 28)
        .WithValueField(2, 2, FieldMode.Read | FieldMode.Write, name: "IDLEMODE")
        .WithReservedBits(0, 2);

      for (int i = 0; i < 64; i++)
      {
        RegistersCollection
          .DefineRegister((long) Registers.DChMap0 + i * sizeof(uint))
          .WithReservedBits(14, 18)
          .WithValueField(5, 9, FieldMode.Read | FieldMode.Write, name: "PAENTRY")
          .WithReservedBits(0, 5);
      }

      for (int i = 0; i < 8; i++)
      {
        RegistersCollection
          .DefineRegister((long) Registers.QChMap0 + i * sizeof(uint))
          .WithReservedBits(14, 18)
          .WithValueField(5, 9, FieldMode.Read | FieldMode.Write, name: "PAENTRY")
          .WithValueField(2, 3, FieldMode.Read | FieldMode.Write, name: "TRWORD")
          .WithReservedBits(0, 2);
      }

      // DMAQNUM_0 -> DMAQNUM_7
      for (int i = 0; i < 8; i++)
      {
        DefineQueueRegister((long) Registers.DmaQNum0 + i * sizeof(uint));
      }

      DefineQueueRegister((long) Registers.QDmaQNum);

      RegistersCollection
        .DefineRegister((long) Registers.QuePri, resetValue: 0x00000777)
        .WithReservedBits(11, 21)
        .WithValueField(8, 3, FieldMode.Read | FieldMode.Write, name: "PRIQ2")
        .WithReservedBits(7, 1)
        .WithValueField(4, 3, FieldMode.Read | FieldMode.Write, name: "PRIQ1")
        .WithReservedBits(3, 1)
        .WithValueField(0, 3, FieldMode.Read | FieldMode.Write, name: "PRIQ0");

      RegistersCollection
        .DefineRegister((long) Registers.EMR)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EMRH)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EMCR)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EMCRH)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QEMR)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QEMCR)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.CCErr)
        .WithReservedBits(17, 15)
        .WithFlag(16, FieldMode.Read, name: "TCCERR")
        .WithReservedBits(3, 13)
        .WithFlag(2, FieldMode.Read, name: "QTHRXCD2")
        .WithFlag(1, FieldMode.Read, name: "QTHRXCD1")
        .WithFlag(0, FieldMode.Read, name: "QTHRXCD0");

      RegistersCollection
        .DefineRegister((long) Registers.CCErrClr)
        .WithReservedBits(17, 15)
        .WithFlag(16, FieldMode.Write, name: "TCCERR")
        .WithReservedBits(3, 13)
        .WithFlag(2, FieldMode.Write, name: "QTHRXCD2")
        .WithFlag(1, FieldMode.Write, name: "QTHRXCD1")
        .WithFlag(0, FieldMode.Write, name: "QTHRXCD0");

      RegistersCollection
        .DefineRegister((long) Registers.EEval)
        .WithReservedBits(1, 31)
        .WithFlag(0, FieldMode.Write, name: "EVAL");

      // DRAE0, DRAEH0 -> DRAE7, DRAEH7
      for (int i = 0; i < 8; i++)
      {
        int offset = i * sizeof(uint) * 2;

        RegistersCollection
          .DefineRegister((long) Registers.DRAE0 + offset)
          .WithFlags(0, 32, FieldMode.Read | FieldMode.Write, name: "En");

        RegistersCollection
          .DefineRegister((long) Registers.DRAEH0 + offset)
          .WithFlags(0, 32, FieldMode.Read | FieldMode.Write, name: "En");
      }

      // QRAE_0 -> QRAE_7
      for (int i = 0; i < 8; i++)
      {
        RegistersCollection
          .DefineRegister((long) Registers.QRAE0 + i * sizeof(uint))
          .WithReservedBits(8, 24)
          .WithFlags(0, 8, FieldMode.Read | FieldMode.Write, name: "En");
      }

      // Q0E0 -> Q2E15
      for (int q = 0; q < 3; q++)
      {
        int qOffset = q * 16 * sizeof(uint);

        for (int e = 0; e < 16; e++)
        {
          long qeOffset = (long) Registers.Q0E0 + qOffset + e * sizeof(uint);

          RegistersCollection
            .DefineRegister(qeOffset)
            .WithReservedBits(8, 24)
            .WithValueField(6, 2, FieldMode.Read, name: "ETYPE")
            .WithValueField(0, 6, FieldMode.Read, name: "ENUM");
        }
      }

      // QSTAT_0 -> QSTAT_2
      for (int i = 0; i < 3; i++)
      {
        long offset = (long) Registers.QStat0 + i * sizeof(uint);

        RegistersCollection
          .DefineRegister(offset, resetValue: 0x0000000F)
          .WithReservedBits(25, 7)
          .WithFlag(24, FieldMode.Read, name: "THRXCD")
          .WithReservedBits(21, 3)
          .WithValueField(16, 5, FieldMode.Read, name: "WM")
          .WithReservedBits(13, 3)
          .WithValueField(8, 5, FieldMode.Read, name: "NUMVAL")
          .WithReservedBits(4, 4)
          .WithValueField(0, 4, FieldMode.Read, name: "STRTPTR");
      }

      RegistersCollection
        .DefineRegister((long) Registers.QWmThrA, resetValue: 0x000A0A0A)
        .WithReservedBits(21, 11)
        .WithValueField(16, 5, FieldMode.Read | FieldMode.Write, name: "Q2")
        .WithReservedBits(13, 3)
        .WithValueField(8, 5, FieldMode.Read | FieldMode.Write, name: "Q1")
        .WithReservedBits(5, 3)
        .WithValueField(0, 5, FieldMode.Read | FieldMode.Write, name: "Q0");

      RegistersCollection
        .DefineRegister((long) Registers.CCStat)
        .WithReservedBits(19, 13)
        .WithFlag(18, FieldMode.Read, name: "QUEACTV2")
        .WithFlag(17, FieldMode.Read, name: "QUEACTV1")
        .WithFlag(16, FieldMode.Read, name: "QUEACTV0")
        .WithReservedBits(14, 2)
        .WithValueField(8, 6, FieldMode.Read, name: "COMPACTV")
        .WithReservedBits(5, 3)
        .WithFlag(4, FieldMode.Read, name: "ACTV")
        .WithReservedBits(3, 1)
        .WithFlag(2, FieldMode.Read, name: "TRACTV")
        .WithFlag(1, FieldMode.Read, name: "QEVTACTV")
        .WithFlag(0, FieldMode.Read, name: "EVTACTV");

      RegistersCollection
        .DefineRegister((long) Registers.MPFAR)
        .WithValueField(0, 32, FieldMode.Read, name: "FADDR");

      RegistersCollection
        .DefineRegister((long) Registers.MPFSR)
        .WithReservedBits(13, 19)
        .WithValueField(9, 4, FieldMode.Read, name: "FID")
        .WithReservedBits(6, 3)
        .WithFlag(5, FieldMode.Read, name: "SRE")
        .WithFlag(4, FieldMode.Read, name: "SWE")
        .WithFlag(3, FieldMode.Read, name: "SXE")
        .WithFlag(2, FieldMode.Read, name: "URE")
        .WithFlag(1, FieldMode.Read, name: "UWE")
        .WithFlag(0, FieldMode.Read, name: "UXE");

      RegistersCollection
        .DefineRegister((long) Registers.MPFCR)
        .WithReservedBits(1, 31)
        .WithFlag(0, FieldMode.Write, name: "MPFCLR");

      void DefineMPPARegister(long offset)
      {
        RegistersCollection
          .DefineRegister((long) Registers.MPPAG, resetValue: 0x676)
          .WithReservedBits(16, 16)
          .WithValueField(10, 6, FieldMode.Read | FieldMode.Write, name: "AIDm")
          .WithFlag(9, FieldMode.Read | FieldMode.Write, name: "EXT")
          .WithReservedBits(6, 3)
          .WithFlag(5, FieldMode.Read | FieldMode.Write, name: "SR")
          .WithFlag(4, FieldMode.Read | FieldMode.Write, name: "SW")
          .WithFlag(3, FieldMode.Read | FieldMode.Write, name: "SX")
          .WithFlag(2, FieldMode.Read | FieldMode.Write, name: "UR")
          .WithFlag(1, FieldMode.Read | FieldMode.Write, name: "UW")
          .WithFlag(0, FieldMode.Read | FieldMode.Write, name: "UX");
      }

      DefineMPPARegister((long) Registers.MPPAG);

      // MPPA_0 -> MPPA_7
      for (int i = 0; i < 8; i++)
      {
        DefineMPPARegister((long) Registers.MPPA0 + i * sizeof(uint));
      }

      RegistersCollection
        .DefineRegister((long) Registers.ER)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.ERH)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.ECR)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.ECRH)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.ESR)
        .WithFlags(0, 32, FieldMode.Read | FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.ESRH)
        .WithFlags(0, 32, FieldMode.Read | FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.CER)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.CERH)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EER)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EERH)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EECR)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EECRH)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EESR)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.EESRH)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.SER)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.SERH)
        .WithFlags(0, 32, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.SECR)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.SECRH)
        .WithFlags(0, 32, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.IER)
        .WithFlags(0, 32, FieldMode.Read, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IERH)
        .WithFlags(0, 32, FieldMode.Read, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IECR)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IECRH)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IESR)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IESRH)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IPR)
        .WithFlags(0, 32, FieldMode.Read, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IPRH)
        .WithFlags(0, 32, FieldMode.Read, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.ICR)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.ICRH)
        .WithFlags(0, 32, FieldMode.Write, name: "In");

      RegistersCollection
        .DefineRegister((long) Registers.IEval)
        .WithReservedBits(1, 31)
        .WithFlag(0, FieldMode.Write, name: "EVAL");

      RegistersCollection
        .DefineRegister((long) Registers.QER)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QEER)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QEECR)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QEESR)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Write, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QSER)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Read, name: "En");

      RegistersCollection
        .DefineRegister((long) Registers.QSECR)
        .WithReservedBits(8, 24)
        .WithFlags(0, 8, FieldMode.Write, name: "En");
    }

    private void DefineQueueRegister(long offset)
    {
      DoubleWordRegister reg = RegistersCollection.DefineRegister(offset);

      for (int j = 7; j >= 0; j--)
      {
        int bitOffset = j * 4;
        reg
          .WithReservedBits(bitOffset + 3, 1)
          .WithValueField(
            bitOffset, 3,
            FieldMode.Read | FieldMode.Write,
            name: $"E{j}"
          );
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

    public DoubleWordRegisterCollection RegistersCollection { get; }
    public long Size => 1 << 20;

    private List<EDMA3TransferController> transferControllers;

    public enum Registers
    {
      PID = 0x0000,
      CCCfg = 0x0004,
      SysConfig = 0x0010,
      DChMap0 = 0x0100,
      QChMap0 = 0x0200,
      DmaQNum0 = 0x0240,
      QDmaQNum = 0x0260,
      QuePri = 0x0284,
      EMR = 0x0300,
      EMRH = 0x0304,
      EMCR = 0x0308,
      EMCRH = 0x030C,
      QEMR = 0x0310,
      QEMCR = 0x0314,
      CCErr = 0x0318,
      CCErrClr = 0x031C,
      EEval = 0x0320,
      DRAE0 = 0x0340,
      DRAEH0 = 0x0344,
      QRAE0 = 0x0380,
      Q0E0 = 0x0400,
      QStat0 = 0x0600,
      QWmThrA = 0x0620,
      CCStat = 0x0640,
      MPFAR = 0x0800,
      MPFSR = 0x0804,
      MPFCR = 0x0808,
      MPPAG = 0x080C,
      MPPA0 = 0x0810,
      ER = 0x1000,
      ERH = 0x1004,
      ECR = 0x1008,
      ECRH = 0x100C,
      ESR = 0x1010,
      ESRH = 0x1014,
      CER = 0x1018,
      CERH = 0x101C,
      EER = 0x1020,
      EERH = 0x1024,
      EECR = 0x1028,
      EECRH = 0x102C,
      EESR = 0x1030,
      EESRH = 0x1034,
      SER = 0x1038,
      SERH = 0x103C,
      SECR = 0x1040,
      SECRH = 0x1044,
      IER = 0x1050,
      IERH = 0x1054,
      IECR = 0x1058,
      IECRH = 0x105C,
      IESR = 0x1060,
      IESRH = 0x1064,
      IPR = 0x1068,
      IPRH = 0x106C,
      ICR = 0x1070,
      ICRH = 0x1074,
      IEval = 0x1078,
      QER = 0x1080,
      QEER = 0x1084,
      QEECR = 0x1088,
      QEESR = 0x108C,
      QSER = 0x1090,
      QSECR = 0x1094,
    }
  }
}

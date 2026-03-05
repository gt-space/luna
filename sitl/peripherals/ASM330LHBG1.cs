using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.SPI;

const FieldMode RW = FieldMode.Read | FieldMode.Write;

namespace Antmicro.Renode.Peripherals.Sensors
{
  public class ASM330LHBG1 :
    ISPIPeripheral,
    IProvidesRegisterCollection<ByteRegisterCollection>
  {
    public ASM330LHBG1(IMachine machine)
    {
      this.machine = machine;

      RegistersCollection = new ByteRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    public void Reset()
    {
      RegistersCollection.Reset();
      expectingAddress = true;
      currentAddress = 0;
      readTransfer = false;
      autoIncrement = false;
    }

    // Called in sequence for every byte of a SPI transfer.
    public byte Transmit(byte data)
    {
      if (expectingAddress)
      {
        readTransfer = (data & 0x80) != 0;
        currentAddress = (byte)(data & 0x7F);
        autoIncrement = ((data & 0x40) != 0) || (ifInc?.Value ?? false);
        expectingAddress = false;
        return 0xFF;
      }

      if (readTransfer)
      {
        var value = RegistersCollection.Read(currentAddress);
        if (autoIncrement)
        {
          currentAddress++;
        }
        return value;
      }

      RegistersCollection.Write(currentAddress, data);
      if (autoIncrement)
      {
        currentAddress++;
      }
      return 0xFF;
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      expectingAddress = true;
    }

    public void SetGyroscopeRaw(short x, short y, short z)
    {
      WriteVector(x, gyroXLow, gyroXHigh);
      WriteVector(y, gyroYLow, gyroYHigh);
      WriteVector(z, gyroZLow, gyroZHigh);
      gyroDataReady.Value = true;
    }

    public void SetAccelerationRaw(short x, short y, short z)
    {
      WriteVector(x, accelXLow, accelXHigh);
      WriteVector(y, accelYLow, accelYHigh);
      WriteVector(z, accelZLow, accelZHigh);
      accelDataReady.Value = true;
    }

    public void SetTemperatureRaw(short temp)
    {
      WriteVector(temp, tempLow, tempHigh);
      tempDataReady.Value = true;
    }

    public ByteRegisterCollection RegistersCollection { get; private set; }

    private void DefineRegisters()
    {
      FieldMode R = FieldMode.Read;
      FieldMode W = FieldMode.Write;
      FieldMode RW = R | W;

      RegistersCollection
        .DefineRegister((long) Register.FUNC_CFG_ACCESS, resetValue: 0x00)
        .WithFlag(7, RW, name: "FUNC_CFG_ACCESS")
        .WithReservedBits(0, 7);

      RegistersCollection
        .DefineRegister((long) Register.PIN_CTRL, resetValue: 0x3F)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "SDO_PU_EN")
        .WithReservedBits(0, 6);

      RegistersCollection
        .DefineRegister((long) Register.FIFO_CTRL1, resetValue: 0x00)
        .WithValueField(0, 8, RW, name: "WTM[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_CTRL2, resetValue: 0x00)
        .WithFlag(7, RW, name: "STOP_ON_WTM")
        .WithReservedBits(5, 2)
        .WithFlag(4, RW, name: "ODRCHG_EN")
        .WithReservedBits(1, 3)
        .WithFlag(0, RW, name: "WTM8");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_CTRL3, resetValue: 0x00)
        .WithValueField(4, 4, RW, name: "BDR_GY")
        .WithValueField(0, 4, RW, name: "BDR_XL");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_CTRL4, resetValue: 0x00)
        .WithValueField(6, 2, RW, name: "DEC_TS_BATCH")
        .WithValueField(4, 2, RW, name: "ODR_T_BATCH")
        .WithReservedBits(3, 1)
        .WithValueField(0, 3, RW, name: "FIFO_MODE");

      RegistersCollection
        .DefineRegister((long) Register.COUNTER_BDR_REG1, resetValue: 0x00)
        .WithFlag(7, RW, name: "dataready_pulsed")
        .WithFlag(6, RW, name: "RST_COUNTER_BDR")
        .WithFlag(5, RW, name: "TRIG_COUNTER_BDR")
        .WithReservedBits(3, 2)
        .WithValueField(0, 3, RW, name: "CNT_BDR_TH[10:8]");

      RegistersCollection
        .DefineRegister((long) Register.COUNTER_BDR_REG2, resetValue: 0x00)
        .WithValueField(0, 8, RW, name: "CNT_BDR_TH[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.INT1_CTRL, resetValue: 0x00)
        .WithFlag(7, RW, name: "DEN_DRDY_flag")
        .WithFlag(6, RW, name: "INT1_CNT_BDR")
        .WithFlag(5, RW, name: "INT1_FIFO_FULL")
        .WithFlag(4, RW, name: "INT1_FIFO_OVR")
        .WithFlag(3, RW, name: "INT1_FIFO_TH")
        .WithFlag(2, RW, name: "INT1_BOOT")
        .WithFlag(1, RW, name: "INT1_DRDY_G")
        .WithFlag(0, RW, name: "INT1_DRDY_XL");

      RegistersCollection
        .DefineRegister((long) Register.INT2_CTRL, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "INT2_CNT_BDR")
        .WithFlag(5, RW, name: "INT2_FIFO_FULL")
        .WithFlag(4, RW, name: "INT2_FIFO_OVR")
        .WithFlag(3, RW, name: "INT2_FIFO_TH")
        .WithFlag(2, RW, name: "INT2_DRDY_TEMP")
        .WithFlag(1, RW, name: "INT2_DRDY_G")
        .WithFlag(0, RW, name: "INT2_DRDY_XL");

      RegistersCollection
        .DefineRegister((long) Register.WHO_AM_I, resetValue: 0x6B);

      RegistersCollection
        .DefineRegister((long) Register.CTRL1_XL, resetValue: 0x00)
        .WithValueField(4, 4, RW, name: "ODR_XL")
        .WithValueField(2, 2, RW, name: "FS_XL")
        .WithFlag(1, RW, name: "LPF2_XL_EN")
        .WithReservedBits(0, 1);

      RegistersCollection
        .DefineRegister((long) Register.CTRL2_G, resetValue: 0x00)
        .WithValueField(4, 4, RW, name: "ODR_G")
        .WithValueField(2, 2, RW, name: "FS_G")
        .WithFlag(1, RW, name: "FS_125")
        .WithFlag(0, RW, name: "FS_4000");

      RegistersCollection
        .DefineRegister((long) Register.CTRL3_C, resetValue: 0x04)
        .WithFlag(7, RW, name: "BOOT")
        .WithFlag(6, RW, name: "BDU")
        .WithFlag(5, RW, name: "H_LACTIVE")
        .WithFlag(4, RW, name: "PP_OD")
        .WithFlag(3, RW, name: "SIM")
        .WithFlag(2, RW, name: "IF_INC")
        .WithReservedBits(1, 1)
        .WithFlag(0, RW, name: "SW_RESET");

      RegistersCollection
        .DefineRegister((long) Register.CTRL4_C, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "SLEEP_G")
        .WithFlag(5, RW, name: "INT2_on_INT1")
        .WithReservedBits(4, 1)
        .WithFlag(3, RW, name: "DRDY_MASK")
        .WithFlag(2, RW, name: "I2C_disable")
        .WithFlag(1, RW, name: "LPF1_SEL_G")
        .WithReservedBits(0, 1);

      RegistersCollection
        .DefineRegister((long) Register.CTRL5_C, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithValueField(5, 2, RW, name: "ROUNDING")
        .WithReservedBits(4, 1)
        .WithValueField(2, 2, RW, name: "ST_G")
        .WithValueField(0, 2, RW, name: "ST_XL");

      RegistersCollection
        .DefineRegister((long) Register.CTRL6_C, resetValue: 0x00)
        .WithFlag(7, RW, name: "TRIG_EN")
        .WithFlag(6, RW, name: "LVL1_EN")
        .WithFlag(5, RW, name: "LVL2_EN")
        .WithFlag(4, RW, name: "XL_HM_MODE")
        .WithFlag(3, RW, name: "USR_OFF_W")
        .WithValueField(0, 3, RW, name: "FTYPE");

      RegistersCollection
        .DefineRegister((long) Register.CTRL7_G, resetValue: 0x00)
        .WithFlag(7, RW, name: "G_HM_MODE")
        .WithFlag(6, RW, name: "HP_EN_G")
        .WithValueField(4, 2, RW, name: "HPM_G")
        .WithReservedBits(2, 2)
        .WithFlag(1, RW, name: "USR_OFF_ON_OUT")
        .WithReservedBits(0, 1);

      RegistersCollection
        .DefineRegister((long) Register.CTRL8_XL, resetValue: 0x00)
        .WithValueField(5, 3, RW, name: "HPCF_XL")
        .WithFlag(4, RW, name: "HP_REF_MODE_XL")
        .WithFlag(3, RW, name: "FASTSETTL_MODE_XL")
        .WithFlag(2, RW, name: "HP_SLOPE_XL_EN")
        .WithReservedBits(1, 1)
        .WithFlag(0, RW, name: "LOW_PASS_ON_6D");

      RegistersCollection
        .DefineRegister((long) Register.CTRL9_XL, resetValue: 0xE0)
        .WithFlag(7, RW, name: "DEN_X")
        .WithFlag(6, RW, name: "DEN_Y")
        .WithFlag(5, RW, name: "DEN_Z")
        .WithFlag(4, RW, name: "DEN_XL_G")
        .WithFlag(3, RW, name: "DEN_XL_EN")
        .WithFlag(2, RW, name: "DEN_LH")
        .WithFlag(1, RW, name: "I3C_disable")
        .WithReservedBits(0, 1);

      RegistersCollection
        .DefineRegister((long) Register.CTRL10_C, resetValue: 0x00)
        .WithReservedBits(6, 2)
        .WithFlag(5, RW, name: "TIMESTAMP_EN")
        .WithReservedBits(0, 5);

      RegistersCollection
        .DefineRegister((long) Register.ALL_INT_SRC, resetValue: 0x00)
        .WithFlag(7, R, name: "TIMESTAMP_ENDCOUNT")
        .WithReservedBits(6, 1)
        .WithFlag(5, R, name: "SLEEP_CHANGE_IA")
        .WithFlag(4, R, name: "D6D_IA")
        .WithReservedBits(2, 2)
        .WithFlag(1, R, name: "WU_IA")
        .WithFlag(0, R, name: "FF_IA");

      RegistersCollection
        .DefineRegister((long) Register.WAKE_UP_SRC, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithFlag(6, R, name: "SLEEP_CHANGE_IA")
        .WithFlag(5, R, name: "FF_IA")
        .WithFlag(4, R, name: "SLEEP_STATE")
        .WithFlag(3, R, name: "WU_IA")
        .WithFlag(2, R, name: "X_WU")
        .WithFlag(1, R, name: "Y_WU")
        .WithFlag(0, R, name: "Z_WU");

      RegistersCollection
        .DefineRegister((long) Register.D6D_SRC, resetValue: 0x00)
        .WithFlag(7, R, name: "DEN_DRDY")
        .WithFlag(6, R, name: "D6D_IA")
        .WithFlag(5, R, name: "ZH")
        .WithFlag(4, R, name: "ZL")
        .WithFlag(3, R, name: "YH")
        .WithFlag(2, R, name: "YL")
        .WithFlag(1, R, name: "XH")
        .WithFlag(0, R, name: "XL");

      RegistersCollection
        .DefineRegister((long) Register.STATUS_REG, resetValue: 0x00)
        .WithReservedBits(4, 4)
        .WithFlag(3, R, name: "BOOT_CHECK_FAIL")
        .WithFlag(2, out tempDataReady, R, name: "TDA")
        .WithFlag(1, out gyroDataReady, R, name: "GDA")
        .WithFlag(0, out accelDataReady, R, name: "XLDA");

      // TODO: come back here and do closures

      RegistersCollection
        .DefineRegister((long) Register.OUT_TEMP_L, resetValue: 0x00)
        .WithValueField(0, 8, out tempLow, FieldMode.Read, name: "OUT_TEMP_L");

      RegistersCollection
        .DefineRegister((long) Register.OUT_TEMP_H, resetValue: 0x00)
        .WithValueField(0, 8, out tempHigh, FieldMode.Read, name: "OUT_TEMP_H");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroXLow, FieldMode.Read, name: "OUTX_L_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroXHigh, FieldMode.Read, name: "OUTX_H_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTY_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroYLow, FieldMode.Read, name: "OUTY_L_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTY_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroYHigh, FieldMode.Read, name: "OUTY_H_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_L_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroZLow, FieldMode.Read, name: "OUTZ_L_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_H_G, resetValue: 0x00)
        .WithValueField(0, 8, out gyroZHigh, FieldMode.Read, name: "OUTZ_H_G");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelXLow, FieldMode.Read, name: "OUTX_L_A");

      RegistersCollection
        .DefineRegister((long) Register.OUTX_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelXHigh, FieldMode.Read, name: "OUTX_H_A");

      RegistersCollection
        .DefineRegister((long) Register.OUTY_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelYLow, FieldMode.Read, name: "OUTY_L_A");

      RegistersCollection
        .DefineRegister((long) Register.OUTY_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelYHigh, FieldMode.Read, name: "OUTY_H_A");

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_L_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelZLow, FieldMode.Read, name: "OUTZ_L_A");

      RegistersCollection
        .DefineRegister((long) Register.OUTZ_H_A, resetValue: 0x00)
        .WithValueField(0, 8, out accelZHigh, FieldMode.Read, name: "OUTZ_H_A");

      RegistersCollection
        .DefineRegister((long) Register.EMB_FUNC_STATUS_MAINPAGE, resetValue: 0x00)
        .WithFlag(7, R, name: "IS_FSM_LC")
        .WithReservedBits(0, 6);

      RegistersCollection
        .DefineRegister((long) Register.FSM_STATUS_A_MAINPAGE, resetValue: 0x00)
        .WithFlag(7, R, name: "IS_FSM8")
        .WithFlag(6, R, name: "IS_FSM7")
        .WithFlag(5, R, name: "IS_FSM6")
        .WithFlag(4, R, name: "IS_FSM5")
        .WithFlag(3, R, name: "IS_FSM4")
        .WithFlag(2, R, name: "IS_FSM3")
        .WithFlag(1, R, name: "IS_FSM2")
        .WithFlag(0, R, name: "IS_FSM1");

      RegistersCollection
        .DefineRegister((long) Register.FSM_STATUS_B_MAINPAGE, resetValue: 0x00)
        .WithFlag(7, R, name: "IS_FSM16")
        .WithFlag(6, R, name: "IS_FSM15")
        .WithFlag(5, R, name: "IS_FSM14")
        .WithFlag(4, R, name: "IS_FSM13")
        .WithFlag(3, R, name: "IS_FSM12")
        .WithFlag(2, R, name: "IS_FSM11")
        .WithFlag(1, R, name: "IS_FSM10")
        .WithFlag(0, R, name: "IS_FSM9");

      RegistersCollection
        .DefineRegister((long) Register.MLC_STATUS_MAINPAGE, resetValue: 0x00)
        .WithFlag(7, R, name: "IS_MLC8")
        .WithFlag(6, R, name: "IS_MLC7")
        .WithFlag(5, R, name: "IS_MLC6")
        .WithFlag(4, R, name: "IS_MLC5")
        .WithFlag(3, R, name: "IS_MLC4")
        .WithFlag(2, R, name: "IS_MLC3")
        .WithFlag(1, R, name: "IS_MLC2")
        .WithFlag(0, R, name: "IS_MLC1");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_STATUS1, resetValue: 0x00)
        .WithValueField(0, 8, R, name: "DIFF_FIFO[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_STATUS2, resetValue: 0x00)
        .WithFlag(7, R, name: "FIFO_WTM_IA")
        .WithFlag(6, R, name: "FIFO_OVR_IA")
        .WithFlag(5, R, name: "FIFO_FULL_IA")
        .WithFlag(4, R, name: "COUNTER_BDR_IA")
        .WithFlag(3, R, name: "FIFO_OVR_LATCHED")
        .WithReservedBits(2, 1)
        .WithValueField(0, 2, R, name: "DIFF_FIFO[9:8]");

      RegistersCollection
        .DefineRegister((long) Register.TIMESTAMP0, resetValue: 0x00)
        .WithValueField(0, 8, R, name: "D[31:24]");

      RegistersCollection
        .DefineRegister((long) Register.TIMESTAMP1, resetValue: 0x00)
        .WithValueField(0, 8, R, name: "D[23:16]");

      RegistersCollection
        .DefineRegister((long) Register.TIMESTAMP2, resetValue: 0x00)
        .WithValueField(0, 8, R, name: "D[15:8]");

      RegistersCollection
        .DefineRegister((long) Register.TIMESTAMP3, resetValue: 0x00)
        .WithValueField(0, 8, R, name: "D[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.INT_CFG0, resetValue: 0x00)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "INT_CLR_ON_READ")
        .WithFlag(5, RW, name: "SLEEP_STATUS_ON_INT")
        .WithFlag(4, RW, name: "SLOPE_FDS")
        .WithReservedBits(1, 3)
        .WithFlag(0, RW, name: "LIR");

      RegistersCollection
        .DefineRegister((long) Register.INT_CFG1, resetValue: 0x00)
        .WithFlag(7, RW, name: "INTERRUPTS_ENABLE")
        .WithValueField(5, 2, RW, name: "INACT_EN")
        .WithReservedBits(0, 5);

      RegistersCollection
        .DefineRegister((long) Register.THS_6D)
        .WithFlag(7, RW, name: "D4D_EN")
        .WithValueField(5, 2, RW, name: "SIXD_THS")
        .WithReservedBits(0, 5);

      RegistersCollection
        .DefineRegister((long) Register.WAKE_UP_THS)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "USR_OFF_ON_WU")
        .WithValueField(0, 6, RW, name: "WK_THS");

      RegistersCollection
        .DefineRegister((long) Register.WAKE_UP_DUR)
        .WithValueField(7, 1, RW, name: "FF_DUR5")
        .WithValueField(5, 2, RW, name: "WAKE_DUR")
        .WithFlag(4, RW, name: "WAKE_THS_W")
        .WithValueField(0, 4, RW, name: "SLEEP_DUR");

      RegistersCollection
        .DefineRegister((long) Register.FREE_FALL)
        .WithValueField(3, 5, RW, name: "FF_DUR[4:0]")
        .WithValueField(0, 3, RW, name: "FF_THS");

      RegistersCollection
        .DefineRegister((long) Register.MD1_CFG)
        .WithFlag(7, RW, name: "INT1_SLEEP_CHANGE")
        .WithReservedBits(6, 1)
        .WithFlag(5, RW, name: "INT1_WU")
        .WithFlag(4, RW, name: "INT1_FF")
        .WithReservedBits(3, 1)
        .WithFlag(2, RW, name: "INT1_6D")
        .WithFlag(1, RW, name: "INT1_EMB_FUNC")
        .WithReservedBits(0, 1);

      RegistersCollection
        .DefineRegister((long) Register.MD2_CFG)
        .WithFlag(7, RW, name: "INT2_SLEEP_CHANGE")
        .WithReservedBits(6, 1)
        .WithFlag(5, RW, name: "INT2_WU")
        .WithFlag(4, RW, name: "INT2_FF")
        .WithReservedBits(3, 1)
        .WithFlag(2, RW, name: "INT2_6D")
        .WithFlag(1, RW, name: "INT2_EMB_FUNC")
        .WithFlag(0, RW, name: "INT2_TIMESTAMP");

      RegistersCollection
        .DefineRegister((long) Register.I3C_BUS_AVB)
        .WithReservedBits(5, 3)
        .WithValueField(3, 2, RW, name: "I3C_Bus_Avb_Sel")
        .WithReservedBits(1, 2)
        .WithFlag(0, RW, name: "PD_DIS_INT1");

      RegistersCollection
        .DefineRegister((long) Register.INTERNAL_FREQ_FINE)
        .WithValueField(0, 8, R, name: "FREQ_FINE");

      RegistersCollection
        .DefineRegister((long) Register.X_OFS_USR)
        .WithValueField(0, 8, RW, name: "X_OFS_USR");

      RegistersCollection
        .DefineRegister((long) Register.Y_OFS_USR)
        .WithValueField(0, 8, RW, name: "Y_OFS_USR");

      RegistersCollection
        .DefineRegister((long) Register.Z_OFS_USR)
        .WithValueField(0, 8, RW, name: "Z_OFS_USR");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_TAG)
        .WithValueField(3, 5, R, name: "TAG_SENSOR")
        .WithValueField(1, 2, R, name: "TAG_CNT")
        .WithFlag(0, R, name: "TAG_PARITY");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_X_H)
        .WithValueField(0, 8, R, name: "D[15:8]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_X_L)
        .WithValueField(0, 8, R, name: "D[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_Y_H)
        .WithValueField(0, 8, R, name: "D[15:8]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_Y_L)
        .WithValueField(0, 8, R, name: "D[7:0]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_Z_H)
        .WithValueField(0, 8, R, name: "D[15:8]");

      RegistersCollection
        .DefineRegister((long) Register.FIFO_DATA_OUT_Z_L)
        .WithValueField(0, 8, R, name: "D[7:0]");
    }

    private static void WriteVector(short value, IValueRegisterField low, IValueRegisterField high)
    {
      unchecked
      {
        low.Value = (byte)(value & 0xFF);
        high.Value = (byte)((value >> 8) & 0xFF);
      }
    }

    private IMachine machine;
    private bool expectingAddress;
    private bool readTransfer;
    private bool autoIncrement;
    private byte currentAddress;

    private IFlagRegisterField ifInc = null!;
    private IFlagRegisterField accelDataReady = null!;
    private IFlagRegisterField gyroDataReady = null!;
    private IFlagRegisterField tempDataReady = null!;

    private IValueRegisterField tempLow = null!;
    private IValueRegisterField tempHigh = null!;
    private IValueRegisterField gyroXLow = null!;
    private IValueRegisterField gyroXHigh = null!;
    private IValueRegisterField gyroYLow = null!;
    private IValueRegisterField gyroYHigh = null!;
    private IValueRegisterField gyroZLow = null!;
    private IValueRegisterField gyroZHigh = null!;
    private IValueRegisterField accelXLow = null!;
    private IValueRegisterField accelXHigh = null!;
    private IValueRegisterField accelYLow = null!;
    private IValueRegisterField accelYHigh = null!;
    private IValueRegisterField accelZLow = null!;
    private IValueRegisterField accelZHigh = null!;

    private enum Register
    {
      FUNC_CFG_ACCESS = 0x01,
      PIN_CTRL = 0x02,
      FIFO_CTRL1 = 0x07,
      FIFO_CTRL2 = 0x08,
      FIFO_CTRL3 = 0x09,
      FIFO_CTRL4 = 0x0A,
      COUNTER_BDR_REG1 = 0x0B,
      COUNTER_BDR_REG2 = 0x0C,
      INT1_CTRL = 0x0D,
      INT2_CTRL = 0x0E,
      WHO_AM_I = 0x0F,
      CTRL1_XL = 0x10,
      CTRL2_G = 0x11,
      CTRL3_C = 0x12,
      CTRL4_C = 0x13,
      CTRL5_C = 0x14,
      CTRL6_C = 0x15,
      CTRL7_G = 0x16,
      CTRL8_XL = 0x17,
      CTRL9_XL = 0x18,
      CTRL10_C = 0x19,
      ALL_INT_SRC = 0x1A,
      WAKE_UP_SRC = 0x1B,
      D6D_SRC = 0x1D,
      STATUS_REG = 0x1E,
      OUT_TEMP_L = 0x20,
      OUT_TEMP_H = 0x21,
      OUTX_L_G = 0x22,
      OUTX_H_G = 0x23,
      OUTY_L_G = 0x24,
      OUTY_H_G = 0x25,
      OUTZ_L_G = 0x26,
      OUTZ_H_G = 0x27,
      OUTX_L_A = 0x28,
      OUTX_H_A = 0x29,
      OUTY_L_A = 0x2A,
      OUTY_H_A = 0x2B,
      OUTZ_L_A = 0x2C,
      OUTZ_H_A = 0x2D,
      EMB_FUNC_STATUS_MAINPAGE = 0x35,
      FSM_STATUS_A_MAINPAGE = 0x36,
      FSM_STATUS_B_MAINPAGE = 0x37,
      MLC_STATUS_MAINPAGE = 0x38,
      FIFO_STATUS1 = 0x3A,
      FIFO_STATUS2 = 0x3B,
      TIMESTAMP0 = 0x40,
      TIMESTAMP1 = 0x41,
      TIMESTAMP2 = 0x42,
      TIMESTAMP3 = 0x43,
      INT_CFG0 = 0x56,
      INT_CFG1 = 0x58,
      THS_6D = 0x59,
      WAKE_UP_THS = 0x5B,
      WAKE_UP_DUR = 0x5C,
      FREE_FALL = 0x5D,
      MD1_CFG = 0x5E,
      MD2_CFG = 0x5F,
      I3C_BUS_AVB = 0x62,
      INTERNAL_FREQ_FINE = 0x63,
      X_OFS_USR = 0x73,
      Y_OFS_USR = 0x74,
      Z_OFS_USR = 0x75,
      FIFO_DATA_OUT_TAG = 0x78,
      FIFO_DATA_OUT_X_L = 0x79,
      FIFO_DATA_OUT_X_H = 0x7A,
      FIFO_DATA_OUT_Y_L = 0x7B,
      FIFO_DATA_OUT_Y_H = 0x7C,
      FIFO_DATA_OUT_Z_L = 0x7D,
      FIFO_DATA_OUT_Z_H = 0x7E,
    }

    private enum EmbeddedRegister
    {
      PAGE_SEL = 0x02,
      EMB_FUNC_EN_B = 0x05,
      PAGE_ADDRESS = 0x08,
      PAGE_VALUE = 0x09,
      EMB_FUNC_INT1 = 0x0A,
      FSM_INT1_A = 0x0B,
      FSM_INT1_B = 0x0C,
      MLC_INT1 = 0x0D,
      EMB_FUNC_INT2 = 0x0E,
      FSM_INT2_A = 0x0F,
      FSM_INT2_B = 0x10,
      MLC_INT2 = 0x11,
      EMB_FUNC_STATUS = 0x12,
      FSM_STATUS_A = 0x13,
      FSM_STATUS_B = 0x14,
      MLC_STATUS = 0x15,
      PAGE_RW = 0x17,
      FSM_ENABLE_A = 0x46,
      FSM_ENABLE_B = 0x47,
      FSM_LONG_COUNTER_L = 0x48,
      FSM_LONG_COUNTER_H = 0x49,
      FSM_LONG_COUNTER_CLEAR = 0x4A,
      FSM_OUTS1 = 0x4C,
      FSM_OUTS2 = 0x4D,
      FSM_OUTS3 = 0x4E,
      FSM_OUTS4 = 0x4F,
      FSM_OUTS5 = 0x50,
      FSM_OUTS6 = 0x51,
      FSM_OUTS7 = 0x52,
      FSM_OUTS8 = 0x53,
      FSM_OUTS9 = 0x54,
      FSM_OUTS10 = 0x55,
      FSM_OUTS11 = 0x56,
      FSM_OUTS12 = 0x57,
      FSM_OUTS13 = 0x58,
      FSM_OUTS14 = 0x59,
      FSM_OUTS15 = 0x5A,
      FSM_OUTS16 = 0x5B,
      EMB_FUNC_ODR_CFG_B = 0x5F,
      EMB_FUNC_ODR_CFG_C = 0x60,
      EMB_FUNC_INIT_B = 0x67,
      MLC0_SRC = 0x70,
      MLC1_SRC = 0x71,
      MLC2_SRC = 0x72,
      MLC3_SRC = 0x73,
      MLC4_SRC = 0x74,
      MLC5_SRC = 0x75,
      MLC6_SRC = 0x76,
      MLC7_SRC = 0x77,
    }
  }
}

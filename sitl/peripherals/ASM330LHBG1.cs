using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;
using Antmicro.Renode.Utilities.RESD;

using System;
using System.Collections.Generic;
using System.Numerics;

namespace Antmicro.Renode.Peripherals.Sensors
{
  /// <summary>
  /// ASM330LHG1 6-axis IMU (accelerometer + gyroscope) Renode peripheral.
  /// </summary>
  /// <remarks>
  /// Implemented:
  /// <list type="bullet">
  ///   <item>
  ///     Reading accelerometer, gyroscope, and temperature values directly \
  ///     from the `OUT_.*_[LH]` registers
  ///   </item>
  ///   <item>All registers defined with correct resets</item>
  ///   <item>Embedded register control and switching</item>
  ///   <item>Manual Renode monitor control of environmental conditions</item>
  ///   <item>RESD data stream support</item>
  ///   <item>Block data updates for high-byte sequential readings</item>
  ///   <item>FIFO queuing for accelerometer and gyroscope</item>
  ///   <item>SPI and I2C interfaces</item>
  ///   <item>Full accelerometer and gyroscope parameter configuration</item>
  /// </list>
  ///
  /// Not yet supported:
  /// <list type="bullet">
  ///   <item>Reading timestamps</item>
  ///   <item>Interrupt GPIOs</item>
  ///   <item>Event-detection (free-fall, wake-up, orientation, etc.)</item>
  ///   <item>Machine learning core subsystem</item>
  ///   <item>Programmable finite state machines</item>
  ///   <item>I3C interface</item>
  ///   <item>Post-measurement internal processing filters</item>
  ///   <item>High-performance / low-performance mode distinction</item>
  ///   <item>Modeling of power consumption</item>
  ///   <item>ODR change as FIFO event configuration</item>
  ///   <item>Automatic FIFO mode switching based on event detection</item>
  ///   <item>Gaussian noise applied to sensor measurements</item>
  ///   <item>Software resets and memory wipes using registers</item>
  /// </list>
  /// </remarks>
  public class ASM330LHBG1 :
    BasicBytePeripheral,
    II2CPeripheral,
    ISPIPeripheral,
    ITemperatureSensor,
    IUnderstandRESD
  {
    public ASM330LHBG1(IMachine machine) : base(machine)
    {
      Interrupt1 = new GPIO();
      Interrupt2 = new GPIO();

      fifo = new Fifo(this);

      accelTimer = new LimitTimer(
        machine.ClockSource,
        autoUpdate: true,
        enabled: true,
        eventEnabled: true,
        frequency: 6667,
        localName: "accelTimer",
        owner: this
      );
      accelTimer.LimitReached += OnMeasureAccelerometer;

      gyroTimer = new LimitTimer(
        machine.ClockSource,
        autoUpdate: true,
        enabled: true,
        eventEnabled: true,
        frequency: 6667,
        localName: "gyroTimer",
        owner: this
      );
      gyroTimer.LimitReached += OnMeasureGyroscope;

      tempTimer = new LimitTimer(
        machine.ClockSource,
        autoUpdate: true,
        enabled: true,
        eventEnabled: true,
        frequency: 6667,
        limit: 128, // fixed, per the datasheet
        localName: "tempTimer",
        owner: this
      );
      tempTimer.LimitReached += OnMeasureTemperature;

      batchTimer = new LimitTimer(
        machine.ClockSource,
        autoUpdate: true,
        enabled: true,
        eventEnabled: true,
        frequency: 6667,
        localName: "batchTimer",
        owner: this
      );
      batchTimer.LimitReached += OnCommitBatch;

      DefineRegisters();
      Reset();
    }

    public override void Reset()
    {
      RegistersCollection.Reset();
      address = null;
      isWrite = false;

      // FIFO
      fifo.Clear();
      fifoModeTrigger = false;

      // Reset timer limits.
      accelTimer.Limit = ulong.MaxValue;
      batchTimer.Limit = ulong.MaxValue;
      gyroTimer.Limit = ulong.MaxValue;
      accelTimer.Reset();
      batchTimer.Reset();
      gyroTimer.Reset();

      // Reset batching info.
      accelBatchPeriod = ulong.MaxValue;
      gyroBatchPeriod = ulong.MaxValue;
      accelSampleCount = 0;
      gyroSampleCount = 0;
      batchCounter = 0;

      // Reset latched values (only with BDU).
      latchedTemp = null;
      latchedAccelX = null;
      latchedAccelY = null;
      latchedAccelZ = null;
      latchedGyroX = null;
      latchedGyroY = null;
      latchedGyroZ = null;

      // Reset samples.
      accelSample = new DiscreteSample3D(0, 0, 0);
      gyroSample = new DiscreteSample3D(0, 0, 0);
      tempSample = 0;
    }

    [IrqProvider]
    public GPIO Interrupt1 { get; private set; }

    [IrqProvider]
    public GPIO Interrupt2 { get; private set; }

    public byte Transmit(byte data)
    {
      // Capture and decompose the first byte received as the address + RW byte.
      if (address is null)
      {
        isWrite = (data & 0x80) == 0;
        address = (byte) (data & 0x7F);
        return 0x00;
      }

      byte output = 0x00;

      // Perform read or write, according to the first bit of the address byte.
      if (isWrite)
      {
        WriteByte((long) address, data);
      }
      else
      {
        output = ReadByte((long) address);
      }

      if (autoIncrement.Value)
      {
        address++;
      }

      return output;
    }

    // I2C write
    public void Write(byte[] data)
    {
      if (i2cDisabled.Value)
      {
        this.Log(LogLevel.Warning, "Attempted an I2C write while I2C is disabled.");
        return;
      }

      if (data.Length == 0)
      {
        return;
      }

      address = data[0];

      for (int i = 1; i < data.Length; i++)
      {
        WriteByte((long) address, data[i]);

        if (autoIncrement.Value)
        {
          address++;
        }
      }
    }

    // I2C read
    public byte[] Read(int count)
    {
      byte[] output = new byte[count];

      if (i2cDisabled.Value)
      {
        this.Log(LogLevel.Warning, "Attempted an I2C read while I2C is disabled.");
        return output;
      }

      // By protocol, the address should always be set with a previous write.
      if (address is null)
      {
        this.Log(LogLevel.Warning, "I2C Read called without setting address.");
        return output;
      }

      for (int i = 0; i < count; i++)
      {
        output[i] = ReadByte((long) address);

        if (autoIncrement.Value)
        {
          address++;
        }
      }

      return output;
    }

    // Called when a SPI transfer finishes (chip select pulled high).
    public void FinishTransmission()
    {
      address = null;
    }

    protected override void DefineRegisters()
    {
      FieldMode R = FieldMode.Read;
      FieldMode RW = R | FieldMode.Write;

      Register.FUNC_CFG_ACCESS.Define(this)
        .WithFlag(7, out embeddedRegistersEnabled, RW, name: "FUNC_CFG_EN")
        .WithReservedBits(0, 7);

      Register.PIN_CTRL
        .DefineConditional(
          this,
          () => !embeddedRegistersEnabled.Value,
          resetValue: 0x3F
        )
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "SDO_PU_EN")
        .WithReservedBits(0, 6);

      Register.FIFO_CTRL1.Define(this)
        .WithValueField(
          0, 8, RW, name: "WTM[7:0]",
          valueProviderCallback: _ => (byte) (fifo.Watermark & 0xFF),
          writeCallback: (_, bits) => {
            fifo.Watermark = (fifo.Watermark & ~0xFF) | (int) bits;
          }
        );

      Register.FIFO_CTRL2
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(
          7, RW, name: "STOP_ON_WTM",
          valueProviderCallback: _ => fifo.Watermark == fifo.Capacity,
          writeCallback: (_, stop) => {
            fifo.Capacity = stop ? fifo.Watermark : Fifo.MaxCapacity;
          }
        )
        .WithReservedBits(5, 2)
        .WithFlag(4, out fifoOdrChangeEnabled, RW, name: "ODRCHG_EN")
        .WithReservedBits(1, 3)
        .WithFlag(
          0, RW, name: "WTM8",
          valueProviderCallback: _ => (fifo.Watermark & 0x100) != 0,
          writeCallback: (_, bit8) => {
            fifo.Watermark = (fifo.Watermark & 0xFF) | (bit8 ? 1 : 0) << 8;
          }
        );

      Register.FIFO_CTRL3
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithEnumField<ByteRegister, DataRate>(
          4, 4, RW, name: "BDR_GY",
          valueProviderCallback: _ => DataRateFromPeriod(gyroBatchPeriod),
          writeCallback: (_, bdr) => {
            gyroBatchPeriod = DataRateToPeriod(bdr);
            UpdateBatchTimer();
          }
        )
        .WithEnumField<ByteRegister, DataRate>(
          0, 4, RW, name: "BDR_XL",
          valueProviderCallback: _ => DataRateFromPeriod(accelBatchPeriod),
          writeCallback: (_, bdr) => {
            accelBatchPeriod = DataRateToPeriod(bdr);
            UpdateBatchTimer();
          }
        );

      Register.FIFO_CTRL4
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(6, 2, RW, name: "DEC_TS_BATCH")
        .WithValueField(4, 2, RW, name: "ODR_T_BATCH")
        .WithReservedBits(3, 1)
        .WithEnumField(0, 3, out fifoModeSetting, RW, name: "FIFO_MODE");

      Register.COUNTER_BDR_REG1
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "dataready_pulsed")
        .WithFlag(6, RW, name: "RST_COUNTER_BDR")
        .WithFlag(5, RW, name: "TRIG_COUNTER_BDR")
        .WithReservedBits(3, 2)
        .WithValueField(0, 3, RW, name: "CNT_BDR_TH[10:8]");

      Register.COUNTER_BDR_REG2
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "CNT_BDR_TH[7:0]");

      Register.INT1_CTRL
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "DEN_DRDY_flag")
        .WithFlag(6, RW, name: "INT1_CNT_BDR")
        .WithFlag(5, RW, name: "INT1_FIFO_FULL")
        .WithFlag(4, RW, name: "INT1_FIFO_OVR")
        .WithFlag(3, RW, name: "INT1_FIFO_TH")
        .WithFlag(2, RW, name: "INT1_BOOT")
        .WithFlag(1, RW, name: "INT1_DRDY_G")
        .WithFlag(0, RW, name: "INT1_DRDY_XL");

      Register.INT2_CTRL
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "INT2_CNT_BDR")
        .WithFlag(5, RW, name: "INT2_FIFO_FULL")
        .WithFlag(4, RW, name: "INT2_FIFO_OVR")
        .WithFlag(3, RW, name: "INT2_FIFO_TH")
        .WithFlag(2, RW, name: "INT2_DRDY_TEMP")
        .WithFlag(1, RW, name: "INT2_DRDY_G")
        .WithFlag(0, RW, name: "INT2_DRDY_XL");

      Register.WHO_AM_I
        .DefineConditional(
          this,
          () => !embeddedRegistersEnabled.Value,
          resetValue: 0x6B
        );

      Register.CTRL1_XL
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithEnumField<ByteRegister, DataRate>(
          4, 4,
          RW,
          name: "ODR_XL",
          valueProviderCallback: _ => DataRateFromPeriod(accelTimer.Limit),
          writeCallback: (_, odr) => accelTimer.Limit = DataRateToPeriod(odr)
        )
        .WithEnumField(2, 2, out accelFullScale, RW, name: "FS_XL")
        .WithFlag(1, out accelLpf2Enabled, RW, name: "LPF2_XL_EN")
        .WithReservedBits(0, 1);

      Register.CTRL2_G
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithEnumField<ByteRegister, DataRate>(
          4, 4,
          RW,
          name: "ODR_G",
          valueProviderCallback: _ => DataRateFromPeriod(gyroTimer.Limit),
          writeCallback: (_, odr) => gyroTimer.Limit = DataRateToPeriod(odr)
        )
        .WithEnumField(2, 2, out gyroFullScale, RW, name: "FS_G")
        .WithFlag(1, out gyroFs125, RW, name: "FS_125")
        .WithFlag(0, out gyroFs4000, RW, name: "FS_4000");

      Register.CTRL3_C
        .DefineConditional(
          this,
          () => !embeddedRegistersEnabled.Value,
          resetValue: 0x04
        )
        .WithFlag(7, RW, name: "BOOT")
        .WithFlag(6, out blockDataUpdate, RW, name: "BDU")
        .WithFlag(5, RW, name: "H_LACTIVE")
        .WithFlag(4, RW, name: "PP_OD")
        .WithFlag(3, RW, name: "SIM")
        .WithFlag(2, out autoIncrement, RW, name: "IF_INC")
        .WithReservedBits(1, 1)
        .WithFlag(0, RW, name: "SW_RESET");

      Register.CTRL4_C
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "SLEEP_G")
        .WithFlag(5, RW, name: "INT2_on_INT1")
        .WithReservedBits(4, 1)
        .WithFlag(3, RW, name: "DRDY_MASK")
        .WithFlag(2, out i2cDisabled, RW, name: "I2C_disable")
        .WithFlag(1, RW, name: "LPF1_SEL_G")
        .WithReservedBits(0, 1);

      Register.CTRL5_C
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithReservedBits(7, 1)
        .WithValueField(5, 2, RW, name: "ROUNDING")
        .WithReservedBits(4, 1)
        .WithValueField(2, 2, RW, name: "ST_G")
        .WithValueField(0, 2, RW, name: "ST_XL");

      Register.CTRL6_C
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "TRIG_EN")
        .WithFlag(6, RW, name: "LVL1_EN")
        .WithFlag(5, RW, name: "LVL2_EN")
        .WithFlag(4, RW, name: "XL_HM_MODE")
        .WithFlag(3, RW, name: "USR_OFF_W")
        .WithValueField(0, 3, RW, name: "FTYPE");

      Register.CTRL7_G.Define(this)
        .WithFlag(7, RW, name: "G_HM_MODE")
        .WithFlag(6, RW, name: "HP_EN_G")
        .WithValueField(4, 2, RW, name: "HPM_G")
        .WithReservedBits(2, 2)
        .WithFlag(1, RW, name: "USR_OFF_ON_OUT")
        .WithReservedBits(0, 1);

      Register.CTRL8_XL
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(5, 3, RW, name: "HPCF_XL")
        .WithFlag(4, RW, name: "HP_REF_MODE_XL")
        .WithFlag(3, RW, name: "FASTSETTL_MODE_XL")
        .WithFlag(2, RW, name: "HP_SLOPE_XL_EN")
        .WithReservedBits(1, 1)
        .WithFlag(0, RW, name: "LOW_PASS_ON_6D");

      Register.CTRL9_XL.Define(this, resetValue: 0xE0)
        .WithFlag(7, RW, name: "DEN_X")
        .WithFlag(6, RW, name: "DEN_Y")
        .WithFlag(5, RW, name: "DEN_Z")
        .WithFlag(4, RW, name: "DEN_XL_G")
        .WithFlag(3, RW, name: "DEN_XL_EN")
        .WithFlag(2, RW, name: "DEN_LH")
        .WithFlag(1, RW, name: "I3C_disable")
        .WithReservedBits(0, 1);

      Register.CTRL10_C.Define(this)
        .WithReservedBits(6, 2)
        .WithFlag(5, RW, name: "TIMESTAMP_EN")
        .WithReservedBits(0, 5);

      Register.ALL_INT_SRC.Define(this)
        .WithFlag(7, R, name: "TIMESTAMP_ENDCOUNT")
        .WithReservedBits(6, 1)
        .WithFlag(5, R, name: "SLEEP_CHANGE_IA")
        .WithFlag(4, R, name: "D6D_IA")
        .WithReservedBits(2, 2)
        .WithFlag(1, R, name: "WU_IA")
        .WithFlag(0, R, name: "FF_IA");

      Register.WAKE_UP_SRC.Define(this)
        .WithReservedBits(7, 1)
        .WithFlag(6, R, name: "SLEEP_CHANGE_IA")
        .WithFlag(5, R, name: "FF_IA")
        .WithFlag(4, R, name: "SLEEP_STATE")
        .WithFlag(3, R, name: "WU_IA")
        .WithFlag(2, R, name: "X_WU")
        .WithFlag(1, R, name: "Y_WU")
        .WithFlag(0, R, name: "Z_WU");

      Register.D6D_SRC.Define(this)
        .WithFlag(7, R, name: "DEN_DRDY")
        .WithFlag(6, R, name: "D6D_IA")
        .WithFlag(5, R, name: "ZH")
        .WithFlag(4, R, name: "ZL")
        .WithFlag(3, R, name: "YH")
        .WithFlag(2, R, name: "YL")
        .WithFlag(1, R, name: "XH")
        .WithFlag(0, R, name: "XL");

      Register.STATUS_REG.Define(this)
        .WithReservedBits(4, 4)
        .WithFlag(3, R, name: "BOOT_CHECK_FAIL")
        .WithFlag(2, out tempDataReady, R, name: "TDA")
        .WithFlag(1, out gyroDataReady, R, name: "GDA")
        .WithFlag(0, out accelDataReady, R, name: "XLDA");

      Register.OUT_TEMP_L.Define(this)
        .WithValueField(
          0, 8, R, name: "OUT_TEMP_L",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedTemp = tempSample;
            }

            return (byte) (tempSample & 0xFF);
          }
        );

      Register.OUT_TEMP_H.Define(this)
        .WithValueField(
          0, 8, R, name: "OUT_TEMP_H",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedTemp is not null
              ? (short) latchedTemp
              : tempSample;

            latchedTemp = null;
            tempDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTX_L_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_L_G",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedGyroX = gyroSample.X;
            }

            return (byte) (gyroSample.X & 0xFF);
          }
        );

      Register.OUTX_H_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_H_G",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedGyroX is not null
              ? (short) latchedGyroX
              : gyroSample.X;

            latchedGyroX = null;
            gyroDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTY_L_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_L_G",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedGyroY = gyroSample.Y;
            }

            return (byte) (gyroSample.Y & 0xFF);
          }
        );

      Register.OUTY_H_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_H_G",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedGyroY is not null
              ? (short) latchedGyroY
              : gyroSample.Y;

            latchedGyroY = null;
            gyroDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTZ_L_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_L_G",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedGyroZ = gyroSample.Z;
            }

            return (byte) (gyroSample.Z & 0xFF);
          }
        );

      Register.OUTZ_H_G.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_H_G",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedGyroZ is not null
              ? (short) latchedGyroZ
              : gyroSample.Z;

            latchedGyroZ = null;
            gyroDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTX_L_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_L_A",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedAccelX = accelSample.X;
            }

            return (byte) (accelSample.X & 0xFF);
          }
        );

      Register.OUTX_H_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTX_H_A",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedAccelX is not null
              ? (short) latchedAccelX
              : accelSample.X;

            latchedAccelX = null;
            accelDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTY_L_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_L_A",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedAccelY = accelSample.Y;
            }

            return (byte) (accelSample.Y & 0xFF);
          }
        );

      Register.OUTY_H_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTY_H_A",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedAccelY is not null
              ? (short) latchedAccelY
              : accelSample.Y;

            latchedAccelY = null;
            accelDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.OUTZ_L_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_L_A",
          valueProviderCallback: _ => {
            if (blockDataUpdate.Value)
            {
              latchedAccelZ = accelSample.Z;
            }

            return (byte) (accelSample.Z & 0xFF);
          }
        );

      Register.OUTZ_H_A.Define(this)
        .WithValueField(
          0, 8, R, name: "OUTZ_H_A",
          valueProviderCallback: _ => {
            short val = blockDataUpdate.Value && latchedAccelZ is not null
              ? (short) latchedAccelZ
              : accelSample.Z;

            latchedAccelZ = null;
            accelDataReady.Value = false;
            return (byte) (val >> 8);
          }
        );

      Register.EMB_FUNC_STATUS_MAINPAGE.Define(this)
        .WithFlag(7, R, name: "IS_FSM_LC")
        .WithReservedBits(0, 7);

      Register.FSM_STATUS_A_MAINPAGE.Define(this)
        .WithFlag(7, R, name: "IS_FSM8")
        .WithFlag(6, R, name: "IS_FSM7")
        .WithFlag(5, R, name: "IS_FSM6")
        .WithFlag(4, R, name: "IS_FSM5")
        .WithFlag(3, R, name: "IS_FSM4")
        .WithFlag(2, R, name: "IS_FSM3")
        .WithFlag(1, R, name: "IS_FSM2")
        .WithFlag(0, R, name: "IS_FSM1");

      Register.FSM_STATUS_B_MAINPAGE.Define(this)
        .WithFlag(7, R, name: "IS_FSM16")
        .WithFlag(6, R, name: "IS_FSM15")
        .WithFlag(5, R, name: "IS_FSM14")
        .WithFlag(4, R, name: "IS_FSM13")
        .WithFlag(3, R, name: "IS_FSM12")
        .WithFlag(2, R, name: "IS_FSM11")
        .WithFlag(1, R, name: "IS_FSM10")
        .WithFlag(0, R, name: "IS_FSM9");

      Register.MLC_STATUS_MAINPAGE.Define(this)
        .WithFlag(7, R, name: "IS_MLC8")
        .WithFlag(6, R, name: "IS_MLC7")
        .WithFlag(5, R, name: "IS_MLC6")
        .WithFlag(4, R, name: "IS_MLC5")
        .WithFlag(3, R, name: "IS_MLC4")
        .WithFlag(2, R, name: "IS_MLC3")
        .WithFlag(1, R, name: "IS_MLC2")
        .WithFlag(0, R, name: "IS_MLC1");

      Register.FIFO_STATUS1
        .Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "DIFF_FIFO[7:0]",
          valueProviderCallback: _ => (byte) (fifo.Count & 0xFF)
        );

      Register.FIFO_STATUS2.Define(this)
        .WithFlag(
          7, R, name: "FIFO_WTM_IA",
          valueProviderCallback: _ => fifo.Count >= (int) fifo.Watermark
        )
        .WithFlag(
          6, R, name: "FIFO_OVR_IA",
          valueProviderCallback: _ => fifo.Count >= fifo.Capacity
        )
        .WithFlag(
          5, R, name: "FIFO_FULL_IA",
          valueProviderCallback: _ => fifo.Count >= fifo.Capacity - 1
        )
        .WithFlag(4, FieldMode.ReadToClear, name: "COUNTER_BDR_IA")
        .WithFlag(3, FieldMode.ReadToClear, name: "FIFO_OVR_LATCHED")
        .WithReservedBits(2, 1)
        .WithValueField(
          0, 2,
          R,
          name: "DIFF_FIFO[9:8]",
          valueProviderCallback: _ => (byte) ((fifo.Count >> 8) & 0b11)
        );

      Register.TIMESTAMP0.Define(this)
        .WithValueField(0, 8, R, name: "D[31:24]");

      Register.TIMESTAMP1.Define(this)
        .WithValueField(0, 8, R, name: "D[23:16]");

      Register.TIMESTAMP2.Define(this)
        .WithValueField(0, 8, R, name: "D[15:8]");

      Register.TIMESTAMP3.Define(this)
        .WithValueField(0, 8, R, name: "D[7:0]");

      Register.INT_CFG0
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "INT_CLR_ON_READ")
        .WithFlag(5, RW, name: "SLEEP_STATUS_ON_INT")
        .WithFlag(4, RW, name: "SLOPE_FDS")
        .WithReservedBits(1, 3)
        .WithFlag(0, RW, name: "LIR");

      Register.INT_CFG1
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INTERRUPTS_ENABLE")
        .WithValueField(5, 2, RW, name: "INACT_EN")
        .WithReservedBits(0, 5);

      Register.THS_6D
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "D4D_EN")
        .WithValueField(5, 2, RW, name: "SIXD_THS")
        .WithReservedBits(0, 5);

      Register.WAKE_UP_THS
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithReservedBits(7, 1)
        .WithFlag(6, RW, name: "USR_OFF_ON_WU")
        .WithValueField(0, 6, RW, name: "WK_THS");

      Register.WAKE_UP_DUR.Define(this)
        .WithValueField(7, 1, RW, name: "FF_DUR5")
        .WithValueField(5, 2, RW, name: "WAKE_DUR")
        .WithFlag(4, RW, name: "WAKE_THS_W")
        .WithValueField(0, 4, RW, name: "SLEEP_DUR");

      Register.FREE_FALL.Define(this)
        .WithValueField(3, 5, RW, name: "FF_DUR[4:0]")
        .WithValueField(0, 3, RW, name: "FF_THS");

      Register.MD1_CFG.Define(this)
        .WithFlag(7, RW, name: "INT1_SLEEP_CHANGE")
        .WithReservedBits(6, 1)
        .WithFlag(5, RW, name: "INT1_WU")
        .WithFlag(4, RW, name: "INT1_FF")
        .WithReservedBits(3, 1)
        .WithFlag(2, RW, name: "INT1_6D")
        .WithFlag(1, RW, name: "INT1_EMB_FUNC")
        .WithReservedBits(0, 1);

      Register.MD2_CFG
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT2_SLEEP_CHANGE")
        .WithReservedBits(6, 1)
        .WithFlag(5, RW, name: "INT2_WU")
        .WithFlag(4, RW, name: "INT2_FF")
        .WithReservedBits(3, 1)
        .WithFlag(2, RW, name: "INT2_6D")
        .WithFlag(1, RW, name: "INT2_EMB_FUNC")
        .WithFlag(0, RW, name: "INT2_TIMESTAMP");

      Register.I3C_BUS_AVB.Define(this)
        .WithReservedBits(5, 3)
        .WithValueField(3, 2, RW, name: "I3C_Bus_Avb_Sel")
        .WithReservedBits(1, 2)
        .WithFlag(0, RW, name: "PD_DIS_INT1");

      Register.INTERNAL_FREQ_FINE.Define(this)
        .WithValueField(0, 8, R, name: "FREQ_FINE");

      Register.X_OFS_USR
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "X_OFS_USR");

      Register.Y_OFS_USR
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "Y_OFS_USR");

      Register.Z_OFS_USR
        .DefineConditional(this, () => !embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "Z_OFS_USR");

      Register.FIFO_DATA_OUT_TAG.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "TAG",
          valueProviderCallback: _ => fifo.ReadByte(0)
        );

      Register.FIFO_DATA_OUT_X_H.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[15:8]",
          valueProviderCallback: _ => fifo.ReadByte(1)
        );

      Register.FIFO_DATA_OUT_X_L.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[7:0]",
          valueProviderCallback: _ => fifo.ReadByte(2)
        );

      Register.FIFO_DATA_OUT_Y_H.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[15:8]",
          valueProviderCallback: _ => fifo.ReadByte(3)
        );

      Register.FIFO_DATA_OUT_Y_L.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[7:0]",
          valueProviderCallback: _ => fifo.ReadByte(4)
        );


      Register.FIFO_DATA_OUT_Z_H.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[15:8]",
          valueProviderCallback: _ => fifo.ReadByte(5)
        );

      Register.FIFO_DATA_OUT_Z_L.Define(this)
        .WithValueField(
          0, 8,
          R,
          name: "D[7:0]",
          valueProviderCallback: _ => fifo.ReadByte(6)
        );


      // Embedded registers //

      EmbeddedRegister.PAGE_SEL
        .DefineConditional(
          this,
          () => embeddedRegistersEnabled.Value,
          resetValue: 0x01
        )
        .WithValueField(4, 4, RW, name: "PAGE_SEL")
        .WithReservedBits(2, 2)
        .WithFlag(1, RW, name: "EMB_FUNC_CLK_DIS")
        .WithReservedBits(0, 1);

      EmbeddedRegister.EMB_FUNC_EN_B
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithReservedBits(5, 3)
        .WithFlag(4, RW, name: "MLC_EN")
        .WithReservedBits(1, 3)
        .WithFlag(0, RW, name: "FSM_EN");

      EmbeddedRegister.PAGE_ADDRESS
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "PAGE_ADDR");

      EmbeddedRegister.PAGE_VALUE
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "PAGE_VALUE");

      EmbeddedRegister.EMB_FUNC_INT1
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT1_FSM_LC")
        .WithReservedBits(0, 7);

      EmbeddedRegister.FSM_INT1_A
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT1_FSM8")
        .WithFlag(6, RW, name: "INT1_FSM7")
        .WithFlag(5, RW, name: "INT1_FSM6")
        .WithFlag(4, RW, name: "INT1_FSM5")
        .WithFlag(3, RW, name: "INT1_FSM4")
        .WithFlag(2, RW, name: "INT1_FSM3")
        .WithFlag(1, RW, name: "INT1_FSM2")
        .WithFlag(0, RW, name: "INT1_FSM1");

      EmbeddedRegister.FSM_INT1_B
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT1_FSM16")
        .WithFlag(6, RW, name: "INT1_FSM15")
        .WithFlag(5, RW, name: "INT1_FSM14")
        .WithFlag(4, RW, name: "INT1_FSM13")
        .WithFlag(3, RW, name: "INT1_FSM12")
        .WithFlag(2, RW, name: "INT1_FSM11")
        .WithFlag(1, RW, name: "INT1_FSM10")
        .WithFlag(0, RW, name: "INT1_FSM9");

      EmbeddedRegister.MLC_INT1
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT1_MLC8")
        .WithFlag(6, RW, name: "INT1_MLC7")
        .WithFlag(5, RW, name: "INT1_MLC6")
        .WithFlag(4, RW, name: "INT1_MLC5")
        .WithFlag(3, RW, name: "INT1_MLC4")
        .WithFlag(2, RW, name: "INT1_MLC3")
        .WithFlag(1, RW, name: "INT1_MLC2")
        .WithFlag(0, RW, name: "INT1_MLC1");

      EmbeddedRegister.EMB_FUNC_INT2
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT2_FSM_LC")
        .WithReservedBits(0, 7);

      EmbeddedRegister.FSM_INT2_A
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT2_FSM8")
        .WithFlag(6, RW, name: "INT2_FSM7")
        .WithFlag(5, RW, name: "INT2_FSM6")
        .WithFlag(4, RW, name: "INT2_FSM5")
        .WithFlag(3, RW, name: "INT2_FSM4")
        .WithFlag(2, RW, name: "INT2_FSM3")
        .WithFlag(1, RW, name: "INT2_FSM2")
        .WithFlag(0, RW, name: "INT2_FSM1");

      EmbeddedRegister.FSM_INT2_B
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT2_FSM16")
        .WithFlag(6, RW, name: "INT2_FSM15")
        .WithFlag(5, RW, name: "INT2_FSM14")
        .WithFlag(4, RW, name: "INT2_FSM13")
        .WithFlag(3, RW, name: "INT2_FSM12")
        .WithFlag(2, RW, name: "INT2_FSM11")
        .WithFlag(1, RW, name: "INT2_FSM10")
        .WithFlag(0, RW, name: "INT2_FSM9");

      EmbeddedRegister.MLC_INT2
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "INT2_MLC8")
        .WithFlag(6, RW, name: "INT2_MLC7")
        .WithFlag(5, RW, name: "INT2_MLC6")
        .WithFlag(4, RW, name: "INT2_MLC5")
        .WithFlag(3, RW, name: "INT2_MLC4")
        .WithFlag(2, RW, name: "INT2_MLC3")
        .WithFlag(1, RW, name: "INT2_MLC2")
        .WithFlag(0, RW, name: "INT2_MLC1");

      EmbeddedRegister.EMB_FUNC_STATUS
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, R, name: "IS_FSM_LC")
        .WithReservedBits(0, 7);

      EmbeddedRegister.FSM_STATUS_A
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, R, name: "IS_FSM8")
        .WithFlag(6, R, name: "IS_FSM7")
        .WithFlag(5, R, name: "IS_FSM6")
        .WithFlag(4, R, name: "IS_FSM5")
        .WithFlag(3, R, name: "IS_FSM4")
        .WithFlag(2, R, name: "IS_FSM3")
        .WithFlag(1, R, name: "IS_FSM2")
        .WithFlag(0, R, name: "IS_FSM1");

      EmbeddedRegister.FSM_STATUS_B
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, R, name: "IS_FSM16")
        .WithFlag(6, R, name: "IS_FSM15")
        .WithFlag(5, R, name: "IS_FSM14")
        .WithFlag(4, R, name: "IS_FSM13")
        .WithFlag(3, R, name: "IS_FSM12")
        .WithFlag(2, R, name: "IS_FSM11")
        .WithFlag(1, R, name: "IS_FSM10")
        .WithFlag(0, R, name: "IS_FSM9");

      EmbeddedRegister.MLC_STATUS
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, R, name: "IS_MLC8")
        .WithFlag(6, R, name: "IS_MLC7")
        .WithFlag(5, R, name: "IS_MLC6")
        .WithFlag(4, R, name: "IS_MLC5")
        .WithFlag(3, R, name: "IS_MLC4")
        .WithFlag(2, R, name: "IS_MLC3")
        .WithFlag(1, R, name: "IS_MLC2")
        .WithFlag(0, R, name: "IS_MLC1");

      EmbeddedRegister.PAGE_RW
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "EMB_FUNC_LIR")
        .WithFlag(6, RW, name: "PAGE_WRITE")
        .WithFlag(5, RW, name: "PAGE_READ")
        .WithReservedBits(0, 5);

      EmbeddedRegister.FSM_ENABLE_A
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "FSM8_EN")
        .WithFlag(6, RW, name: "FSM7_EN")
        .WithFlag(5, RW, name: "FSM6_EN")
        .WithFlag(4, RW, name: "FSM5_EN")
        .WithFlag(3, RW, name: "FSM4_EN")
        .WithFlag(2, RW, name: "FSM3_EN")
        .WithFlag(1, RW, name: "FSM2_EN")
        .WithFlag(0, RW, name: "FSM1_EN");

      EmbeddedRegister.FSM_ENABLE_B
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithFlag(7, RW, name: "FSM16_EN")
        .WithFlag(6, RW, name: "FSM15_EN")
        .WithFlag(5, RW, name: "FSM14_EN")
        .WithFlag(4, RW, name: "FSM13_EN")
        .WithFlag(3, RW, name: "FSM12_EN")
        .WithFlag(2, RW, name: "FSM11_EN")
        .WithFlag(1, RW, name: "FSM10_EN")
        .WithFlag(0, RW, name: "FSM9_EN");

      EmbeddedRegister.FSM_LONG_COUNTER_L
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "FSM_LC[7:0]");

      EmbeddedRegister.FSM_LONG_COUNTER_H
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithValueField(0, 8, RW, name: "FSM_LC[15:8]");

      EmbeddedRegister.FSM_LONG_COUNTER_CLEAR
        .DefineConditional(this, () => embeddedRegistersEnabled.Value)
        .WithReservedBits(2, 6)
        .WithFlag(1, R, name: "FSM_LC_CLEARED")
        .WithFlag(0, RW, name: "FSM_LC_CLEAR");

      EmbeddedRegister.FSM_OUTS1.DefineManyConditional(
        this,
        count: 16,
        condition: (_) => embeddedRegistersEnabled.Value,
        setup: (reg, _) => reg
          .WithFlag(7, R, name: "P_X")
          .WithFlag(6, R, name: "N_X")
          .WithFlag(5, R, name: "P_Y")
          .WithFlag(4, R, name: "N_Y")
          .WithFlag(3, R, name: "P_Z")
          .WithFlag(2, R, name: "N_Z")
          .WithFlag(1, R, name: "P_V")
          .WithFlag(0, R, name: "N_V")
      );

      EmbeddedRegister.EMB_FUNC_ODR_CFG_B
        .DefineConditional(
          this,
          condition: () => embeddedRegistersEnabled.Value,
          resetValue: 0x4B
        )
        .WithReservedBits(5, 3)
        .WithValueField(3, 2, RW, name: "FSM_ODR")
        .WithReservedBits(0, 3);

      EmbeddedRegister.EMB_FUNC_ODR_CFG_C
        .DefineConditional(
          this,
          condition: () => embeddedRegistersEnabled.Value,
          resetValue: 0x15
        )
        .WithReservedBits(6, 2)
        .WithValueField(4, 2, RW, name: "MLC_ODR")
        .WithReservedBits(0, 4);

      EmbeddedRegister.EMB_FUNC_INIT_B
        .DefineConditional(
          this,
          condition: () => embeddedRegistersEnabled.Value
        )
        .WithReservedBits(5, 3)
        .WithFlag(4, RW, name: "MLC_INIT")
        .WithReservedBits(1, 3)
        .WithFlag(0, RW, name: "FSM_INIT");

      EmbeddedRegister.MLC0_SRC.DefineManyConditional(
        this,
        count: 8,
        condition: (_) => embeddedRegistersEnabled.Value,
        setup: (reg, i) => reg
          .WithValueField(0, 8, R, name: $"MLC{i}_SRC")
      );
    }

    // Transmit
    private byte? address;
    private bool isWrite;

    private IFlagRegisterField accelDataReady = null!;
    private IFlagRegisterField gyroDataReady = null!;
    private IFlagRegisterField tempDataReady = null!;

    private IFlagRegisterField autoIncrement = null!;
    private IFlagRegisterField embeddedRegistersEnabled = null!;

    ///////////////////
    // Accelerometer //
    ///////////////////

    // The true acceleration, in milli-g, of the chip environment.
    public decimal AccelerationX { get; set; }
    public decimal AccelerationY { get; set; }
    public decimal AccelerationZ { get; set; }

    [OnRESDSample(SampleType.Acceleration)]
    private void HandleAccelerationSample(AccelerationSample sample, TimeInterval _)
    {
      // Convert acceleration samples from micro-g to milli-g.
      AccelerationX = (decimal) sample.AccelerationX / 1000m;
      AccelerationY = (decimal) sample.AccelerationY / 1000m;
      AccelerationZ = (decimal) sample.AccelerationZ / 1000m;
    }

    // The last collected acceleration sample.
    private DiscreteSample3D accelSample;
    private ulong accelSampleCount = 0;
    private ulong accelBatchPeriod = ulong.MaxValue;

    /// <summary>
    /// Quantizes an exact physical measurement into a 2-byte sample.
    /// </summary>
    private short QuantizeMeasurement(decimal exact, decimal sensitivity)
    {
      return (short) Math.Clamp(
        exact / sensitivity,
        short.MinValue,
        short.MaxValue
      );
    }

    private void OnMeasureAccelerometer()
    {

      decimal sensitivity = accelFullScale.Value switch
      {
        AccelFullScale.g2 => 0.061m,
        AccelFullScale.g4 => 0.122m,
        AccelFullScale.g8 => 0.244m,
        AccelFullScale.g16 => 0.488m,
        _ => throw new InvalidOperationException("Invalid AccelFullScale."),
      };

      short x = QuantizeMeasurement(AccelerationX, sensitivity);
      short y = QuantizeMeasurement(AccelerationY, sensitivity);
      short z = QuantizeMeasurement(AccelerationZ, sensitivity);

      accelSample = new DiscreteSample3D(x, y, z);
      accelDataReady.Value = true;
      this.Log(LogLevel.Debug, $"Measurement: acceleration [{AccelerationX}, {AccelerationY}, {AccelerationZ}] -> [{x}, {y}, {z}] @ sensitivity={sensitivity}");

      // Enqueue to the FIFO if this is a BDR sample.
      ulong bdrRatio = Math.Max(accelBatchPeriod / accelTimer.Limit, 1);
      if (++accelSampleCount % bdrRatio == 0)
      {
        fifo.Enqueue(Sensor.Accelerometer, x, y, z);
      }
    }

    private readonly struct DiscreteSample3D
    {
      public readonly short X, Y, Z;

      public DiscreteSample3D(short x, short y, short z)
      {
        X = x;
        Y = y;
        Z = z;
      }
    }

    private IEnumRegisterField<AccelFullScale> accelFullScale = null!;
    private IFlagRegisterField accelLpf2Enabled = null!;
    private readonly LimitTimer accelTimer;

    private enum AccelFullScale
    {
      g2 = 0,
      g16 = 1,
      g4 = 2,
      g8 = 3,
    }

    ///////////////
    // Gyroscope //
    ///////////////

    // Units are millidegrees per second (mdps).
    public decimal AngularVelocityX { get; set; }
    public decimal AngularVelocityY { get; set; }
    public decimal AngularVelocityZ { get; set; }

    [OnRESDSample(SampleType.AngularRate)]
    private void HandleAngularRateSample(
      AngularRateSample sample,
      TimeInterval time
    )
    {
      // Convert from rad/s * 10^5 to mdps.
      const decimal factor = 1000m * 180m / ((decimal) Double.Pi * 1e5m);
      AngularVelocityX = (decimal) sample.AngularRateX * factor;
      AngularVelocityY = (decimal) sample.AngularRateY * factor;
      AngularVelocityZ = (decimal) sample.AngularRateZ * factor;
    }

    // The latest gyroscope sample.
    private DiscreteSample3D gyroSample;
    private ulong gyroSampleCount = 0;
    private ulong gyroBatchPeriod = ulong.MaxValue;

    private void OnMeasureGyroscope()
    {
      decimal sensitivity;

      if (gyroFs4000.Value)
      {
        sensitivity = 140.00m;
      }
      else if (gyroFs125.Value)
      {
        sensitivity = 4.37m;
      }
      else
      {
        sensitivity = gyroFullScale.Value switch
        {
          GyroFullScale.dps250 => 8.75m,
          GyroFullScale.dps500 => 17.50m,
          GyroFullScale.dps1000 => 35.00m,
          GyroFullScale.dps2000 => 70.00m,
          _ => throw new InvalidOperationException("Invalid GyroFullScale"),
        };
      }

      short x = QuantizeMeasurement(AngularVelocityX, sensitivity);
      short y = QuantizeMeasurement(AngularVelocityY, sensitivity);
      short z = QuantizeMeasurement(AngularVelocityZ, sensitivity);

      gyroSample = new DiscreteSample3D(x, y, z);
      gyroDataReady.Value = true;
      this.Log(LogLevel.Debug, $"Measurement: angular velocity [{AngularVelocityX}, {AngularVelocityY}, {AngularVelocityZ}] -> [{x}, {y}, {z}] @ sensitivity={sensitivity}");

      // Enqueue to the FIFO if this is a BDR sample.
      ulong bdrRatio = Math.Max(gyroBatchPeriod / gyroTimer.Limit, 1);
      if (++gyroSampleCount % bdrRatio == 0)
      {
        fifo.Enqueue(Sensor.Gyroscope, x, y, z);
      }
    }

    private IEnumRegisterField<GyroFullScale> gyroFullScale = null!;
    private IFlagRegisterField gyroFs125 = null!;
    private IFlagRegisterField gyroFs4000 = null!;

    private readonly LimitTimer gyroTimer;

    private enum GyroFullScale
    {
      dps250 = 0,
      dps500 = 1,
      dps1000 = 2,
      dps2000 = 3,
    }

    ///////////////
    // Timestamp //
    ///////////////

    /////////////////
    // Temperature //
    /////////////////

    public decimal Temperature { get; set; }

    [OnRESDSample(SampleType.Temperature)]
    private void HandleTemperatureSample(TemperatureSample sample, TimeInterval _)
    {
      // Convert temperature samples from milli-C to C.
      Temperature = (decimal) sample.Temperature / 1000m;
    }
      
    private LimitTimer tempTimer;
    private short tempSample;

    private void OnMeasureTemperature()
    {
      // The temperature sensor does not collect measurements if the
      // accelerometer and gyroscope are both off.
      if (
        gyroTimer.Limit == ulong.MaxValue
        && accelTimer.Limit == ulong.MaxValue
      )
      {
        return;
      }

      const decimal sensitivity = 256; // 256 LSB per degree Celsius

      // Clamp to physical measurement constraints.
      decimal clamped = Math.Clamp(Temperature, -40m, 125m);
      tempSample = QuantizeMeasurement(clamped - 25, sensitivity);
      tempDataReady.Value = true;
      this.Log(LogLevel.Debug, $"Measurement: temperature {Temperature} -> {tempSample} @ sensitivity={sensitivity}");
    }

    //////////
    // FIFO //
    //////////

    private LimitTimer batchTimer;
    private uint batchCounter = 0;

    private void UpdateBatchTimer()
    {
      batchTimer.Limit = Math.Min(gyroBatchPeriod, accelBatchPeriod); // TODO: add temperature?
    }

    private void OnCommitBatch()
    {
      batchCounter = (batchCounter + 1) & 0b11;
    }

    private Fifo fifo;
    private IEnumRegisterField<FifoModeSetting> fifoModeSetting = null!;
    private IFlagRegisterField fifoOdrChangeEnabled = null!;

    // TODO: relocate and reform
    private bool fifoModeTrigger = false;

    /// <summary>
    /// TODO: fix doc comment
    ///
    /// Different from Fifo.Mode in that Fifo.Mode reflects the currently
    /// active mode, while FifoModeSetting is an expanded definition encompassing
    /// all of the possible _settings_, which may also specify transitions.
    /// </summary>
    private enum FifoModeSetting
    {
      // FIFO is not operational and remains empty.
      Bypass = 0,

      // Data from outputs channels is stored in the FIFO until full.
      Fifo = 1,

      // As new data arrives, older data is discarded.
      Continuous = 6,

      // FIFO behavior changes according to the trigger event detected in the
      // case of wake-up, free-fall, and D6D interrupt events.
      //
      // Trigger bit = 1 -> FIFO mode
      // Trigger bit = 0 -> continuous mode
      ContinuousToFifo = 5,

      // Operates in continuous mode when selected triggers are 1, otherwise
      // FIFO content is reset.
      BypassToContinuous = 4,

      // Operates in FIFO mode when selected triggers are 1, otherwise FIFO
      // content is reset.
      BypassToFifo = 7,
    }

    private class Fifo
    {
      // The datasheet specifies that the FIFO queue is 3KB long.
      // This is not evenly divided by the entry length, which is 7 bytes, so
      // the result has been rounded up.
      public const int MaxCapacity = 439;

      private readonly ASM330LHBG1 peripheral;
      private readonly Queue<Sample> queue;
      private uint fieldsRead = 0;

      public Fifo(ASM330LHBG1 peripheral)
      {
        this.peripheral = peripheral;
        queue = new Queue<Sample>();
        Capacity = MaxCapacity;
      }

      // Calculate active mode based on the mode setting plus triggers.
      public Mode ActiveMode => peripheral.fifoModeSetting.Value switch {
        FifoModeSetting.Bypass => Mode.Bypass,
        FifoModeSetting.Fifo => Mode.Fifo,
        FifoModeSetting.Continuous => Mode.Continuous,
        FifoModeSetting.ContinuousToFifo => peripheral.fifoModeTrigger
          ? Mode.Continuous
          : Mode.Fifo,
        FifoModeSetting.BypassToContinuous => peripheral.fifoModeTrigger
          ? Mode.Bypass
          : Mode.Continuous,
        FifoModeSetting.BypassToFifo => peripheral.fifoModeTrigger
          ? Mode.Bypass
          : Mode.Fifo,
        _ => throw new InvalidOperationException("Invalid FifoModeSetting."),
      };

      public int Capacity { get; set; }
      public int Count => queue.Count;
      public int Watermark { get; set; }

      /// <summary>
      /// Enqueues a sample onto the FIFO queue, if there is room for it.
      /// </summary>
      public void Enqueue(Sensor sensor, short x, short y, short z)
      {
        // Ignore all enqueued samples in bypass mode (FIFO inactive).
        if (ActiveMode == Mode.Bypass)
        {
          peripheral.Log(LogLevel.Debug, "Enqueue on bypassed FIFO. Discarding.");
          return;
        }

        // Handle full FIFO condition based on active mode.
        if (Count >= Capacity)
        {
          if (ActiveMode == Mode.Fifo)
          {
            peripheral.Log(LogLevel.Debug, "Enqueue on full FIFO. Discarding.");
            return;
          }
          else
          {
            peripheral.Log(LogLevel.Debug, "Enqueue on full FIFO. Bumping.");
            queue.Dequeue();
          }
        }

        SampleTag tag = new SampleTag(sensor, peripheral.batchCounter);
        Sample sample = new Sample(tag, x, y, z);
        queue.Enqueue(sample);

        peripheral.Log(LogLevel.Debug, $"Enqueued {sample.Tag.Sensor} sample in batch {sample.Tag.Count}");

        // Check that the observed count is not above the hardware maximum.
        // This should never happen, which is why it's an error.
        if (Count > MaxCapacity)
        {
          peripheral.Log(LogLevel.Error, "FIFO exceeds maximum capacity.");
        }
      }

      /// <summary>
      /// Clears the FIFO queue of all entries.
      /// </summary>
      public void Clear()
      {
        peripheral.Log(LogLevel.Debug, $"Clearing FIFO of {Count} entries.");
        queue.Clear();
        fieldsRead = 0;
      }

      /// <summary>
      /// Reads a single byte from the head sample, automatically dequeuing the
      /// sample once all of its bytes have been read.
      /// </summary>
      public byte ReadByte(int i)
      {
        Sample entry;
        if (!queue.TryPeek(out entry))
        {
          return 0x00;
        }

        if ((fieldsRead |= (uint) (1 << i)) == 0x7F)
        {
          queue.Dequeue();
          fieldsRead = 0;
        }

        return entry.GetByte(i);
      }

      public enum Mode
      {
        Bypass,
        Fifo,
        Continuous,
      }

      public readonly struct Sample
      {
        public readonly SampleTag Tag;
        public readonly short X, Y, Z;

        public Sample(SampleTag tag, short x, short y, short z)
        {
          Tag = tag;
          X = x;
          Y = y;
          Z = z;
        }

        public byte GetByte(int i)
        {
          return (byte) (
            i switch
            {
              0 => Tag.ToByte(),
              1 => X >> 8,
              2 => X & 0xFF,
              3 => Y >> 8,
              4 => Y & 0xFF,
              5 => Z >> 8,
              6 => Z & 0xFF,
              _ => throw new ArgumentOutOfRangeException("Byte index out of bounds."),
            }
          );
        }
      }

      public readonly struct SampleTag
      {
        public readonly Sensor Sensor;
        public readonly uint Count;
        public readonly bool Parity;

        public SampleTag(Sensor sensor, uint count)
        {
          if (count > 3)
          {
            throw new ArgumentOutOfRangeException("Invalid tag count.");
          }

          Sensor = sensor;
          Count = count;
          Parity = BitOperations.PopCount((uint) sensor << 3 | count) % 2 == 1;
        }

        public byte ToByte()
        {
          return (byte) ((byte) Sensor << 3 | (byte) Count << 1 | (Parity ? 1 : 0));
        }
      }
    }

    private enum Sensor
    {
      Gyroscope = 1,
      Accelerometer = 2,
      Temperature = 3,
      Timestamp = 4,
      ConfigurationChange = 5,
    }

    // Latched data values for when block data update (BDU) is enabled.
    private short? latchedTemp;
    private short? latchedAccelX;
    private short? latchedAccelY;
    private short? latchedAccelZ;
    private short? latchedGyroX;
    private short? latchedGyroY;
    private short? latchedGyroZ;

    private IFlagRegisterField blockDataUpdate = null!;
    private IFlagRegisterField i2cDisabled = null!;

    private enum DataRate
    {
      PowerDown = 0,
      Hz12_5 = 1,
      Hz26 = 2,
      Hz52 = 3,
      Hz104 = 4,
      Hz208 = 5,
      Hz416 = 6,
      Hz833 = 7,
      Hz1677 = 8,
      Hz6_5 = 11,
    }

    private static DataRate DataRateFromPeriod(ulong limit)
    {
      return limit switch
      {
        ulong.MaxValue => DataRate.PowerDown,
        1024 => DataRate.Hz6_5,
        512 => DataRate.Hz12_5,
        256 => DataRate.Hz26,
        128 => DataRate.Hz52,
        64 => DataRate.Hz104,
        32 => DataRate.Hz208,
        16 => DataRate.Hz416,
        8 => DataRate.Hz833,
        4 => DataRate.Hz1677,
        _ => throw new ArgumentException("Invalid data rate limit value."),
      };
    }

    private static ulong DataRateToPeriod(DataRate rate)
    {
      return rate switch
      {
        DataRate.PowerDown => ulong.MaxValue,
        DataRate.Hz6_5 => 1024,
        DataRate.Hz12_5 => 512,
        DataRate.Hz26 => 256,
        DataRate.Hz52 => 128,
        DataRate.Hz104 => 64,
        DataRate.Hz208 => 32,
        DataRate.Hz416 => 16,
        DataRate.Hz833 => 8,
        DataRate.Hz1677 => 4,
        _ => throw new ArgumentException("Invalid representation of DataRate."),
      };
    }

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

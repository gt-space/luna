using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities.RESD;

namespace Antmicro.Renode.Peripherals.Sensors
{
  /// <summary>
  ///  MS5611 barometric pressure sensor Renode peripheral.
  /// </summary>
  public class MS5611 :
    ISPIPeripheral,
    IProvidesRegisterCollection<WordRegisterCollection>,
    IUnderstandRESD
  {
    public byte? command;
    public int byteIndex;
    public uint digitalValue;
    public bool conversionDone;
    private IMachine machine;

    public MS5611(IMachine machine)
    {
      this.machine = machine;
      RegistersCollection = new WordRegisterCollection(this);
      DefineRegisters();
      Reset();
    }

    /// <summary>
    /// Called when the sensor is reset, such as a RESET pin being pulled
    /// active.
    /// </summary>
    public void Reset()
    {
      RegistersCollection.Reset();
      command = null;
      byteIndex = 0;
      digitalValue = 0;
      conversionDone = false;
    }

    /// <summary>
    /// Defines registers and their fields according to the datasheet.
    /// </summary>
    private void DefineRegisters()
    {
      const FieldMode R = FieldMode.Read;

      // Use the example / typical values found in the datasheet for resets.

      Register.C1.Define(this, resetValue: 40127)
        .WithValueField(0, 16, R, name: "C1");

      Register.C2.Define(this, resetValue: 36924)
        .WithValueField(0, 16, R, name: "C2");

      Register.C3.Define(this, resetValue: 23317)
        .WithValueField(0, 16, R, name: "C3");

      Register.C4.Define(this, resetValue: 23282)
        .WithValueField(0, 16, R, name: "C4");

      Register.C5.Define(this, resetValue: 33464)
        .WithValueField(0, 16, R, name: "C5");

      Register.C6.Define(this, resetValue: 28312)
        .WithValueField(0, 16, R, name: "C6");
    }

    /// <summary>
    /// Called in sequence for every byte of a SPI transfer.
    /// </summary>
    public byte Transmit(byte data)
    {
      if (command == null)
      {
        command = data;
        this.Log(LogLevel.Debug, $"Received command: 0x{data:X2}");
        byteIndex = 0;

        // Handle one-byte commands.

        if (command == 0x1E) // Reset
        {
          Reset();
        }
        else if ((command & 0xF0) == 0x40) // D1 conversion (read pressure)
        {
          ConvertPressure();
        }
        else if ((command & 0xF0) == 0x50) // D2 conversion (read temperature)
        {
          ConvertTemperature();
        }

        return 0xFF;
      }

      if (command == 0x00) // Read ADC
      {
        if (!conversionDone)
        {
          return 0x00;
        }

        byte adcByte = (byte) (digitalValue >> ((2 - byteIndex) * 8) & 0xFF);

        if (++byteIndex >= 3)
        {
          conversionDone = false;
        }

        return adcByte;
      }
      else if ((command & 0xF0) == 0xA0) // Read PROM
      {
        if (byteIndex > 1)
        {
          return 0x00;
        }

        int cnum = ((int) command >> 1) & 0b111; // 0xA2 -> cnum = 1

        if (cnum == 0)
        {
          return 0x00;
        }
        else if (cnum == 7)
        {
          ushort[] proms = new ushort[] {
            RegistersCollection.Read(0x02),
            RegistersCollection.Read(0x04),
            RegistersCollection.Read(0x06),
            RegistersCollection.Read(0x08),
            RegistersCollection.Read(0x0A),
            RegistersCollection.Read(0x0C),
          };

          return crc4(proms);
        }

        ushort promWord = RegistersCollection.Read(cnum * 2);
        return (byte) (promWord >> ((1 - byteIndex++) * 8) & 0xFF);
      }

      this.Log(
        LogLevel.Warning,
        $"Unhandled Transmit for command: 0x{command!:X2}"
      );
      return 0x00;
    }

    /// <summary>
    /// Calculates CRC. Ported directly from the C code provided by AN520.
    /// </summary>
    /// <remark>
    /// The code quality on this CRC function is awful, but it was ported
    /// one-for-one from TE Connectivity's C implementation. We should consider
    /// refactoring this in the future, but it works for now.
    /// </remark>
    private byte crc4(ushort[] n_prom)
    {
      int cnt; // simple counter
      uint n_rem; // crc reminder
      uint crc_read; // original value of the crc
      byte n_bit;
      n_rem = 0x00;
      crc_read = n_prom[7]; //save read CRC
      n_prom[7] = (ushort) (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
      for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
      { // choose LSB or MSB
        if (cnt % 2 == 1) n_rem ^= (ushort)((n_prom[cnt >> 1]) & 0x00FF);
        else n_rem ^= (ushort) (n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
          if ((n_rem & (0x8000)) != 0)
          {
            n_rem = (n_rem << 1) ^ 0x3000;
          }
          else
          {
            n_rem = (n_rem << 1);
          }
        }
      }
      n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
      n_prom[7] = (ushort) crc_read; // restore the crc_read to its original place
      return (byte) (n_rem ^ 0x00);
    }

    /// <summary>
    /// Called when a SPI transfer finishes (chip select pulled high).
    /// </summary>
    public void FinishTransmission()
    {
      byteIndex = 0;
      command = null;
    }

    //////////////
    // Pressure //
    //////////////

    /// <summary>
    /// The current environmental pressure, in millibar.
    /// </summary>
    public decimal Pressure { get; set; }

    [OnRESDSample(SampleType.Pressure)]
    private void HandlePressureSample(PressureSample sample, TimeInterval _)
    {
      // Convert pressure from milliPascals to millibar.
      Pressure = (decimal) sample.Pressure * 1e-5m;
    }

    /// <summary>
    /// Converts the environmental pressure measurement into the digital value
    /// read from the virtual D1 register.
    /// </summary>
    /// <remark>
    /// These calculations are derived by inversing the datasheet calculations
    /// for pressure intended to be performed by the driver.
    ///
    /// Original:
    /// OFF = C2 * 2^16 + (C4 * dT) / 2^7
    /// SENS = C1 * 2^15 + (C3 * dT) / 2^8
    /// P = (D1 * SENS / 2^21 - OFF) / 2^15
    ///
    /// Inversed:
    /// D1 = (P * 2^15 + OFF) * 2^21 / SENS
    ///
    /// The original definitions of OFF and SENS do not need to be rearranged.
    /// Note that dT must be calculated as in ConvertTemperature.
    /// </remark>
    private void ConvertPressure()
    {
      this.Log(LogLevel.Debug, $"Converting Pressure: {Pressure} mb");

      ushort c1 = RegistersCollection.Read((long) Register.C1);
      ushort c2 = RegistersCollection.Read((long) Register.C2);
      ushort c3 = RegistersCollection.Read((long) Register.C3);
      ushort c4 = RegistersCollection.Read((long) Register.C4);

      // Clamp pressure to simulate hardware constraints.
      decimal clampedPressure = Math.Clamp(Pressure, 10m, 1200m);
      long quantizedPressure = (long) (clampedPressure * 100m);

      long dT = (long) TemperatureDifferential();
      long offset = ((long) c2 << 16) + (((long) c4 * dT) >> 7);
      long sensitivity = ((long) c1 << 15) + (((long) c3 * dT) >> 8);

      // The temperature used below must be the temperature that would be read
      // _before_ low-temperature compensation. As such, the T2 offset must be
      // re-added to TEMP.
      decimal clampedTemp = Math.Clamp(Temperature, -40m, 85m);
      long uncompensatedTemp = (long) (clampedTemp * 100m) + ((dT * dT) >> 31);

      // Low-temperature compensation for pressure.
      if (uncompensatedTemp < 2000)
      {
        long normalizedTemp = uncompensatedTemp - 2000;
        long intermediate = 5 * normalizedTemp * normalizedTemp;

        offset -= intermediate >> 1;
        sensitivity -= intermediate >> 2;

        if (uncompensatedTemp < -1500)
        {
          normalizedTemp = uncompensatedTemp + 1500;
          intermediate = normalizedTemp * normalizedTemp;

          offset -= 7 * intermediate;
          sensitivity -= (11 * intermediate) >> 1;
        }
      }

      long offsetPressure = (quantizedPressure << 15) + offset;
      digitalValue = (uint) ((offsetPressure << 21) / sensitivity);
      conversionDone = true;
    }

    /////////////////
    // Temperature //
    /////////////////

    /// <summary>
    /// The current environmental temperature, in °C.
    /// </summary>
    public decimal Temperature { get; set; }

    [OnRESDSample(SampleType.Temperature)]
    private void HandleTemperatureSample(
      TemperatureSample sample,
      TimeInterval _
    )
    {
      // Convert temperature from milli-°C to °C.
      Temperature = (decimal) sample.Temperature / 1000m;
    }

    /// <summary>
    /// Converts the environmental temperature measurement into the digital
    /// value read from the virtual D2 register.
    /// </summary>
    /// <remark>
    /// These calculations are derived by inversing the datasheet calculations
    /// for temperature intended to be performed by the driver. Note that the
    /// calculation for dT is done in <c>TemperatureDifferential</c>.
    ///
    /// Original:
    /// dT = D2 - C5 * 2^8
    ///
    /// Inversed:
    /// D2 = dT + C5 * 256
    ///
    /// </remark>
    private void ConvertTemperature()
    {
      this.Log(LogLevel.Debug, $"Converting temperature: {Temperature}° C");

      ushort c5 = RegistersCollection.Read((long) Register.C5);
      int dT = TemperatureDifferential();
      uint d2 = (uint) (dT + ((int) c5 << 8));

      digitalValue = d2;
      conversionDone = true;
    }

    /// <summary>
    /// Determines the temperature differential, dT, using the environmental
    /// temperature and calibrated coefficients.
    ///
    /// Used for both pressure and temperature calculations due to temperature
    /// compensation in pressure calculations.
    /// </summary>
    /// <remark>
    /// There are two equations used for calculating dT, depending on whether
    /// low-temperature compensation is being used.
    ///
    /// Normal temperature:
    /// TEMP = 2000 + dT * C6 / 2^23
    /// dT = (TEMP - 2000) * 2^23 / C6
    ///
    /// Low temperature: (less than 20°C)
    /// TEMP = 2000 + dT * C6 / 2^23 - dT^2 / 2^31
    /// dT = 2^7 * C6 - sqrt(2^14 * C6^2 + 2^31 * (2000 - TEMP))
    /// </remark>
    private int TemperatureDifferential()
    {
      // Clamp the environment temperature to replicate hardware constraints.
      decimal clamped = Math.Clamp(Temperature, -40m, 85m);

      // Quantize with 0.01°C resolution.
      int quantized = (int) (clamped * 100m);
      ushort c6 = RegistersCollection.Read((long) Register.C6);

      if (Temperature >= 20m)
      {
        return (int) (((long) (quantized - 2000) << 23) / c6);
      }
      else
      {
        long c = (long) c6;
        long normalized = (long) (2000 - quantized);
        double discriminant = ((c * c) << 14) + (normalized << 31);

        // Perform low-temperature compensation.
        return ((int) c6 << 7) - (int) Math.Sqrt(discriminant);
      }
    }

    // Maps the register map of the chip onto a virtual register collection.
    public WordRegisterCollection RegistersCollection { get; private set; }

    /// <summary>
    /// Register addresses retrieved from the datasheet's register map.
    /// </summary>
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

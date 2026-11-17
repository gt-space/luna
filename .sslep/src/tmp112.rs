//put all the temp data in a digital register that will be read over i2c
//temp is in Celsius
use embedded_hal::i2c::I2c;

// pre configured TMP112 register addresses for each sensor
//stores measured temp
const TEMP_REGISTER: u8 = 0x00;
//stores sensor's operating settings
const CONFIG_REGISTER: u8 = 0x01;
//stores low temp alert
const T_LOW_REGISTER: u8 = 0x02;
//stores high temp alert
const T_HIGH_REGISTER: u8 = 0x03;

// Configuration register bit masks
const SHUTDOWN_BIT: u16 = 0x0100;
const EXTENDED_MODE_BIT: u16 = 0x0010;

// TMP112 temperature resolution
const TEMP_RESOLUTION: f32 = 0.0625;

// Represents one TMP112 sensor
pub struct Tmp112 {
    address: u8,
}

// Error when constructing a sensor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidAddress(pub u8);

impl Tmp112 {
    // TMP112 uses I2C to send ADC converted digital data from TEMP_REGISTER to pi
    // Create a TMP112 using its 7-bit I2C address
    // ADD0 pin determines i2c address for each temp sensor
    // dtsheet specifies which I2C address corresponds to each ADD0 connnection (GND, V, SDA, SCL)
    // dependent on physical ADD0 connection
    pub fn new(address: u8) -> Result<Self, InvalidAddress> {
        if !(0x48..=0x4B).contains(&address) {
            return Err(InvalidAddress(address));
        }

        Ok(Self { address })
    }

    // Return this sensor's I2C address
    pub fn address(&self) -> u8 {
        self.address
    }

    // Read any 16-bit TMP112 register
    fn read_register<I: I2c>(
        &self,
        i2c: &mut I,
        register: u8,
    ) -> Result<u16, I::Error> {
        let mut buffer = [0u8; 2];

        //not continuously requesting data from sensor, need to request first
        i2c.write_read(
            self.address,
            &[register],
            &mut buffer,
        )?;

        Ok(u16::from_be_bytes(buffer))
    }

    // Write a 16-bit register
    fn write_register<I: I2c>(
        &self,
        i2c: &mut I,
        register: u8,
        value: u16,
    ) -> Result<(), I::Error> {
        let [msb, lsb] = value.to_be_bytes();

        i2c.write(
            self.address,
            &[register, msb, lsb],
        )
    }

    // Read the configuration register
    pub fn read_config<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<u16, I::Error> {
        self.read_register(i2c, CONFIG_REGISTER)
    }

    // Write the configuration register
    pub fn write_config<I: I2c>(
        &self,
        i2c: &mut I,
        config: u16,
    ) -> Result<(), I::Error> {
        self.write_register(i2c, CONFIG_REGISTER, config)
    }

    // Decode a raw temperature register value
    fn decode_temperature(raw: u16, extended: bool) -> f32 {
        // Shift the signed two's-complement value
        let counts = if extended {
            (raw as i16) >> 3
        } else {
            (raw as i16) >> 4
        };

        counts as f32 * TEMP_RESOLUTION
    }

    // Read current temperature in Celsius
    pub fn read_temperature_celsius<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<f32, I::Error> {
        let config = self.read_config(i2c)?;

        let extended = (config & EXTENDED_MODE_BIT) != 0;

        let raw = self.read_register(i2c, TEMP_REGISTER)?;

        Ok(Self::decode_temperature(raw, extended))
    }

    // Read current temperature in Fahrenheit
    pub fn read_temperature_fahrenheit<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<f32, I::Error> {
        let celsius = self.read_temperature_celsius(i2c)?;

        Ok(celsius * 9.0 / 5.0 + 32.0)
    }

    // Read the low temperature threshold
    pub fn read_low_threshold<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<f32, I::Error> {
        let config = self.read_config(i2c)?;
        let extended = (config & EXTENDED_MODE_BIT) != 0;

        let raw = self.read_register(i2c, T_LOW_REGISTER)?;

        Ok(Self::decode_temperature(raw, extended))
    }

    // Read the high temperature threshold
    pub fn read_high_threshold<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<f32, I::Error> {
        let config = self.read_config(i2c)?;
        let extended = (config & EXTENDED_MODE_BIT) != 0;

        let raw = self.read_register(i2c, T_HIGH_REGISTER)?;

        Ok(Self::decode_temperature(raw, extended))
    }

    // Enable or disable shutdown mode.
    // enabled = true means stop continuous conversions.
    pub fn set_shutdown<I: I2c>(
        &self,
        i2c: &mut I,
        enabled: bool,
    ) -> Result<(), I::Error> {
        let mut config = self.read_config(i2c)?;

        if enabled {
            config |= SHUTDOWN_BIT;
        } else {
            config &= !SHUTDOWN_BIT;
        }

        self.write_config(i2c, config)
    }

    // Check whether the sensor is in shutdown mode
    pub fn is_shutdown<I: I2c>(
        &self,
        i2c: &mut I,
    ) -> Result<bool, I::Error> {
        let config = self.read_config(i2c)?;

        Ok((config & SHUTDOWN_BIT) != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn temperature_conversion() {
        // 25 degrees Celsius
        assert_eq!(
            Tmp112::decode_temperature(0x1900, false),
            25.0
        );

        // -10 degrees Celsius
        assert_eq!(
            Tmp112::decode_temperature(0xF600, false),
            -10.0
        );
    }

    #[test]
    fn address_validation() {
        assert!(Tmp112::new(0x48).is_ok());
        assert!(Tmp112::new(0x4B).is_ok());
        assert!(Tmp112::new(0x50).is_err());
    }
}

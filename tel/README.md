# Telemetry

# Pin Mappings

## Development Kits

There is a special `devkit` deployment target that is meant for development
testing with the SX1280DVK1ZHP development kit, the one originally meant for
interfacing with the STM Nucleo L073RZ board.

Following the pinout diagrams below, you can connect the development kit board
to a Raspberry Pi 4B, flash the `devkit` image onto the Pi 4B, and run tests
between two boards.

Note that before compiling one of the `devkit` images, you will need to change
the IP address of the `radio0` interface so that they can communicate with each
other on the same subnet instead of effectively becoming loopback interfaces.

### Raspberry Pi 4B

Connect wires to the following positions on the Raspberry Pi 4B. In this pinout
diagram, the Pi is oriented with the SoC upward, with the GPIO pinout in the
upper-right corner.

```
   3V3 - ▪▪ - 5V
         ▪▪
         ▪▪
         ▪▪
         ▪▪
NRESET - ▪▪
  DIOx - ▪▪
  BUSY - ▪▪ - ANT_SW
         ▪▪
  MOSI - ▪▪
  MISO - ▪▪
   SCK - ▪▪ - NSS
   GND - ▪▪
         ▪▪
         ▪▪ - GND
         ▪▪
         ▪▪ - GND
         ▪▪
         ▪▪
         ▪▪
```

### SX1280DVK1ZHP

Connect the other ends of the wires you connected to the Pi 4B above to the
development kit board in the following configuration. In this pinout diagram,
the board is "facing down", with a large Semtech logo and recycle logo pointed
upwards, and the Semtech logo is positioned so that it can be read directly.

```
▪                      ▪
▪                      ▪
▪ - ANT_SW             ▪
▪               BUSY - ▪
▪                      ▪
▪ - NRESET      DIOx - ▪
                       ▪
                 NSS - ▪
▪
▪ - GND                ▪
▪ - GND                ▪
▪ - 5V                 ▪
▪ - 3V3         MOSI - ▪
▪               MISO - ▪
▪                SCK - ▪
▪                GND - ▪
                       ▪
                       ▪
                       ▪
```

## Flight TEL

The following tables represent the hard-wired pin connections of the Raspberry
Pi CM4 to the various chips on the flight and ground telemetry boards, through
the BeagleBone breakout board.

| SX1280        | BeagleBone           | CM4                  |
| ------------- | -------------------- | -------------------- |
| BUSY          | GPIO_81              | GPIO_27              |
| CS            | SPI1_CS1             | GPIO_17 (SPI1_CE1_N) |
| DIO1          | GPIO_73              | GPIO_0               |
| DIO2          | GPIO_75              | GPIO_5               |
| DIO3          | GPIO_77              | GPIO_22              |
| MISO          | GPIO_111 (SPI1_D0)   | GPIO_19 (SPI1_MISO)  |
| MOSI          | GPIO_112 (SPI1_D1)   | GPIO_20 (SPI1_MOSI)  |
| RESET         | GPIO_71              | GPIO_6               |
| SCLK          | GPIO_110 (SPI1_SCLK) | GPIO_21 (SPI1_SCLK)  |

| ADC 0         | BeagleBone           | CM4                  |
| ------------- | -------------------- | -------------------- |
| CS            | GPIO_51              | GPIO_24              |
| DRDY          | GPIO_48              | GPIO_23              |

| ADC 1         | BeagleBone           | CM4                  |
| ------------- | -------------------- | -------------------- |
| CS            | GPIO_50              | GPIO_14              |
| DRDY          | GPIO_31              | GPIO_15              |

| SX1262        | BeagleBone           | CM4                  |
| ------------- | -------------------- | -------------------- |
| BUSY          | GPIO_113             | GPIO_25              |
| CS            | SPI1_CS0             | GPIO_18 (SPI1_CE0_N) |
| DIO1          | GPIO_117             | GPIO_16              |
| DIO2          | GPIO_14              | GPIO_12              |
| DIO3          | GPIO_115             | GPIO_13              |
| MISO          | GPIO_111 (SPI1_D0)   | GPIO_19 (SPI1_MISO)  |
| MOSI          | GPIO_112 (SPI1_D1)   | GPIO_20 (SPI1_MOSI)  |
| RESET         | GPIO_20              | GPIO_26              |
| SCLK          | GPIO_110 (SPI1_SCLK) | GPIO_21 (SPI1_SCLK)  |

| GPS           | BeagleBone           | CM4                  |
| ------------- | -------------------- | -------------------- |
| CS            | GPIO_5 (SPI0_CS0)    | GPIO_8 (SPI0_CE0_N)  |
| MISO          | GPIO_3 (SPI0_D0)     | GPIO_9 (SPI0_MISO)   |
| MOSI          | GPIO_4 (SPI0_D1)     | GPIO_10 (SPI0_MOSI)  |
| RESET         | GPIO_89              | GPIO_4               |
| SCLK          | GPIO_2 (SPI0_SCLK)   | GPIO_11 (SPI0_SCLK)  |

| Switches      | BeagleBone           | CM4                  |
| ------------  | -------------------- | -------------------- |
| RF-2400-SW-V1 | GPIO_61              | GPIO_3               |
| RF-2400-SW-V2 | GPIO_65              | GPIO_2               |
| RF-915-SW-V1  | GPIO_15              | GPIO_7               |
| RF-915-SW-V2  | GPIO_49              | GPIO_1               |

## Ground TEL

Ground TEL has the same pin mappings as Flight TEL, except it is missing a few
pins that are not necessary, since it can't transmit:

- `GPS-CS`
- `GPS-MISO`
- `GPS-MOSI`
- `GPS-RESET`
- `GPS-SCLK`
- `RF-2400-SW-V1`
- `RF-2400-SW-V2`
- `RF-915-SW-V1`
- `RF-915-SW-V2`

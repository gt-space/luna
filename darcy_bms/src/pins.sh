#!/bin/bash

# P9 GPIO (use pin number from Altium schematic)
config-pin p9.16 gpio
config-pin p9.23 gpio
config-pin p9.24 gpio
config-pin p9.26 gpio

# P8 GPIO (subtract 46 from the pin number on Altium schematic)
config-pin p8.7 gpio
config-pin p8.18 gpio
config-pin p8.8 gpio
config-pin p8.10 gpio
config-pin p8.16 gpio
config-pin p8.26 gpio

# SPI 0
config-pin p9_18 spi
config-pin p9_21 spi
config-pin p9_22 spi_sclk
#!/bin/bash

# P9 GPIO
config-pin p9.11 gpio
config-pin p9.12 gpio
config-pin p9.13 gpio
config-pin p9.14 gpio
config-pin p9.15 gpio
config-pin p9.16 gpio
config-pin p9.23 gpio
config-pin p9.24 gpio
config-pin p9.26 gpio

# P8 GPIO (subtract 46 from the pin number on Altium schematic)
config-pin p8.7 gpio
config-pin p8.8 gpio
config-pin p8.9 gpio
config-pin p8.10 gpio
config-pin p8.11 gpio
config-pin p8.12 gpio
config-pin p8.13 gpio
config-pin p8.14 gpio
config-pin p8.18 gpio
config-pin p8.19 gpio
config-pin p8.21 gpio
config-pin p8.23 gpio
config-pin p8.30 gpio

# SPI 0
config-pin p9_18 spi
config-pin p9_21 spi
config-pin p9_22 spi_sclk
#!/bin/bash

# AHRS pin configuration

# P8 GPIO
config-pin p8.34 gpio # IMU-NRESET
config-pin p8.30 gpio # IMU-DR
config-pin p8.46 gpio # CAM-EN

# SPI1
config-pin p9.31 spi_sclk
config-pin p9.29 spi
config-pin p9.30 spi
config-pin p9.19 spi_cs # MAG-CS
config-pin p9.20 spi_cs # BAR-CS

# SPI0
config-pin p9.22 spi_sclk
config-pin p9.21 spi
config-pin p9.18 spi
config-pin p9.17 spi_cs # IMU-CS

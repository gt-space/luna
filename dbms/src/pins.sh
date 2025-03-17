#!/bin/bash

# P9 GPIO
config-pin p9.5 gpio #CS#
config-pin p9.11 gpio #DRDY#
config-pin p9.13 gpio #FLT#1
config-pin p9.14 gpio #FLT#2
config-pin p9.15 gpio #FLT#3
config-pin p9.16 gpio #FLT#4
config-pin p9.24 gpio #EN1
config-pin p9.25 gpio #EN2
config-pin p9.26 gpio #EN3
config-pin p9.27 gpio #EN4

# SPI 0
config-pin p9_18 spi #MISO
config-pin p9_21 spi #MOSI
config-pin p9_22 spi_sclk #SCLK
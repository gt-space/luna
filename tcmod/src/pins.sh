#!/bin/bash

# Chip‑select lines for each thermocouple ADC bank
config-pin p9.11 gpio    # CS‑TC1
config-pin p9.12 gpio    # CS‑TC2
config-pin p9.14 gpio    # CS‑TC3
config-pin p9.17 gpio    # CS‑TC4

# SPI0 bus 
config-pin p9_18 spi      # MISO
config-pin p9_21 spi      # MOSI
config-pin p9_22 spi_sclk # SCLK
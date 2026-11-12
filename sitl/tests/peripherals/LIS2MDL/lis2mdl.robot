*** Settings ***
Documentation     LIS2MDL magnetometer model. The cases and expected bytes come
...               from sitl/scripts/test_lis2mdl.resc, which printed them for
...               manual checking; here they are asserted.
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Create Magnetometer
Test Teardown     Test Teardown
Resource          ${CURDIR}/../../common/peripheral.resource

*** Variables ***
${MAG}            sysbus.spi1.mag

*** Keywords ***
Create Magnetometer
    Reset Emulation
    Create Machine With Peripheral    LIS2MDL    mag: Sensors.LIS2MDL @ spi1

Write Register
    [Arguments]    ${register}    ${value}
    SPI Transfer    ${MAG}    ${register}    ${value}

Trigger Single Measurement
    Write Register    0x60    0x01

Set Field
    [Arguments]    ${x}    ${y}    ${z}
    Execute Command    ${MAG} MagneticFieldX ${x}
    Execute Command    ${MAG} MagneticFieldY ${y}
    Execute Command    ${MAG} MagneticFieldZ ${z}

*** Test Cases ***
Should Report Its Device Id
    SPI Read Should Return    ${MAG}    0xCF    0x40

Should Reset Configuration Registers
    [Documentation]    CFG_REG_A/B/C and INT_CRTL_REG after reset.
    SPI Read Should Return    ${MAG}    0xE0    0x03    0x00    0x00    0xE0

Should Report Data Ready After A Measurement
    Set Field    1.5    -0.75    0.3
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xE7    0x0F

Should Report The Applied Magnetic Field
    [Documentation]    1.5 mG/LSB: 1.5 G = 1000, -0.75 G = -500, 0.3 G = 200.
    Set Field    1.5    -0.75    0.3
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xE8    0xE8    0x03    0x0C    0xFE    0xC8    0x00

Should Report Positive Temperature
    [Documentation]    8 LSB/degC around 25 degC: 37 degC = 96.
    Execute Command    ${MAG} Temperature 37
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xEE    0x60    0x00

Should Report Negative Temperature
    [Documentation]    -10 degC = -280.
    Execute Command    ${MAG} Temperature -10
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xEE    0xE8    0xFE

Should Apply The Hard Iron Offset
    [Documentation]    X offset of 256 LSB on a 1000 LSB field reads 1256.
    Set Field    1.5    -0.75    0.3
    Write Register    0x45    0x00
    Write Register    0x46    0x01
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xE8    0xE8    0x04

Should Swap Byte Order When BLE Is Set
    Set Field    1.5    -0.75    0.3
    Write Register    0x45    0x00
    Write Register    0x46    0x01
    Write Register    0x62    0x08
    Trigger Single Measurement
    SPI Read Should Return    ${MAG}    0xE8    0x04    0xE8

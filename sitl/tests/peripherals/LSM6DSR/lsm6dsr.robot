*** Settings ***
Documentation     LSM6DSR IMU model. Expected values come from the datasheet:
...               0.244 mg/LSB at +/-8 g and 70 mdps/LSB at 2000 dps.
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Create IMU
Test Teardown     Test Teardown
Resource          ${CURDIR}/../../common/peripheral.resource

*** Variables ***
${IMU}            sysbus.spi1.imu

*** Keywords ***
Create IMU
    Reset Emulation
    Create Machine With Peripheral    LSM6DSR    imu: Sensors.LSM6DSR @ spi1

*** Test Cases ***
Should Report Its Device Id
    SPI Read Should Return    ${IMU}    0x8F    0x6B

Should Read Back A Control Register
    SPI Transfer    ${IMU}    0x12    0x44
    SPI Read Should Return    ${IMU}    0x92    0x44

Should Scale Acceleration
    [Documentation]    1 g = 1000 mg / 0.244 = 4098 = 0x1002 on OUTZ_XL.
    Execute Command    ${IMU} AccelerationZ 1
    SPI Read Should Return    ${IMU}    0xAC    0x02    0x10

Should Scale Negative Acceleration
    [Documentation]    -1 g = -4098 = 0xEFFE on OUTY_XL.
    Execute Command    ${IMU} AccelerationY -1
    SPI Read Should Return    ${IMU}    0xAA    0xFE    0xEF

Should Scale Angular Rate
    [Documentation]    70 dps = 70000 mdps / 70 = 1000 = 0x03E8 on OUTX_G.
    Execute Command    ${IMU} AngularRateX 70
    SPI Read Should Return    ${IMU}    0xA2    0xE8    0x03

Should Read All Accelerometer Axes In One Burst
    [Documentation]    IF_INC is set at reset, so the address auto-increments.
    Execute Command    ${IMU} AccelerationZ 1
    SPI Read Should Return    ${IMU}    0xA8    0x00    0x00    0x00    0x00    0x02    0x10

Should Saturate Beyond Full Scale
    [Documentation]    10 g is past the +/-8 g range and clamps to 0x7FFF.
    Execute Command    ${IMU} AccelerationX 10
    SPI Read Should Return    ${IMU}    0xA8    0xFF    0x7F

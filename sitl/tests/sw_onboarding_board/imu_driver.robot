*** Settings ***
Documentation     SITL tests for the SW Onboarding Board IMU driver.
...
...               The firmware configures the LSM6DSR, reads WHO_AM_I every
...               loop pass, and drives the debug LEDs: D9 on a match, D11 on a
...               mismatch or SPI error. It also prints a report once a second
...               over semihosting. Both are asserted here.
Suite Setup       Setup
Suite Teardown    Teardown
Test Setup        Reset Emulation
Test Teardown     Test Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${SITL}           ${CURDIR}/../..
${ELF}            ${CURDIR}/../../../onboarding/imu_driver/build/Debug/imu_driver.elf

*** Keywords ***
Create Machine
    Execute Command    include @${SITL}/peripherals/STM32F0_SPI.cs
    Execute Command    include @${SITL}/peripherals/LSM6DSR.cs
    Execute Command    mach create "sw-onboarding-board"
    Execute Command    machine LoadPlatformDescription @${SITL}/platforms/sw_onboarding_board.repl
    Execute Command    sysbus LoadELF @${ELF}

*** Test Cases ***
Should Configure The IMU
    [Documentation]    Every CTRL register write must read back unchanged.
    Create Machine
    ${uart}=    Create Terminal Tester    sysbus.cpu.semihosting.console    defaultPauseEmulation=true
    Start Emulation
    Wait For Line On Uart    imu_driver started    testerId=${uart}
    Wait For Line On Uart    IMU configured OK     testerId=${uart}

Should Report The Correct Device Id
    [Documentation]    The LSM6DSR answers 0x6B, and the driver agrees.
    Create Machine
    ${uart}=    Create Terminal Tester    sysbus.cpu.semihosting.console    defaultPauseEmulation=true
    Start Emulation
    # Renode's Robot bridge splits arguments on '=', so an expected string
    # containing one has to be matched as a regex with '.' in its place.
    Wait For Line On Uart    WHO_AM_I.0x6B    treatAsRegex=true    testerId=${uart}

Should Accept The Device Id The Imu Reports
    [Documentation]    D9 is the driver's "IMU recognised" signal and D11 its
    ...                error signal, so this asserts the driver agreed with the
    ...                part that is actually fitted.
    Create Machine
    ${d9}=     Create LED Tester    sysbus.gpioPortB.ledD9
    ${d11}=    Create LED Tester    sysbus.gpioPortB.ledD11
    Start Emulation
    Assert LED State    true     timeout=2    testerId=${d9}
    Assert LED State    false    timeout=2    testerId=${d11}

Should Hold The 200 Hz Loop Without Overruns
    [Documentation]    Checkpoint 3: the deadline scheduler keeps its period.
    Create Machine
    ${uart}=    Create Terminal Tester    sysbus.cpu.semihosting.console    defaultPauseEmulation=true
    Start Emulation
    Wait For Line On Uart    rate.(199|200|201) Hz overruns.0 read_fail.0
    ...    treatAsRegex=true    testerId=${uart}

Should Measure One G On Z
    [Documentation]    With 1 g injected on Z the driver must scale it to
    ...                about +1000 mg, using the datasheet's 0.244 mg/LSB.
    Create Machine
    Execute Command    sysbus.spi1.imu AccelerationZ 1
    ${uart}=    Create Terminal Tester    sysbus.cpu.semihosting.console    defaultPauseEmulation=true
    Start Emulation
    Wait For Line On Uart    Accel \\[mg\\] x.0 y.0 z.99\\d
    ...    treatAsRegex=true    testerId=${uart}

*** Settings ***
Documentation     Shared keywords for testing a Renode peripheral model on its
...               own, without any firmware. The model is attached to a bus on
...               a stock Renode board and driven directly from the test, the
...               way a driver would drive it.
Resource          ${RENODEKEYWORDS}

*** Variables ***
${PERIPHERALS}    ${CURDIR}/../../peripherals
${TEST_BOARD}     platforms/boards/stm32f4_discovery.repl

*** Keywords ***
Create Machine With Peripheral
    [Documentation]    Loads `${model}.cs` and attaches it to the test board
    ...                using a platform description line such as
    ...                `mag: Sensors.LIS2MDL @ spi1`.
    [Arguments]    ${model}    ${attachment}
    Execute Command    include @${PERIPHERALS}/${model}.cs
    Execute Command    mach create "peripheral-test"
    Execute Command    machine LoadPlatformDescription @${TEST_BOARD}
    Execute Command    machine LoadPlatformDescriptionFromString "${attachment}"

SPI Transfer
    [Documentation]    Sends each byte to the peripheral in one transaction and
    ...                returns the bytes it answered with, as a list of
    ...                "0xNN" strings.
    [Arguments]    ${peripheral}    @{bytes}
    @{replies}=    Create List
    FOR    ${byte}    IN    @{bytes}
        ${reply}=    Execute Command    ${peripheral} Transmit ${byte}
        ${reply}=    Normalize Byte    ${reply}
        Append To List    ${replies}    ${reply}
    END
    Execute Command    ${peripheral} FinishTransmission
    RETURN    ${replies}

SPI Read Should Return
    [Documentation]    Sends a command byte followed by dummy bytes and checks
    ...                the bytes returned after the command.
    [Arguments]    ${peripheral}    ${command}    @{expected}
    @{dummies}=    Create List
    FOR    ${_}    IN    @{expected}
        Append To List    ${dummies}    0x00
    END
    @{replies}=    SPI Transfer    ${peripheral}    ${command}    @{dummies}
    @{data}=    Get Slice From List    ${replies}    1
    @{wanted}=    Evaluate    ["0x%02X" % int(b, 16) for b in $expected]
    Lists Should Be Equal    ${data}    ${wanted}

Normalize Byte
    [Arguments]    ${raw}
    ${value}=    Evaluate    "0x%02X" % (int($raw.strip(), 0) & 0xFF)
    RETURN    ${value}

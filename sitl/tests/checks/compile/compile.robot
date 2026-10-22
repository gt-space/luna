*** Settings ***
Documentation     Every peripheral model in sitl/peripherals compiles and loads
...               in Renode. Catches broken C# before anyone's tests run.
Suite Setup       Setup
Suite Teardown    Teardown
Test Teardown     Test Teardown
Library           OperatingSystem
Resource          ${RENODEKEYWORDS}

*** Test Cases ***
Every Peripheral Model Compiles
    @{models}=    List Files In Directory    ${CURDIR}/../../../peripherals    *.cs    absolute
    Should Not Be Empty    ${models}
    FOR    ${model}    IN    @{models}
        Execute Command    include @${model}
    END

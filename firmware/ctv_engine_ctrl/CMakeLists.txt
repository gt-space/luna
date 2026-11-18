# CMake build for the dual-core STM32H745 engine controller (CM7 + CM4).
#
# Mirrors the STM32CubeIDE project (CM7/.cproject, CM4/.cproject) so both
# tools can build the same tree. Sources are taken from the CubeIDE metadata
# where possible, so CubeMX regenerations done from CubeIDE are picked up:
#   - HAL / LwIP files: the linked resources listed in <core>/.project
#   - Core/, LWIP/, Common/: everything on disk (globbed)
# Include paths and defines are copied from .cproject and must be kept in
# sync by hand if CubeMX adds a new middleware.
cmake_minimum_required(VERSION 3.22)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

project(ctv_engine_ctrl C ASM)

# Source files CubeIDE links into <core> from the repo root (<core>/.project).
function(cubeide_linked_sources core out)
  file(READ ${CMAKE_SOURCE_DIR}/${core}/.project xml)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/${core}/.project)
  string(REGEX MATCHALL "<locationURI>PARENT-1-PROJECT_LOC/[^<]+\\.c</locationURI>" links "${xml}")
  set(srcs)
  foreach(l IN LISTS links)
    string(REGEX REPLACE "<locationURI>PARENT-1-PROJECT_LOC/([^<]+)</locationURI>" "${CMAKE_SOURCE_DIR}/\\1" p "${l}")
    list(APPEND srcs ${p})
  endforeach()
  set(${out} ${srcs} PARENT_SCOPE)
endfunction()

set(COMMON_DEFS USE_HAL_DRIVER STM32H745xx USE_PWR_DIRECT_SMPS_SUPPLY $<$<CONFIG:Debug>:DEBUG>)
set(COMMON_INCS
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/CMSIS/Include
)

# Settings shared by both cores (CubeIDE defaults for Debug/Release).
function(stm32_target tgt cpu_flags linker_script)
  target_compile_definitions(${tgt} PRIVATE ${COMMON_DEFS})
  target_include_directories(${tgt} PRIVATE ${COMMON_INCS})
  target_compile_options(${tgt} PRIVATE
    ${cpu_flags} -mthumb --specs=nano.specs
    -ffunction-sections -fdata-sections -Wall
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
    $<$<CONFIG:Debug>:-g3 -O0>
    $<$<CONFIG:Release>:-g0 -Os>
  )
  target_link_options(${tgt} PRIVATE
    ${cpu_flags} -mthumb
    -T${linker_script}
    --specs=nosys.specs --specs=nano.specs -static
    -Wl,--gc-sections -Wl,-Map=$<TARGET_FILE_DIR:${tgt}>/${tgt}.map
    -Wl,--print-memory-usage
    -Wl,--start-group -lc -lm -Wl,--end-group
  )
  set_property(TARGET ${tgt} APPEND PROPERTY LINK_DEPENDS ${linker_script})
  add_custom_command(TARGET ${tgt} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${tgt}> $<TARGET_FILE_DIR:${tgt}>/${tgt}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${tgt}>
  )
endfunction()

# ---------------------------------------------------------------- CM7 core
cubeide_linked_sources(CM7 CM7_LINKED)
file(GLOB_RECURSE CM7_LOCAL CONFIGURE_DEPENDS
  CM7/Core/Src/*.c CM7/Core/Startup/*.s CM7/LWIP/*.c Common/Src/*.c)
add_executable(ctv_engine_ctrl_CM7 ${CM7_LOCAL} ${CM7_LINKED})
stm32_target(ctv_engine_ctrl_CM7
  "-mcpu=cortex-m7;-mfpu=fpv5-d16;-mfloat-abi=hard"
  ${CMAKE_SOURCE_DIR}/CM7/STM32H745XIHX_FLASH.ld)
target_compile_definitions(ctv_engine_ctrl_CM7 PRIVATE CORE_CM7)
target_include_directories(ctv_engine_ctrl_CM7 PRIVATE
  CM7/Core/Inc
  CM7/LWIP/App
  CM7/LWIP/Target
  Middlewares/Third_Party/LwIP/src/include
  Middlewares/Third_Party/LwIP/system
  Middlewares/Third_Party/LwIP/src/include/netif/ppp
  Middlewares/Third_Party/LwIP/src/include/lwip
  Middlewares/Third_Party/LwIP/src/include/lwip/apps
  Middlewares/Third_Party/LwIP/src/include/lwip/priv
  Middlewares/Third_Party/LwIP/src/include/lwip/prot
  Middlewares/Third_Party/LwIP/src/include/netif
  Middlewares/Third_Party/LwIP/src/include/compat/posix
  Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa
  Middlewares/Third_Party/LwIP/src/include/compat/posix/net
  Middlewares/Third_Party/LwIP/src/include/compat/posix/sys
  Middlewares/Third_Party/LwIP/src/include/compat/stdc
  Middlewares/Third_Party/LwIP/system/arch
)

# ---------------------------------------------------------------- CM4 core
cubeide_linked_sources(CM4 CM4_LINKED)
file(GLOB_RECURSE CM4_LOCAL CONFIGURE_DEPENDS
  CM4/Core/Src/*.c CM4/Core/Startup/*.s Common/Src/*.c)
add_executable(ctv_engine_ctrl_CM4 ${CM4_LOCAL} ${CM4_LINKED})
# CubeIDE's CM4 Debug config links with the MMT template script (512K flash).
stm32_target(ctv_engine_ctrl_CM4
  "-mcpu=cortex-m4;-mfpu=fpv4-sp-d16;-mfloat-abi=hard"
  ${CMAKE_SOURCE_DIR}/CM4/$<IF:$<CONFIG:Debug>,STM32H745XIHX_FLASH_MMT_TEMPLATE.ld,STM32H745XIHX_FLASH.ld>)
target_compile_definitions(ctv_engine_ctrl_CM4 PRIVATE CORE_CM4)
target_include_directories(ctv_engine_ctrl_CM4 PRIVATE CM4/Core/Inc)

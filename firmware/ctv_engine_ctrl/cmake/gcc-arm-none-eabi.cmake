# Cross-compile with arm-none-eabi-gcc (GNU Tools for STM32).
# The compiler must be on PATH: the STM32Cube VS Code extension handles this
# via cube-cmake; on the command line, put the bundle's bin/ dir on PATH.
set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_SIZE         arm-none-eabi-size)

set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Per-config flags are set per target in CMakeLists.txt (matching CubeIDE).
set(CMAKE_C_FLAGS_DEBUG     "")
set(CMAKE_C_FLAGS_RELEASE   "")
set(CMAKE_ASM_FLAGS_DEBUG   "")
set(CMAKE_ASM_FLAGS_RELEASE "")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PATH_HINT "C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin")

find_program(ARM_GCC arm-none-eabi-gcc HINTS ${TOOLCHAIN_PATH_HINT})
find_program(ARM_GXX arm-none-eabi-g++ HINTS ${TOOLCHAIN_PATH_HINT})
find_program(ARM_OBJCOPY arm-none-eabi-objcopy HINTS ${TOOLCHAIN_PATH_HINT})
find_program(ARM_SIZE arm-none-eabi-size HINTS ${TOOLCHAIN_PATH_HINT})
find_program(ARM_OBJDUMP arm-none-eabi-objdump HINTS ${TOOLCHAIN_PATH_HINT})

if(NOT ARM_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc not found! Please check toolchain installation.")
endif()

set(CMAKE_C_COMPILER ${ARM_GCC})
set(CMAKE_CXX_COMPILER ${ARM_GXX})
set(CMAKE_ASM_COMPILER ${ARM_GCC})

set(CMAKE_OBJCOPY ${ARM_OBJCOPY} CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${ARM_SIZE} CACHE INTERNAL "size tool")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

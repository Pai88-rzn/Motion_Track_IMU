cmake_minimum_required(VERSION 3.20)
project(STM32_IMU C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Device target definitions
add_definitions(
    -DSTM32F103xB
    -DUSE_HAL_DRIVER
    -D__ARM_ARCH_7M__=1
)

# Compiler flags
set(CPU_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${CPU_FLAGS} -x assembler-with-cpp")

# Linker flags
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS
    "${CPU_FLAGS} -T\"${LINKER_SCRIPT}\" -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -specs=nano.specs -lc -lm -Wl,--no-warn-rwx-segments"
)

# Include directories
include_directories(
    Core/Inc
    Drivers/BSP
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
    App
)

# Source files
set(SOURCES
    # Core
    Core/Src/main.c
    Core/Src/stm32f1xx_it.c
    Core/Src/system_stm32f1xx.c
    Core/Src/syscalls.c
    Core/Src/startup_stm32f103xb.s

    # Drivers BSP
    Drivers/BSP/bno055.c
    Drivers/BSP/onboard_leds.c

    # Application (Direct UART Telemetry Protocol)
    App/imu_app.c
    App/serial_protocol.c

    # STM32F1xx HAL Drivers
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Post-build: Generate .hex, .bin, and print memory footprint
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE_UTIL} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin and reporting memory size..."
)

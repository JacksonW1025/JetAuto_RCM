cmake_minimum_required(VERSION 3.10)

# Set the compilers
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING TRUE)


# Indicate the compiler works
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

# Set the project name
project(RosRobotControllerM4)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the executable
add_executable(${PROJECT_NAME})

# Set the C Standard
set(CMAKE_C_STANDARD 99)

# Set the CXX Standard
set(CMAKE_CXX_STANDARD 14)

# Set the ASM compiler
enable_language(ASM)

# Set the CPU, FPU and Float-ABI flags
set(CPU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# Set the MCU flags
set(MCU_FLAGS "${CPU_FLAGS} -mthumb")

# Set the C and CXX flags
set(CMAKE_C_FLAGS "${MCU_FLAGS} -DUSE_HAL_DRIVER -DSTM32F407xx -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP")
set(CMAKE_CXX_FLAGS "${MCU_FLAGS} -DUSE_HAL_DRIVER -DSTM32F407xx -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP")

# Set the ASM flags
set(CMAKE_ASM_FLAGS "${MCU_FLAGS}")

# Set the LDFLAGS
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -specs=nano.specs -TSTM32F407VETx_FLASH.ld -lc -lm -lnosys -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections")

# Include directories
include_directories(
    Core/Inc
    USB_HOST/App
    USB_HOST/Target
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/ST/STM32_USB_Host_Library/Core/Inc
    Middlewares/ST/STM32_USB_Host_Library/Class/HID/Inc
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include

    Hiwonder/Peripherals
    Hiwonder/Chassis
    Hiwonder/Misc
    Hiwonder/Portings
    Hiwonder/System
    Hiwonder/USB_HOST
    Third_Party/Lw
    Third_Party/U8g2
    Third_Party/RTT

)

# Source files
file(GLOB_RECURSE SOURCES "Core/Src/*.c" "USB_HOST/App/*.c" "USB_HOST/Target/*.c" "Drivers/STM32F4xx_HAL_Driver/Src/*.c" "Middlewares/Third_Party/FreeRTOS/Source/*.c" "Middlewares/ST/STM32_USB_Host_Library/Core/Src/*.c" "Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/*.c")

# ASM source files
file(GLOB_RECURSE ASM_SOURCES "*.s")

# Add the source and ASM files to the executable
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${ASM_SOURCES})

# Add custom command for .hex file generation
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMENT "Generating HEX file")

# Add custom command for .bin file generation
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Generating BIN file")
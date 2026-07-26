cmake_minimum_required(VERSION 3.22)

# This is always a bare-metal STM32 target. Keep plain IDE profiles from
# silently falling back to the host MinGW compiler when no preset is selected.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "STM32 ARM GCC toolchain")
endif()

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME BalanceCar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        BSP/Inc/Key.h
        BSP/Src/Key.c
        BSP/Inc/BlueToothSerial.h
        BSP/Src/BlueToothSerial.c
        BSP/Inc/Motor.h
        BSP/Src/Motor.c
        BSP/Inc/MPU6050.h
        BSP/Src/mpu6050.c
        BSP/Inc/Encoder.h
        BSP/Src/Encoder.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    BSP/Src/LED.c
    BSP/Src/font.c
    BSP/Src/oled.c
    App/Src/App.c
    App/Src/Command.c
    App/Src/PID.c
    App/Src/LongitudinalControl.c
    App/Src/BluetoothProtocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m

    # Add user defined libraries
)

# Expose a real firmware download target to CLion. Running the ELF as a normal
# desktop application is not meaningful for a bare-metal Cortex-M binary.
find_program(STM32_PROGRAMMER_CLI
    NAMES STM32_Programmer_CLI.exe STM32_Programmer_CLI)
if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash
        COMMAND "${STM32_PROGRAMMER_CLI}"
                -c port=SWD freq=1000
                -d $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
                -v -rst
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} through ST-Link"
        USES_TERMINAL
        VERBATIM)
endif()

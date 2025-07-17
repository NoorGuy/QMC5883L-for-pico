# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(HMC5883Compass C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(HMC5883Compass HMC5883Compass.c )

pico_set_program_name(HMC5883Compass "HMC5883Compass")
pico_set_program_version(HMC5883Compass "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(HMC5883Compass 0)
pico_enable_stdio_usb(HMC5883Compass 1)

# Add the standard library to the build
target_link_libraries(HMC5883Compass
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(HMC5883Compass PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(HMC5883Compass 
        hardware_i2c
        
        )

pico_add_extra_outputs(HMC5883Compass)


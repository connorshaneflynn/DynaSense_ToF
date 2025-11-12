cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Set MODEs
# Select mode by commenting or value
# set(MODE "ranging_single")
# set(MODE "ranging_double")
set(MODE "latency")

# MODE is passed to compiler and source is set
if(MODE STREQUAL "ranging_single")
    add_compile_definitions(RANGING_SINGLE)
    set(SRC_CODE Core/Src/vl53l5cx_ranging.c)
    set(INC_CODE Core/Inc/ranging)
elseif(MODE STREQUAL "ranging_double")
    add_compile_definitions(RANGING_DOUBLE)
    set(SRC_CODE Core/Src/vl53l5cx_ranging_twosensors.c)
    set(INC_CODE Core/Inc/ranging_twosensors)
elseif(MODE STREQUAL "latency")
    add_compile_definitions(HID_LATENCY_TEST)
    set(SRC_CODE Core/Src/other/hid_latency_test.c)
    set(INC_CODE Core/Inc/other)
else()
    message([FATAL_ERROR] "ERROR: CMake MODE was not set properly")
endif()

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DynaSense_ToF_L432KC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # main, middleware, drivers, etc. are compiled in cmake/stm32cubemx/CMakeLists.txt
    
    ${SRC_CODE}

    Drivers/VL53L5CX/Src/vl53l5cx_platform.c
    Drivers/VL53L5CX/Src/vl53l5cx_api.c
    
    Core/Src/usb_app.c
    )

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # main, middleware, drivers, etc. are compiled in cmake/stm32cubemx/CMakeLists.txt

    ${INC_CODE}
    Drivers/VL53L5CX/Inc

    # copied from cubemx cmakelists.txt so fix include errors
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32L4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
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

    # Add user defined libraries
)

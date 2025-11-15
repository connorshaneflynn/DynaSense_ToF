cmake_minimum_required(VERSION 3.10.0)
project(cdc_reader VERSION 0.1.0 LANGUAGES C CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    cdc_reader.cpp
)

# Create Executable
add_executable(${CMAKE_PROJECT_NAME} cdc_reader.cpp)

# Include directory for your own headers
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # ${CMAKE_SOURCE_DIR}/inc
)


# Packages
find_path(SP_INCLUDE_DIR libserialport.h)
find_library(SP_LIBRARY serialport)

target_include_directories(cdc_reader PRIVATE
    ${SP_INCLUDE_DIR}
)

target_link_libraries(cdc_reader PRIVATE
    ${SP_LIBRARY}
)


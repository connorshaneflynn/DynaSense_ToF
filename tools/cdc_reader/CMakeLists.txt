cmake_minimum_required(VERSION 3.10.0)
project(cdc_reader VERSION 0.1.0 LANGUAGES C CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# # Source files
# set(SOURCES
#     cdc_reader_singlefile.cpp
# )

# # Create Executable
# add_executable(${CMAKE_PROJECT_NAME} cdc_reader_singlefile.cpp)

# # Include directory for your own headers
# target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
#     # ${CMAKE_SOURCE_DIR}/inc
# )

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

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${SP_INCLUDE_DIR}
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    ${SP_LIBRARY}
)


# UTILS
# temp compile of other files
set(FNAME list_cdc_devices)
# Create Executable
add_executable(${FNAME} utils/list_cdc_devices.cpp)

# Include directory for your own headers
target_include_directories(${FNAME} PRIVATE
    # ${CMAKE_SOURCE_DIR}/inc
)


# Packages
target_include_directories(${FNAME} PRIVATE
    ${SP_INCLUDE_DIR}
)

target_link_libraries(${FNAME} PRIVATE
    ${SP_LIBRARY}
)
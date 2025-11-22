cmake_minimum_required(VERSION 3.10.0)
project(cdc_reader VERSION 0.1.0 LANGUAGES C CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################################

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

# # Packages
# find_path(SP_INCLUDE_DIR libserialport.h)
# find_library(SP_LIBRARY serialport)

# target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
#     ${SP_INCLUDE_DIR}
# )

# target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
#     ${SP_LIBRARY}
# )

###############################################

# find external libraries
find_library(SERIALPORT_LIB serialport REQUIRED)
if(NOT SERIALPORT_LIB)
    message(FATAL_ERROR "libserialport not found")
endif()

# create custom library
add_library(cdcreader STATIC
    cdc_reader.cpp
    cdc_reader.h
)

target_include_directories(cdcreader
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(cdcreader
    PRIVATE ${SERIALPORT_LIB}
)


# create a test main file
add_executable(${CMAKE_PROJECT_NAME}
    main.cpp
    cdc_reader.cpp
)

# Include directory for your own headers
target_include_directories(${CMAKE_PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE ${SERIALPORT_LIB}
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
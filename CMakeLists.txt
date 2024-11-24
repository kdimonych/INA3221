cmake_minimum_required(VERSION 3.13)

project(external-devices.ina3221 C CXX)

set(HEADER_LIST
    ExternalHardware/ina3221/INA3221.hpp)

set(SOURCE_LIST
    ExternalHardware/ina3221/INA3221.cpp)

# Add library cpp files
add_library(external-devices.ina3221 ${HEADER_LIST} ${SOURCE_LIST}) 

target_sources(external-devices.ina3221 
    INTERFACE ${HEADER_LIST}
    PUBLIC ${SOURCE_LIST}
)

target_link_libraries(external-devices.ina3221 abstract-platform.common abstract-platform.i2c)

# Add include directory
target_include_directories(external-devices.ina3221 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
# target_link_libraries(external-devices.ina3221 pico_stdlib hardware_pio)
install(TARGETS external-devices.ina3221 FILE_SET ${HEADER_LIST})
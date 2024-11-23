cmake_minimum_required(VERSION 3.13)

project(external-devices.ina3221)

# Add library cpp files
add_library(external-devices.ina3221) 

# Enable or disable exceptions based on the platform or configuration
option(USE_EXCEPTIONS "Enable exceptions" ON)

if(USE_EXCEPTIONS)
    target_compile_definitions(external-devices.ina3221 PUBLIC __EXCEPTIONS)
else()
    target_compile_definitions(external-devices.ina3221 PUBLIC -fno-exceptions)
endif()

set(HEADER_LIST
    INA3221.h )

set(SOURCE_LIST
    INA3221.cpp)

target_sources(external-devices.ina3221 
    INTERFACE ${HEADER_LIST}
    PUBLIC ${SOURCE_LIST}
)

target_link_libraries(external-devices.ina3221 abstract-platform.i2c)

# Add include directory
target_include_directories(external-devices.ina3221 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
# target_link_libraries(external-devices.ina3221 pico_stdlib hardware_pio)
# install(TARGETS external-devices.ina3221 FILE_SET INA3221.h)
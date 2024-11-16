cmake_minimum_required(VERSION 3.13)

project(ina3221 VERSION 0.1.0 LANGUAGES CXX C)

# Add library cpp files
add_library(ina3221 INTERFACE) 

# Enable or disable exceptions based on the platform or configuration
option(USE_EXCEPTIONS "Enable exceptions" ON)

if(USE_EXCEPTIONS)
    target_compile_definitions(ina3221 INTERFACE __EXCEPTIONS)
else()
    target_compile_definitions(ina3221 INTERFACE -fno-exceptions)
endif()

set(HEADER_LIST
    INA3221.h )

set(SOURCE_LIST
    INA3221.cpp)

target_sources(ina3221 
    INTERFACE ${HEADER_LIST}
    PUBLIC ${SOURCE_LIST}
)

# Add include directory
message(STATUS "Add include dirrectory:" ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(ina3221 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
# target_link_libraries(ina3221 INTERFACE pico_stdlib hardware_pio)
# install(TARGETS ina3221 FILE_SET INA3221.h)
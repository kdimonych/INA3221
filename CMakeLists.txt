cmake_minimum_required(VERSION 3.13)

project(ina3221 VERSION 0.1.0 LANGUAGES CXX C)

# Add library cpp files
add_library(ina3221 INTERFACE) 

set(PUBLIC_SOURCE_LIST
    INA3221.h )

set(PRIVATE_SOURCE_LIST
    INA3221.cpp)

target_sources(ina3221 
    INTERFACE ${PUBLIC_SOURCE_LIST}
    PRIVATE ${PRIVATE_SOURCE_LIST}
)

# Add include directory
message(STATUS "Add include dirrectory:" ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(ina3221 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
# target_link_libraries(ina3221 INTERFACE pico_stdlib hardware_pio)
install(TARGETS ina3221 FILE_SET INA3221.h)
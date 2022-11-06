cmake_minimum_required(VERSION 3.22)
project(serial)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
file(GLOB LIB_SOURCES
    "ofSerial.h"
    "ofSerial.cpp"
)
file(GLOB SOURCES
    "main.cpp"
    "ofSerial.h"
)

add_library(shared_serial SHARED ${LIB_SOURCES})
add_library(static_serial STATIC ${LIB_SOURCES})

add_executable(serial ${SOURCES} )
target_link_libraries(serial shared_serial)

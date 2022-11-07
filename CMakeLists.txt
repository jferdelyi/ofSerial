cmake_minimum_required(VERSION 3.22)
project(serial)

set(CMAKE_CXX_STANDARD 17)
SET(IS_STATIC "STATIC" CACHE STRING "If true compile static")
SET(WITH_DEMO "DEMO" CACHE STRING "If true compile the demo")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(include)
file(GLOB LIB_SOURCES
    "include/ofSerial.h"
    "src/ofSerial.cpp"
)
file(GLOB SOURCES
    "include/ofSerial.h"
    "example/main.cpp"
)

IF (${STATIC} STREQUAL "ON")
    add_library(ofserial STATIC ${LIB_SOURCES})
    IF (WIN32)
        target_link_libraries(ofserial Setupapi.lib)
    ENDIF()
ENDIF()

IF (${STATIC} STREQUAL "OFF")
    add_library(ofserial SHARED ${LIB_SOURCES})
    IF (WIN32)
        target_link_libraries(ofserial Setupapi.dll)
    ENDIF()
ENDIF()

IF (${DEMO} STREQUAL "ON")
    add_executable(serial ${SOURCES})
    target_link_libraries(serial ofserial)
ENDIF()
cmake_minimum_required(VERSION 3.7)

message(STATUS "CMAKE_BUILD_TYPE = " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 11)
message(STATUS "Starting build")
message(STATUS "USERNAME: " $ENV{USER})
set(CMAKE_VERBOSE_MAKEFILE ON)
project(modbustest)
set(PROJECT_NAME modbustest)



set(Sources
        main.cpp
        modbuslib/modbus_client.cpp
        modbuslib/modbus_crc_tables.c
        uart.cpp
        )
add_executable(${PROJECT_NAME} ${Sources})
include_directories(modbuslib)

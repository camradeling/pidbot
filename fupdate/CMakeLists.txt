cmake_minimum_required(VERSION 3.6)
project(fupdate)

set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME fupdate)

set(Sources
        main.cpp
        modbus_client.cpp
        fupdater.cpp
        )

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
include_directories(channellib)
include_directories(threadlib)
include_directories(${CMAKE_SOURCE_DIR}/../firmware)

add_subdirectory(channellib)
add_subdirectory(threadlib)

add_executable(${PROJECT_NAME} ${Sources})
target_link_libraries(${PROJECT_NAME} ChannelLib )
target_link_libraries(${PROJECT_NAME} ProgramThread )
target_link_libraries(${PROJECT_NAME} pthread )

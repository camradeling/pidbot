cmake_minimum_required(VERSION 3.6)
project(fupdate)

set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME fupdate)

set(Sources
        main.cpp
        modbuslib/modbus_client.cpp
        modbuslib/modbus_crc_tables.c
        mpoller.cpp
        fupdater.cpp
        )

set(PollerSources
        poller.cpp
	modbuslib/modbus_client.cpp
        modbuslib/modbus_crc_tables.c
        mpoller.cpp
		)        

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
include_directories(channellib)
include_directories(threadlib)
include_directories(modbuslib)
include_directories(${CMAKE_SOURCE_DIR}/../firmware)

add_subdirectory(channellib)
add_subdirectory(threadlib)

add_executable(${PROJECT_NAME} ${Sources})
target_link_libraries(${PROJECT_NAME} ChannelLib )
target_link_libraries(${PROJECT_NAME} ProgramThread )
target_link_libraries(${PROJECT_NAME} pthread )

add_executable(poller ${PollerSources})
target_link_libraries(poller ChannelLib )
target_link_libraries(poller ProgramThread )
target_link_libraries(poller pthread)
cmake_minimum_required(VERSION 3.7)
option(DEBUG_INFO "debug info" OFF)
message(STATUS "CMAKE_BUILD_TYPE = " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD 11)
message(STATUS "Starting build")
message(STATUS "USERNAME: " $ENV{USER})
#set(CMAKE_VERBOSE_MAKEFILE ON)
project(pidbot)
set(PROJECT_NAME pidbot)

set(FreeRTOSSources
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM3/port.c
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/queue.c
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/list.c
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/timers.c
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/tasks.c
        ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/portable/MemMang/heap_4.c
        )

set(PeripheralSources
        ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c
        ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
        ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
        ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
        ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
        ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Device/ST/STM32F10x/Source/Templates/system_stm32f10x.c
        )

set(USBSources
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/tusb.c
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
        )

set(Sources
        main.c
        bootloader.c
        eeprom.c
        Libraries/modbuslib/modbus.c
        globals.c
        serial.c
        stm32f10x_it.c
        stm32f10x_vector.c
        ${FreeRTOSSources}
        ${PeripheralSources}
        ${USBSources}
        )


#set(CROSS_PATH /home/$ENV{USER}/x-tools/arm-none-eabi/bin)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
#set(CMAKE_C_LINK_EXECUTABLE ${CROSS_PATH}/arm-none-eabi-ld)

set(LINKER_SCRIPT stm32f103md2.ld)
#set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
set(CMAKE_EXE_LINKER_FLAGS "-T ${PROJECT_SOURCE_DIR}/${LINKER_SCRIPT}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_FLAGS "-mcpu=cortex-m3 -mthumb -mno-thumb-interwork -mfpu=vfp -msoft-float -mfix-cortex-m3-ldrd -nostdlib -fdata-sections -ffunction-sections")
if(DEBUG_INFO)
        message(STATUS "DEBUG_INFO set")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
        add_definitions(-DDEBUG)
else()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -s")
endif()
add_executable(${PROJECT_NAME}.elf ${Sources} ${LINKER_SCRIPT})

target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/modbuslib)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM3)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Device/ST/STM32F10x/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src)

add_definitions(-DSTM32F10X_MD)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMAND ${CMAKE_SOURCE_DIR}/usage.sh ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        )

add_custom_target(USAGE
        ${CMAKE_SOURCE_DIR}/usage.sh ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

add_dependencies(${PROJECT_NAME}.elf USAGE)        
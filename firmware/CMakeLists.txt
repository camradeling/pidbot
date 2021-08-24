cmake_minimum_required(VERSION 3.7)
message(STATUS "CMAKE_BUILD_TYPE = " ${CMAKE_BUILD_TYPE})
message(STATUS "Starting build")
message(STATUS "USERNAME: " $ENV{USER})
#############################################################
project(pidbot)
set(PROJECT_NAME pidbot)
#############################################################
######## common options and definitions######################
option(DEBUG_INFO "debug info" OFF)
#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_C_FLAGS "-mcpu=cortex-m3 -mthumb -mno-thumb-interwork -mfpu=vfp -msoft-float -mfix-cortex-m3-ldrd -nostdlib -fdata-sections -ffunction-sections")
if(DEBUG_INFO)
        message(STATUS "DEBUG_INFO set")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
        add_definitions(-DDEBUG)
else()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Os -s")
endif()
add_definitions(-DSTM32F10X_MD)
add_definitions(-DSTM32F103x6)
message(STATUS "CMAKE_C_FLAGS ${CMAKE_C_FLAGS}")
add_custom_target(USAGE
        ${CMAKE_SOURCE_DIR}/usage.sh ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
#############################################################
####### sorces sets #########################################
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
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/common/tusb_fifo.c
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/device/usbd.c
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/device/usbd_control.c
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/class/cdc/cdc_device.c
        ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
        )

set(Firmware_Sources
        main.c
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

set(Bootloader_Sources
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
#############################################################
####### pidbot options ######################################
add_executable(${PROJECT_NAME}.elf ${Firmware_Sources} ${PROJECT_SOURCE_DIR}/${FIRMWARE_LINKER_SCRIPT})
#add_compile_definitions(${PROJECT_NAME}.elf PUBLIC )
set(FIRMWARE_LINKER_SCRIPT stm32f103md_firmware.ld)
target_link_options(${PROJECT_NAME}.elf PUBLIC "SHELL:-T ${PROJECT_SOURCE_DIR}/${FIRMWARE_LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--gc-sections")

target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/modbuslib)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM3)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Device/ST/STM32F10x/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMAND ${CMAKE_SOURCE_DIR}/usage.sh ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        )
add_dependencies(${PROJECT_NAME}.elf USAGE)
#############################################################
####### pidbot options ######################################
add_executable(bootloader.elf ${Bootloader_Sources} ${PROJECT_SOURCE_DIR}/${BOOTLOADER_LINKER_SCRIPT})
set(BOOTLOADER_LINKER_SCRIPT stm32f103md_bootloader.ld)
target_link_options(bootloader.elf PUBLIC "SHELL:-T ${PROJECT_SOURCE_DIR}/${BOOTLOADER_LINKER_SCRIPT} -Wl,-Map=${PROJECT_BINARY_DIR}/bootloader.map -Wl,--gc-sections")

target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR})
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/modbuslib)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/include)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/FreeRTOS-Kernel/portable/GCC/ARM_CM3)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/STM32F10x_StdPeriph_Driver/inc)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Device/ST/STM32F10x/Include)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/CMSIS/Include)
target_include_directories(bootloader.elf PUBLIC ${PROJECT_SOURCE_DIR}/Libraries/tinyusb/src)
add_custom_command(TARGET bootloader.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex bootloader.elf bootloader.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary bootloader.elf bootloader.bin
        COMMAND ${CMAKE_SOURCE_DIR}/usage.sh ${PROJECT_BINARY_DIR}/bootloader.map
        )
add_dependencies(bootloader.elf USAGE)
#############################################################
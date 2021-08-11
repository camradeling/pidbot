#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H
//------------------------------------------------------------------------------
//modbus registers configuration
#define MBHR_BOOTLOADER_STATUS      0
#define MBHR_REG_MY_MBADDR          1
#define MBHR_REG_FLASH_PAGE_SIZE    2
#define MBHR_REG_IN_UART_PACKS      3
#define MBHR_REG_IN_UART_PACKS_ERR  4
#define MBHR_WRITE_FLASH_ADDR       5
#define MBHR_WRITE_FLASH_BUF_0      6
#define MBHR_WRITE_FLASH_BUF_63     69
#define MBHR_FIRMWARE_BLOCK_CRC     70
#define MBHR_REG_COMMAND            71
#define MBHR_COMMAND_STATUS         72
#define MBHR_FIRMWARE_CRC16         73
#define MBHR_SPACE_LAST_ADDR               MBHR_FIRMWARE_CRC16
#define MBHR_SPACE_SIZE                    MBHR_SPACE_LAST_ADDR+1
//------------------------------------------------------------------------------
#define TXRX_BUFFER_SIZE 256
//------------------------------------------------------------------------------
#define BOOTLOADER_JUMP				0
#define BOOTLOADER_WAIT_30S			1
#define FIRMARE_RUNNING				2
//------------------------------------------------------------------------------
#define BOOTLOADER_JUMP_COUNTER     30000
//------------------------------------------------------------------------------
#endif/*MODBUS_CONFIG_H*/
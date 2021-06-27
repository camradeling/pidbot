#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H
//------------------------------------------------------------------------------
//modbus registers configuration
#define MBHR_REG_MY_MBADDR          0
#define MBHR_TEST_VALUE             1
#define MBHR_REG_FLASH_PAGE_SIZE    2
#define MBHR_REG_IN_UART_PACKS      3
#define MBHR_REG_IN_UART_PACKS_ERR  4
#define MBHR_COMMAND_STATUS         5
#define MBHR_REG_COMMAND            6
#define MBHR_WRITE_FLASH_ADDR       7
#define MBHR_WRITE_FLASH_BUF_0      8
#define MBHR_WRITE_FLASH_BUF_63     71
#define MBHR_FIRMWARE_STATUS        72
#define MBHR_FIRMWARE_CRC16         73
#define MBHR_SPACE_LAST_ADDR               MBHR_FIRMWARE_CRC16
#define MBHR_SPACE_SIZE                    MBHR_SPACE_LAST_ADDR+1
//------------------------------------------------------------------------------
#define TXRX_BUFFER_SIZE 256
//------------------------------------------------------------------------------
#endif/*MODBUS_CONFIG_H*/
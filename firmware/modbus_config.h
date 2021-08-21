#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H
//------------------------------------------------------------------------------
//modbus registers configuration
#define MBHR_DUMMY1			        0
#define MBHR_DUMMY2		            1
#define MBHR_REG_FLASH_PAGE_SIZE    2
#define MBHR_REG_IN_UART_PACKS      3
#define MBHR_REG_IN_UART_PACKS_ERR  4
#define MBHR_WRITE_FLASH_ADDR       5
#define MBHR_WRITE_FLASH_BUF_0      6
#define MBHR_WRITE_FLASH_BUF_63     69
#define MBHR_FIRMWARE_BLOCK_LEN     70
#define MBHR_FIRMWARE_BLOCK_CRC     71
#define MBHR_REG_COMMAND            72
#define MBHR_COMMAND_STATUS         73
#define MBHR_REG_MY_MBADDR          74
#define MBHR_BOOTLOADER_STATUS      75
#define MBHR_FIRMWARE_FULL_LEN      76
#define MBHR_FIRMWARE_CRC16         77
#define MBHR_SPACE_LAST_ADDR               MBHR_FIRMWARE_CRC16
#define MBHR_SPACE_SIZE                    MBHR_SPACE_LAST_ADDR+1
//------------------------------------------------------------------------------
#define MAX_FIRMWARE_BLOCK_SIZE		(64*2)
#define TXRX_BUFFER_SIZE 256
#define MODBUS_03_DATASTART_IND		3
//------------------------------------------------------------------------------
#define CMD_WRITE_FIRMWARE_BLOCK		5607
#define CMD_START_FIRMWARE				5608
#define CMD_REBOOT						6666
//------------------------------------------------------------------------------
#define COMMAND_STATUS_OK			1
#define COMMAND_STATUS_FAILED		0
//------------------------------------------------------------------------------
#define BOOTLOADER_JUMP				0
#define BOOTLOADER_WAIT_30S			1
#define FIRMARE_RUNNING				2
//------------------------------------------------------------------------------
#define BOOTLOADER_JUMP_COUNTER     30000
//------------------------------------------------------------------------------
#endif/*MODBUS_CONFIG_H*/
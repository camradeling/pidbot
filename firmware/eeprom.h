#ifndef EEPROM_H
#define EEPROM_H
//-----------------------------------------------------------------------------------------------
#define DEFAULT_FLASH_WRTMT 		1000
#define BSY_FLAG_TIMEOUT			-1
#define EOP_FLAG_TIMEOUT			-2
#define FIRMWARE_ADDR_INVALID		-3
#define FIRMWARE_PROGRAMMING_ERROR	-4
//-----------------------------------------------------------------------------------------------
int write_firmware_block(uint32_t addr, uint16_t* data, int len, int erase);
//-----------------------------------------------------------------------------------------------
int write_eeprom();
//-----------------------------------------------------------------------------------------------
int init_eeprom();
//-----------------------------------------------------------------------------------------------
#endif/*EEPROM_H*/
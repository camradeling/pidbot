#include <stdint.h>
#include "stm32f10x.h"
#include "app_config.h"
#include "modbus_config.h"
#include "modbus.h"
//------------------------------------------------------------------------------
extern int write_flash(uint32_t addr, uint16_t* data, int len, int erase);
extern int flash_erase(uint32_t addr);
extern uint16_t MODBUS_HR[];
//-----------------------------------------------------------------------------------------------
#define EEPROM_REG_MY_MBADDR          	0
#define EEPROM_REG_TEST_VALUE			1
#define EEPROM_NUM_REGS					(EEPROM_REG_TEST_VALUE + 1)
//-----------------------------------------------------------------------------------------------
int write_eeprom()
{
	int res = write_flash(EEPROM_START, &MODBUS_HR[MBHR_REG_MY_MBADDR], EEPROM_NUM_REGS*sizeof(uint16_t), 1);
	if(res < 0)
	{
		for(int i = 0; i < EEPROM_NUM_REGS; i++)
			MODBUS_HR[i] = res;
	}
	return res;
}
//-----------------------------------------------------------------------------------------------
int init_eeprom()
{
	for(int i = 0; i < EEPROM_NUM_REGS; i++)
		MODBUS_HR[MBHR_REG_MY_MBADDR+i] = *(uint16_t*)(EEPROM_START + i*sizeof(uint16_t));
	return 0;
}
//-----------------------------------------------------------------------------------------------

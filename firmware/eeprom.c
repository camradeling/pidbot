#include <stdint.h>
#include "stm32f10x.h"
#include "app_config.h"
#include "modbus_config.h"
#include "modbus.h"
#include "eeprom.h"
//-----------------------------------------------------------------------------------------------
#define EEPROM_BACKUP_START_REG			MBHR_REG_MY_MBADDR
#define EEPROM_BACKUP_LAST_REG			MBHR_FIRMWARE_CRC16
#define EEPROM_NUM_REGS					(EEPROM_BACKUP_LAST_REG - EEPROM_BACKUP_START_REG+1)
//-----------------------------------------------------------------------------------------------
extern uint16_t MODBUS_HR[];
//-----------------------------------------------------------------------------------------------
int flash_unlock()
{
	int32_t tmt=DEFAULT_FLASH_WRTMT;
	if(FLASH->CR & FLASH_CR_LOCK)
	{	// Разблокировка LOCK при необходимости.
		FLASH->KEYR=FLASH_KEY1;
		FLASH->KEYR=FLASH_KEY2;
		while((FLASH->SR & FLASH_SR_BSY) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
		if(FLASH->SR & FLASH_SR_PGERR)
			return FIRMWARE_PROGRAMMING_ERROR;
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
	}
	return 0;
}
//-----------------------------------------------------------------------------------------------
int flash_erase(uint32_t addr)
{
	int32_t tmt=DEFAULT_FLASH_WRTMT;
	while((FLASH->SR & FLASH_SR_BSY) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
	if(FLASH->SR & FLASH_SR_PGERR)
		return FIRMWARE_PROGRAMMING_ERROR;		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
	if(tmt <= 0)
		return BSY_FLAG_TIMEOUT;
	FLASH->CR |= FLASH_CR_PER;		// Разрешение стирания страницы 1 кБайт.
	FLASH->AR=addr;		// Занести адрес стираемой страницы.
	FLASH->CR |= FLASH_CR_STRT;		// Старт стирания.
	tmt=DEFAULT_FLASH_WRTMT;
	while((FLASH->SR & FLASH_SR_BSY) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
	if(FLASH->SR & FLASH_SR_PGERR)
		return FIRMWARE_PROGRAMMING_ERROR;		// Ожидание конца стирания.
	if(tmt <= 0)
		return BSY_FLAG_TIMEOUT;
	// Ожидание конца операции.
	tmt=DEFAULT_FLASH_WRTMT;
	FLASH->CR &=~(FLASH_CR_PER | FLASH_CR_STRT);		// Вернуть взад.
	while(!(FLASH->SR & FLASH_SR_EOP) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
	if(FLASH->SR & FLASH_SR_PGERR)
		return FIRMWARE_PROGRAMMING_ERROR;
	if(tmt <= 0)
		return EOP_FLAG_TIMEOUT;
	FLASH->SR |= FLASH_SR_EOP;		// Закончить операцию.
	return 0;
}
//-----------------------------------------------------------------------------------------------
int write_flash(uint32_t addr, uint16_t* data, int len, int erase)
{
	uint32_t PageStart = addr & FLASH_PAGE_SIZE_MASK;
	int eres = 0;
	int bytesleft=len;
	int32_t tmt=DEFAULT_FLASH_WRTMT;
	eres = flash_unlock();
	if(eres)
		return eres;
	if(erase)
	{
		eres = flash_erase(PageStart);
		if(eres)
			return eres;
	}
	PageStart=addr;
	while((addr < PageStart+len) && (bytesleft > 0))
	{
		tmt=DEFAULT_FLASH_WRTMT;
		while((FLASH->SR & FLASH_SR_BSY) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
		if(FLASH->SR & FLASH_SR_PGERR)
			return FIRMWARE_PROGRAMMING_ERROR;		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
		FLASH->CR |= FLASH_CR_PG;		// Старт программирования.
		*(u16 *)addr=*data;		// Запись данного data по адресу addr.
		tmt=DEFAULT_FLASH_WRTMT;
		while((FLASH->SR & FLASH_SR_BSY) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);
		if(FLASH->SR & FLASH_SR_PGERR)
			return FIRMWARE_PROGRAMMING_ERROR;		// Ожидание конца программирования.
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
		FLASH->CR &=~FLASH_CR_PG;		// Конец программирования.
		tmt=DEFAULT_FLASH_WRTMT;
		while(!(FLASH->SR & FLASH_SR_EOP) && !(FLASH->SR & FLASH_SR_PGERR) && tmt--);		// Ожидание конца операции.
		if(FLASH->SR & FLASH_SR_PGERR)
			return FIRMWARE_PROGRAMMING_ERROR;
		if(tmt <= 0)
			return EOP_FLAG_TIMEOUT;
		FLASH->SR |= FLASH_SR_EOP;		// Закончить операцию.
		addr += 2;
		bytesleft -= 2;
		data++;
	}
	FLASH->CR |= FLASH_CR_LOCK;//lock it back
	return 0;
}
//-----------------------------------------------------------------------------------------------
int write_firmware_block(uint32_t addr, uint16_t* data, int len, int erase)
{
	if(addr < FIRMWARE_START)
		return FIRMWARE_ADDR_INVALID;
	if(len%2)
		len+=1;
	if(addr+len > FIRMWARE_FINISH+1)
		return FIRMWARE_ADDR_INVALID;
	int res = write_flash(addr, data, len, erase);
	return res;
}
//-----------------------------------------------------------------------------------------------
int write_eeprom()
{
	int res = write_flash(EEPROM_START, &MODBUS_HR[EEPROM_BACKUP_START_REG], EEPROM_NUM_REGS*sizeof(uint16_t), 1);
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
		MODBUS_HR[EEPROM_BACKUP_START_REG+i] = *(uint16_t*)(EEPROM_START + i*sizeof(uint16_t));
	return 0;
}
//-----------------------------------------------------------------------------------------------

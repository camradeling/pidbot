#include <stdint.h>
#include "stm32f10x.h"
#include "app_config.h"
#include "bootloader.h"
//-----------------------------------------------------------------------------------------------
int flash_erase(uint32_t addr)
{
	int32_t tmt=DEFAULT_FLASH_WRTMT;
	if(FLASH->CR & FLASH_CR_LOCK)
	{	// Разблокировка LOCK при необходимости.
		FLASH->KEYR=FLASH_KEY1;
		FLASH->KEYR=FLASH_KEY2;
		while(FLASH->SR & FLASH_SR_BSY && tmt--);
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
	}
	tmt=DEFAULT_FLASH_WRTMT;
	while(FLASH->SR & FLASH_SR_BSY && tmt--);		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
	if(tmt <= 0)
		return BSY_FLAG_TIMEOUT;
	FLASH->CR |= FLASH_CR_PER;		// Разрешение стирания страницы 1 кБайт.
	FLASH->AR=addr;		// Занести адрес стираемой страницы.
	FLASH->CR |= FLASH_CR_STRT;		// Старт стирания.
	//NOP();		// Обязательная задержка.
	tmt=DEFAULT_FLASH_WRTMT;
	while(FLASH->SR & FLASH_SR_BSY && tmt--);		// Ожидание конца стирания.
	if(tmt <= 0)
		return BSY_FLAG_TIMEOUT;
	// Ожидание конца операции.
	tmt=DEFAULT_FLASH_WRTMT;
	FLASH->CR &=~(FLASH_CR_PER | FLASH_CR_STRT);		// Вернуть взад.
	while((FLASH->SR & FLASH_SR_EOP)==0);
	if(tmt <= 0)
		return EOP_FLAG_TIMEOUT;
	FLASH->SR |= FLASH_SR_EOP;		// Закончить операцию.
	return 0;
}
//-----------------------------------------------------------------------------------------------
int write_flash(uint32_t addr, uint16_t* data, int len, int erase)
{
	uint32_t PageStart = addr & FLASH_PAGE_SIZE_MASK;
	if(erase)
		flash_erase(PageStart);
	int bytesleft=len;
	int32_t tmt=DEFAULT_FLASH_WRTMT;
	if(FLASH->CR & FLASH_CR_LOCK)
	{	// Разблокировка LOCK при необходимости.
		FLASH->KEYR=FLASH_KEY1;
		FLASH->KEYR=FLASH_KEY2;
		while(FLASH->SR & FLASH_SR_BSY);
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
	}
	while((addr < PageStart+len) && (bytesleft > 0))
	{
		tmt=DEFAULT_FLASH_WRTMT;
		while(FLASH->SR & FLASH_SR_BSY);		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
		FLASH->CR |= FLASH_CR_PG;		// Старт программирования.
		*(u16 *)addr=*data;		// Запись данного data по адресу addr.
		tmt=DEFAULT_FLASH_WRTMT;
		while(FLASH->SR & FLASH_SR_BSY);		// Ожидание конца программирования.
		if(tmt <= 0)
			return BSY_FLAG_TIMEOUT;
		FLASH->CR &=~FLASH_CR_PG;		// Конец программирования.
		tmt=DEFAULT_FLASH_WRTMT;
		while((FLASH->SR & FLASH_SR_EOP)==0);		// Ожидание конца операции.
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

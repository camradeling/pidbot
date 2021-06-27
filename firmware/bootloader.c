#include <stdint.h>
#include "stm32f10x.h"
#include "app_config.h"
//-----------------------------------------------------------------------------------------------
int flash_erase(uint32_t addr)
{
	while(FLASH->SR & FLASH_SR_BSY);		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
	FLASH->CR |= FLASH_CR_PER;		// Разрешение стирания страницы 1 кБайт.
	FLASH->AR=addr;		// Занести адрес стираемой страницы.
	FLASH->CR |= FLASH_CR_STRT;		// Старт стирания.
	//NOP();		// Обязательная задержка.
	while(FLASH->SR & FLASH_SR_BSY);		// Ожидание конца стирания.
	while((FLASH->SR & FLASH_SR_EOP)==0);		// Ожидание конца операции.
	FLASH->CR &=~FLASH_CR_PER;		// Вернуть взад.
	FLASH->SR |= FLASH_SR_EOP;		// Закончить операцию.
	return 0;
}
//-----------------------------------------------------------------------------------------------
int write_flash(uint32_t addr, uint16_t* data, int len, int erase)
{
	uint32_t PageStart = addr & (0xffffffff & FLASH_PAGE_SIZE);
	if(erase)
		flash_erase(PageStart);
	int bytesleft=len;
	if(FLASH->CR & FLASH_CR_LOCK)
	{	// Разблокировка LOCK при необходимости.
		FLASH->KEYR=FLASH_KEY1;
		FLASH->KEYR=FLASH_KEY2;
		while(FLASH->SR & FLASH_SR_BSY);
	}
	while(addr < PageStart && bytesleft > 0)
	{
		while(FLASH->SR & FLASH_SR_BSY);		// Дождаться конца незаконченной операции. Разблокировка проводилась при ините.
		FLASH->CR |= FLASH_CR_PG;		// Старт программирования.
		*(u16 *)addr=*data;		// Запись данного data по адресу addr.
		while(FLASH->SR & FLASH_SR_BSY);		// Ожидание конца программирования.
		while((FLASH->SR & FLASH_SR_EOP)==0);		// Ожидание конца операции.
		FLASH->CR &=~FLASH_CR_PG;		// Конец программирования.
		FLASH->SR |= FLASH_SR_EOP;		// Закончить операцию.
		addr += 2;
		bytesleft -= 2;
	}
	FLASH->CR |= FLASH_CR_LOCK;//lock it back
	return 0;
}
//-----------------------------------------------------------------------------------------------
int write_firmware_block(uint32_t addr, uint16_t* data, int len, int erase)
{
	if(addr < FIRMWARE_START)
		return -1;
	if(len%2)
		len+=1;
	if(addr+len > FIRMWARE_FINISH+1)
		return -1;
	write_flash(addr, data, len, erase);
	return 0;
}
//-----------------------------------------------------------------------------------------------

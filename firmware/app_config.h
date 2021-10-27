#ifndef APP_CONFIG_H
#define APP_CONFIG_H
//------------------------------------------------------------------------------
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
//------------------------------------------------------------------------------
#define FLASH_PAGE_SIZE             0x400
#define FLASH_PAGE_SIZE_MASK        0xfffffc00
#define BOOTLOADER_START            0x08000000
#define BOOTLOADER_FINISH           0x08007bff
#define EEPROM_START                0x08007c00
#define EEPROM_FINISH               0x08007fff
#define FIRMWARE_START              0x08008000
#define FIRMWARE_FINISH             0x08010000
//------------------------------------------------------------------------------
#define RUNLED		5
#define RUNLED_PORT GPIOB
#define COIL1 		15
#define COIL1_PORT 	GPIOB
#define COIL2 		14
#define COIL2_PORT 	GPIOB
#define USBDM		11
#define USBDM_PORT	GPIOA
#define USBDP 		12
#define USBDP_PORT	GPIOA
//------------------------------------------------------------------------------
#define mainCOM_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define INIT_BAUDRATE BAUDRATE_115200
#define INIT_COM_TIMER_PERIOD BAUDRATE_115200_TIMER_PERIOD
#define INIT_COM_TIMER_PRESCALER BAUDRATE_115200_TIMER_PRESCALER
#define INIT_COM_TIMER_CLOCK_DIVIDER BAUDRATE_115200_TIMER_CLOCK_DIVIDER
//------------------------------------------------------------------------------
#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
#define REBOOT()   SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04                                     
//------------------------------------------------------------------------------
#define SET_PIN_HIGH(PORT,PIN) PORT->ODR |= (1 << PIN);
#define SET_PIN_LOW(PORT,PIN)  PORT->ODR &=~(1 << PIN);
#define SET_PIN_OUTPUT_PP(PORT,PIN) if(PIN>=8) {PORT->CRH&=~((uint32_t)0x0f<<((PIN%8) << 2));PORT->CRH|=((uint32_t)0x03<<((PIN%8) << 2));} \
							else{PORT->CRL&=~((uint32_t)0x0f<<((PIN%8) << 2));PORT->CRL|=((uint32_t)0x03<<((PIN%8) << 2));}
#define SET_PIN_OUTPUT_OD(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRH |= ((uint32_t)0x07 << ((PIN%8) << 2));} \
  							else{ PORT->CRL &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRL |= ((uint32_t)0x07 << ((PIN%8) << 2));}
#define SET_PIN_INPUT(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRH |= ((uint32_t)0x04 << ((PIN%8) << 2));} \
  							else{ PORT->CRL &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRL |= ((uint32_t)0x04 << ((PIN%8) << 2));}
#define SET_PIN_ALTMODE_PP(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRH |= ((uint32_t)0x0B << ((PIN%8) << 2));} \
  							else{ PORT->CRL &=~ ((uint32_t)0x0f << ((PIN%8) << 2)); PORT->CRL |= ((uint32_t)0x0B << ((PIN%8) << 2));}
#define ISHIGH(PORT,PIN) (PORT->ODR & (1 << PIN))?1:0                                           
//------------------------------------------------------------------------------
#define GPIO_TO_INT(GPIO) (((uint32_t)(&(GPIO->CRL)) - GPIOA_BASE)>>10);
//------------------------------------------------------------------------------
#endif/*APP_CONFIG_H*/
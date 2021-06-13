#ifndef APP_CONFIG_H
#define APP_CONFIG_H
//------------------------------------------------------------------------------
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
//------------------------------------------------------------------------------
#define DATA0           8
#define DATA0_PORT GPIOB
#define DATA1           9
#define DATA1_PORT GPIOB
#define DATA2           10
#define DATA2_PORT GPIOB
#define DATA3           11
#define DATA3_PORT GPIOB
#define DATA4           12
#define DATA4_PORT GPIOB
#define DATA5           13
#define DATA5_PORT GPIOB
#define DATA6           14
#define DATA6_PORT GPIOB
#define DATA7           15
#define DATA7_PORT GPIOB
//------------------------------------------------------------------------------
//#define IN_KEY 6
//#define IN_KEY_PORT GPIOC
#define OUT_1 6
#define OUT_1_PORT GPIOA
#define OUT_2 4
#define OUT_2_PORT GPIOA
//#define OUT_3 9
//#define OUT_3_PORT GPIOC
//#define OUT_4 5
//#define OUT_4_PORT GPIOC
#define IN_1 8
#define IN_1_PORT GPIOA
#define IN_2 11
#define IN_2_PORT GPIOA
//#define IN_3 12
//#define IN_3_PORT GPIOC
//#define SYN_SCAN 2
//#define SYN_SCAN_PORT GPIOD
//------------------------------------------------------------------------------
#define mainCOM_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define INIT_BAUDRATE BAUDRATE_115200
#define INIT_COM_TIMER_PERIOD BAUDRATE_115200_TIMER_PERIOD
#define INIT_COM_TIMER_PRESCALER BAUDRATE_115200_TIMER_PRESCALER
#define INIT_COM_TIMER_CLOCK_DIVIDER BAUDRATE_115200_TIMER_CLOCK_DIVIDER
#define comSTACK_SIZE				configMINIMAL_STACK_SIZE +1000
#define TXRX_BUFFER_SIZE 256
//------------------------------------------------------------------------------
#define CMD_WRITE_FLASH_PARAMETERS_BACKUP               5607
#define CMD_REWRITE_FLASH                               5608
#define CMD_REBOOT                                      6666
//------------------------------------------------------------------------------
#define REBOOT()   SCB->AIRCR = 0x05FA0000 | (u32)0x04                                     
//------------------------------------------------------------------------------
#define SET_PIN_HIGH(PORT,PIN) PORT->ODR |= (1 << PIN);

#define SET_PIN_LOW(PORT,PIN)  PORT->ODR &=~(1 << PIN);

#define SET_PIN_OUTPUT_PP(PORT,PIN) if(PIN>=8) {PORT->CRH&=~(0x0f<<((PIN-8) << 2));PORT->CRH|=(0x03<<((PIN-8) << 2));}else{PORT->CRL&=~(0x0f<<(PIN << 2));PORT->CRL|=(0x03<<(PIN << 2));}

#define SET_PIN_OUTPUT_OD(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ (0x0f << ((PIN - 8) << 2)); PORT->CRH |= (0x07 << ((PIN - 8) << 2));} \
                                            else{ PORT->CRL &=~ (0x0f << (PIN << 2)); PORT->CRL |= (0x07 << (PIN << 2));}

#define SET_PIN_INPUT(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ (0x0f << ((PIN - 8) << 2)); PORT->CRH |= (0x04 << ((PIN - 8) << 2));} \
                                            else{ PORT->CRL &=~ (0x0f << (PIN << 2)); PORT->CRL |= (0x04 << (PIN << 2));}

#define SET_PIN_ALTMODE_PP(PORT,PIN) if(PIN >= 8){ PORT->CRH &=~ (0x0f << ((PIN - 8) << 2)); PORT->CRH |= (0x0B << ((PIN - 8) << 2));} \
                                            else{ PORT->CRL &=~ (0x0f << (PIN << 2)); PORT->CRL |= (0x0B << (PIN << 2));}

#define SET_BUS_VAL(val)        if(val & (1 << 0)) {SET_PIN_HIGH(DATA0_PORT,DATA0);}else {SET_PIN_LOW(DATA0_PORT,DATA0);} \
                                if(val & (1 << 1)) {SET_PIN_HIGH(DATA1_PORT,DATA1);}else {SET_PIN_LOW(DATA1_PORT,DATA1);} \
                                if(val & (1 << 2)) {SET_PIN_HIGH(DATA2_PORT,DATA2);}else {SET_PIN_LOW(DATA2_PORT,DATA2);} \
                                if(val & (1 << 3)) {SET_PIN_HIGH(DATA3_PORT,DATA3);}else {SET_PIN_LOW(DATA3_PORT,DATA3);} \
                                if(val & (1 << 4)) {SET_PIN_HIGH(DATA4_PORT,DATA4);}else {SET_PIN_LOW(DATA4_PORT,DATA4);} \
                                if(val & (1 << 5)) {SET_PIN_HIGH(DATA5_PORT,DATA5);}else {SET_PIN_LOW(DATA5_PORT,DATA5);} \
                                if(val & (1 << 6)) {SET_PIN_HIGH(DATA6_PORT,DATA6);}else {SET_PIN_LOW(DATA6_PORT,DATA6);} \
                                if(val & (1 << 7)) {SET_PIN_HIGH(DATA7_PORT,DATA7);}else {SET_PIN_LOW(DATA7_PORT,DATA7);} 

#define GET_BUS_VAL()           (uint8_t)(((DATA0_PORT->IDR & (1 << DATA0))?(0 << 0):(1 << 0))| \
                                          ((DATA1_PORT->IDR & (1 << DATA1))?(0 << 1):(1 << 1))| \
                                          ((DATA2_PORT->IDR & (1 << DATA2))?(0 << 2):(1 << 2))| \
                                          ((DATA3_PORT->IDR & (1 << DATA3))?(0 << 3):(1 << 3))| \
                                          ((DATA4_PORT->IDR & (1 << DATA4))?(0 << 4):(1 << 4))| \
                                          ((DATA5_PORT->IDR & (1 << DATA5))?(0 << 5):(1 << 5))| \
                                          ((DATA6_PORT->IDR & (1 << DATA6))?(0 << 6):(1 << 6))| \
                                          ((DATA7_PORT->IDR & (1 << DATA7))?(0 << 7):(1 << 7)))

#define SET_BUS_OUTPUT_PP() SET_PIN_OUTPUT_PP(DATA0_PORT,DATA0); \
                            SET_PIN_OUTPUT_PP(DATA1_PORT,DATA1); \
                            SET_PIN_OUTPUT_PP(DATA2_PORT,DATA2); \
                            SET_PIN_OUTPUT_PP(DATA3_PORT,DATA3); \
                            SET_PIN_OUTPUT_PP(DATA4_PORT,DATA4); \
                            SET_PIN_OUTPUT_PP(DATA5_PORT,DATA5); \
                            SET_PIN_OUTPUT_PP(DATA6_PORT,DATA6); \
                            SET_PIN_OUTPUT_PP(DATA7_PORT,DATA7);
                            
#define SET_BUS_INPUT()     SET_PIN_INPUT(DATA0_PORT,DATA0); \
                            SET_PIN_INPUT(DATA1_PORT,DATA1); \
                            SET_PIN_INPUT(DATA2_PORT,DATA2); \
                            SET_PIN_INPUT(DATA3_PORT,DATA3); \
                            SET_PIN_INPUT(DATA4_PORT,DATA4); \
                            SET_PIN_INPUT(DATA5_PORT,DATA5); \
                            SET_PIN_INPUT(DATA6_PORT,DATA6); \
                            SET_PIN_INPUT(DATA7_PORT,DATA7);
//------------------------------------------------------------------------------
#define GPIO_TO_INT(GPIO) (((uint32_t)(&(GPIO->CRL)) - GPIOA_BASE)>>10);
//------------------------------------------------------------------------------
#endif/*APP_CONFIG_H*/
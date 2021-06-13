#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H
//------------------------------------------------------------------------------
#include "stm32f10x_conf.h"
//------------------------------------------------------------------------------
#define SPI_FLASH_PORT GPIOA
#define CS_FLASH1 4
#define SCK 5
#define MISO 6
#define MOSI 7
//------------------------------------------------------------------------------
/*EN_PORT*/
#define LCD_EN_PORT GPIOB
#define EN_SIG 2
#define LCD_BL_PORT GPIOA
#define LCD_BL 1
#define LCD_HIGHZ_PORT GPIOB
#define HIGHZ_5V 12
//------------------------------------------------------------------------------
/*Заголовочный файл для назначения портов и пинов*/
/*на каком порту управление сегментами индикаторов*/
#define LCD_SEGMENTS_PORT GPIOB
/*отступ в порту (необязательно нумерация начинается с нулевого пина)*/
#define LCD_SEGMENTS_PORT_OFFSET 8
#define LCD_CONTROL_PORT GPIOB
#define CS1 2
#define CS2 1
//------------------------------------------------------------------------------
#define DI_SIG 15 
#define CS1_SIG 14 
#define CS2_SIG 13 
#define LCD_RESET 12 
//------------------------------------------------------------------------------
#define CL1_LCD_PORT GPIOC
#define CL1_LCD 9 
#define CL1_KEYS 6 
//------------------------------------------------------------------------------
#define BUTTONS_PORT GPIOB
#define BUTTON0_SIG 4 
#define BUTTON1_SIG 5
#define BUTTON2_SIG 6
#define BUTTON3_SIG 7
#define BUTTON_BLOCK_PIN0 0
#define BUTTON_BLOCK_PIN1 1
#define BUTTON_BLOCK_PIN2 2
#define BUTTON_BLOCK_PIN3 3
#define BUTTON_BLOCK_PIN4 4
#define BUTTON_BLOCK_PIN5 5
//------------------------------------------------------------------------------
#endif//HARDWARE_CONFIG_H

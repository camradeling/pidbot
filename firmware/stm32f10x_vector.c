#include "stm32f10x_it.h"

extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void TIM2_IRQHandler( void );
extern void USART1_IRQHandler( void );
extern void vPortSVCHandler( void );

/* Private typedef -----------------------------------------------------------*/
typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#pragma language=extended
#pragma segment="CSTACK"

//void __iar_program_start( void );

#pragma location = ".intvec"

void Reset_Handler( void );
void *_estack;

const intvec_elem __vector_table[] __attribute__ ((section(".isr_vector"))) =
{
    { .__ptr = &_estack },
    Reset_Handler,
  NMIException,
  HardFaultException,
  MemManageException,
  BusFaultException,
  UsageFaultException,
  0, 0, 0, 0,            /* Reserved */
  vPortSVCHandler,
  DebugMonitor,
  0,                      /* Reserved */
  xPortPendSVHandler,
  xPortSysTickHandler,
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_IRQHandler,
  RTC_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMAChannel1_IRQHandler,
  DMAChannel2_IRQHandler,
  DMAChannel3_IRQHandler,
  DMAChannel4_IRQHandler,
  DMAChannel5_IRQHandler,
  DMAChannel6_IRQHandler,
  DMAChannel7_IRQHandler,
  ADC_IRQHandler,
  USB_HP_CAN_TX_IRQHandler,
  USB_LP_CAN_RX0_IRQHandler,
  CAN_RX1_IRQHandler,
  CAN_SCE_IRQHandler,
  EXTI9_5_IRQHandler,
  TIM1_BRK_IRQHandler,
  TIM1_UP_IRQHandler,
  TIM1_TRG_COM_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI15_10_IRQHandler,
  RTCAlarm_IRQHandler,
  USBWakeUp_IRQHandler,
};

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/


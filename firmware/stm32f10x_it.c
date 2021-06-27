#include "includes.h"
//------------------------------------------------------------------------------
extern void TimingDelay_Decrement(void);
//------------------------------------------------------------------------------
extern int main( void );
void* _sidata;
void* _sdata;
void* _edata;
void* _sbss;
void* _ebss;
void Reset_Handler(void)
{
  unsigned long *pulSrc, *pulDest;

  //
  // Copy the data segment initializers from flash to SRAM.
  //
  pulSrc = (unsigned long *)&_sidata;
  for(pulDest = (unsigned long *)&_sdata; pulDest < (unsigned long *)&_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }

  //
  // Zero fill the bss segment.
  //
  for(pulDest = (unsigned long *)&_sbss; pulDest < (unsigned long *)&_ebss; )
  {
    *(pulDest++) = 0;
  }

  //
  // Call the application's entry point.
  //
  main();
}
//------------------------------------------------------------------------------
void NMIException(void)
{
}
//------------------------------------------------------------------------------
void HardFaultException(void)
{
}
//------------------------------------------------------------------------------
void MemManageException(void)
{
}
//------------------------------------------------------------------------------
void BusFaultException(void)
{
}
//------------------------------------------------------------------------------
void UsageFaultException(void)
{
}
//------------------------------------------------------------------------------
void DebugMonitor(void)
{
}
//------------------------------------------------------------------------------
void SVCHandler(void)
{
}
//------------------------------------------------------------------------------
void PendSVC(void)
{
}
//------------------------------------------------------------------------------
void SysTickHandler(void)
{
  /* Decrement the TimingDelay variable */
  //TimingDelay_Decrement();
}
//------------------------------------------------------------------------------
void WWDG_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void PVD_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void TAMPER_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void RTC_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void FLASH_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void RCC_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI0_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI1_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI2_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI3_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI4_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void DMAChannel1_IRQHandler(void)
{  
}
//------------------------------------------------------------------------------
void DMAChannel2_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void DMAChannel3_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
/*void DMAChannel4_IRQHandler(void)
{
}*/
//------------------------------------------------------------------------------
/*void DMAChannel5_IRQHandler(void)
{
}*/
//------------------------------------------------------------------------------
void DMAChannel6_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void DMAChannel7_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void ADC_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void USB_HP_CAN_TX_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void USB_LP_CAN_RX0_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void CAN_RX1_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void CAN_SCE_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI9_5_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void TIM1_BRK_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
/*void TIM1_UP_IRQHandler(void)
{
}*/
//------------------------------------------------------------------------------
void TIM1_TRG_COM_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
/*void TIM1_CC_IRQHandler(void)
{
}*/
//------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
//------------------------------------------------------------------------------
uint32_t timerOVF = 0;
//------------------------------------------------------------------------------
void TIM3_IRQHandler(void)
{
  timerOVF++;
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
//------------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void I2C1_EV_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void I2C1_ER_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void I2C2_EV_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void I2C2_ER_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void SPI1_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void SPI2_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
/*void USART1_IRQHandler(void)
{
}*/
//------------------------------------------------------------------------------
void USART2_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void USART3_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void EXTI15_10_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void RTCAlarm_IRQHandler(void)
{
}
//------------------------------------------------------------------------------
void USBWakeUp_IRQHandler(void)
{
}
//------------------------------------------------------------------------------


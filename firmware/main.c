#include "includes.h"
//------------------------------------------------------------------------------
/*Configure the clocks, GPIO and other peripherals */
static void prvSetupHardware( void );
//------------------------------------------------------------------------------
static void vI2CTask (void *pvParameters);
//------------------------------------------------------------------------------
static void vInoutsTask (void *pvParameters);
//------------------------------------------------------------------------------
void vApplicationTickHook( void );
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
int main( void )
{
/*#ifdef DEBUG
  debug();
#endif*/
  prvSetupHardware();
  //SET_PIN_HIGH(GPIOB,5);
  Com1RxSemaphore = xSemaphoreCreateCounting(MAX_COM_QUEUE_LENGTH, 0);
  xTaskCreate(vPacketsManagerTask, "Packets_manager", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  //xTaskCreate(vI2CTask, "I2C", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  //xTaskCreate(vInoutsTask, "Inouts", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  //xTaskCreate(vSpiFlashTask, "Flash", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();

  return 0;
}
//------------------------------------------------------------------------------
void vInoutsTask (void *pvParameters)
{
  for(;;)
  {
    //MODBUS_HR[MBHR_DISCRETE_OUTPUTS_LOW] = MODBUS_HR[MBHR_DISCRETE_INPUTS_LOW];
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
//------------------------------------------------------------------------------
void vI2CTask (void *pvParameters)
{
  for(;;)
  {
    //vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
//------------------------------------------------------------------------------
void vApplicationTickHook( void )//вызывается каждую миллисекунду
{
    //return;
  uint8_t val = 0x00;
  SET_BUS_OUTPUT_PP();
  val = MODBUS_HR[MBHR_DISCRETE_OUTPUTS_LOW] & 0x00ff;
  SET_BUS_VAL(val);
  SET_PIN_LOW(OUT_1_PORT, OUT_1);
  SET_PIN_HIGH(OUT_1_PORT, OUT_1);
  val = (MODBUS_HR[MBHR_DISCRETE_OUTPUTS_LOW] & 0xff00) >> 8;
  SET_BUS_VAL(val);
  SET_PIN_LOW(OUT_2_PORT, OUT_2);
  SET_PIN_HIGH(OUT_2_PORT, OUT_2);

  SET_BUS_INPUT();
  SET_PIN_LOW(IN_1_PORT, IN_1);
  val = GET_BUS_VAL();
  SET_PIN_HIGH(IN_1_PORT, IN_1);
  MODBUS_HR[MBHR_DISCRETE_INPUTS_LOW] = (MODBUS_HR[MBHR_DISCRETE_INPUTS_LOW] & 0xff00) | val;
  SET_PIN_LOW(IN_2_PORT, IN_2);
  val = GET_BUS_VAL();
  SET_PIN_HIGH(IN_2_PORT, IN_2);
  MODBUS_HR[MBHR_DISCRETE_INPUTS_LOW] = (MODBUS_HR[MBHR_DISCRETE_INPUTS_LOW] & 0x00ff) | (uint16_t)((uint16_t)val << 8);
}
//------------------------------------------------------------------------------
static void prvSetupHardware( void )
{
  /* Start with the clocks in their expected state. */
  RCC_DeInit();
  /* Enable HSE (high speed external clock). */
  RCC_HSEConfig( RCC_HSE_ON );
  /* Wait till HSE is ready. */
  while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET ){}
  /* 2 wait states required on the flash. */
  *( ( unsigned long * ) 0x40022000 ) = 0x02;
  /* HCLK = SYSCLK */
  RCC_HCLKConfig( RCC_SYSCLK_Div1 );
  /* PCLK2 = HCLK */
  RCC_PCLK2Config( RCC_HCLK_Div1 );
  /* PCLK1 = HCLK/2 */
  RCC_PCLK1Config( RCC_HCLK_Div2 );
  /* PLLCLK = 8MHz * 9 = 72 MHz. */
  RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );
  /* Enable PLL. */
  RCC_PLLCmd( ENABLE );
  /* Wait till PLL is ready. */
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {
  }
  /* Select PLL as system clock source. */
  RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
  /* Wait till PLL is used as system clock source. */
  while( RCC_GetSYSCLKSource() != 0x08 )
  {
  }
  /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
  RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC \
						| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |  \
                                                  RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1 | \
                                                    RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE );
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  //enable dma clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );        
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  /* Configure HCLK clock as SysTick clock source. */
  SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  //пока что для отладки
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   
  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 35;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 0xffff;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
  TIM_Cmd(TIM3, ENABLE);
  
  if(FLASH->CR & FLASH_CR_LOCK)
  {// Разблокировка LOCK при необходимости.
    FLASH->KEYR=FLASH_KEY1;
    FLASH->KEYR=FLASH_KEY2;
    while(FLASH->SR & FLASH_SR_BSY);
  }
  //run led
  SET_PIN_LOW(GPIOB,5);
  SET_PIN_OUTPUT_PP(GPIOB,5);
  SET_PIN_HIGH(GPIOB,5);
  
  //SPI init
  //SET_PIN_HIGH(GPIOA,4);
  //SET_PIN_OUTPUT_PP(GPIOA,4);//NSS
  ////SET_PIN_ALTMODE_PP(GPIOA,5);//SCK
  //SET_PIN_INPUT(GPIOA,6);//MISO
  //SET_PIN_ALTMODE_PP(GPIOA,7);//MOSI
  //SPI1->CR1 &=~SPI_CR1_SPE;
  //SPI1->CR1 = 0x00000004;//8-bit master second rising edge sampling baudrate 36 mhz
  //SPI1->CR2 |= SPI_CR2_SSOE;
  //SPI1->CR1 |= SPI_CR1_SPE;//turn on spi
  //w25_init();
  
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = INIT_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //НОги Rx Tx USART1
  SET_PIN_ALTMODE_PP(GPIOA,9);
  SET_PIN_INPUT(GPIOA,10);
  USART_Init(USART1, &USART_InitStructure);
  USART_Cmd(USART1, ENABLE);
  
  //
  NVIC_InitTypeDef NVIC_InitStructure;
  //usart rx interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  //usart dma tx interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Not used as 4 bits are used for the pre-emption priority. 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  //usart dma rx interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Not used as 4 bits are used for the pre-emption priority. 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  //usart tim rx interrupt cc
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Not used as 4 bits are used for the pre-emption priority. 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  //usart tim rx interrupt upd
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Not used as 4 bits are used for the pre-emption priority. 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );
  //i2c1 interrupt
  /*NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Not used as 4 bits are used for the pre-emption priority. 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );*/
  
  
  //все что работает по i2c
  //i2cAddr = ADC1ADDR;
  //I2C1State = I2C_INITIATING_CONVERSION;
  //recover_I2C1();
  //I2C1->CR1 |= I2C_CR1_START;//а дальше вся работа в прерывании
  
  //BRST
  SET_PIN_HIGH(GPIOA,12);
  SET_PIN_OUTPUT_PP(GPIOA,12);
  uint8_t val = 0x00;
  //выставляем шину выходами
#ifndef NOMAL
  SET_PIN_INPUT(GPIOB,4);
  SET_PIN_INPUT(GPIOB,3);
#endif
  SET_PIN_OUTPUT_PP(OUT_1_PORT, OUT_1);
  SET_PIN_OUTPUT_PP(OUT_2_PORT, OUT_2);
    //SET_PIN_LOW(OUT_1_PORT,OUT_1);
    //SET_PIN_LOW(OUT_2_PORT,OUT_2);
  SET_PIN_HIGH(IN_1_PORT, IN_1);
  SET_PIN_OUTPUT_PP(IN_1_PORT, IN_1);
  SET_PIN_HIGH(IN_2_PORT, IN_2);
  SET_PIN_OUTPUT_PP(IN_2_PORT, IN_2);
  SET_BUS_OUTPUT_PP();
  SET_BUS_VAL(val);
  SET_PIN_LOW(OUT_1_PORT, OUT_1);
  SET_PIN_HIGH(OUT_1_PORT, OUT_1);
  SET_PIN_LOW(OUT_2_PORT, OUT_2);
  SET_PIN_HIGH(OUT_2_PORT, OUT_2);
  //включаем BRST
  SET_PIN_LOW(GPIOA,12);
}
//------------------------------------------------------------------------------
#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned char* pcFile, unsigned long ulLine )
{
	for( ;; )
	{
	}
}
#endif

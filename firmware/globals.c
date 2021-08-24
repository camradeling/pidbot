#define EXTERNS_H/*to prevent linker ambiguity*/
//------------------------------------------------------------------------------
#include "includes.h"
//------------------------------------------------------------------------------
SemaphoreHandle_t Com1RxSemaphore;
//SemaphoreHandle_t ComTxSemaphore;
//------------------------------------------------------------------------------
uint8_t com1TxBuffer[TXRX_BUFFER_SIZE];
uint8_t com1RxBuffer[TXRX_BUFFER_SIZE];
uint8_t transmitActive;
//------------------------------------------------------------------------------
ComMessage Com1RxQueue[MAX_COM_QUEUE_LENGTH];
ComMessage Com1TxQueue[MAX_COM_QUEUE_LENGTH];
uint8_t Com1RxReadInd;
uint8_t Com1RxWriteInd;
uint8_t Com1TxReadInd;
uint8_t Com1TxWriteInd;
//------------------------------------------------------------------------------
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
	/* If the buffers to be provided to the Idle task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------
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


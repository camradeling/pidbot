#ifndef EXTERNS_H
#define EXTERNS_H
//------------------------------------------------------------------------------
#include "includes.h"
//------------------------------------------------------------------------------
extern uint16_t MODBUS_HR[];
extern const uint8_t modbus_crc16H[];
extern const uint8_t modbus_crc16L[];
extern SemaphoreHandle_t ComRxSemaphore;
extern SemaphoreHandle_t ComTxSemaphore;
extern uint8_t com1TxBuffer[];
extern uint8_t com1RxBuffer[];
extern uint8_t transmitActive;
//------------------------------------------------------------------------------
extern SemaphoreHandle_t Com1RxSemaphore;
extern ComMessage Com1RxQueue[];
extern ComMessage Com1TxQueue[];
extern uint8_t Com1RxReadInd;
extern uint8_t Com1RxWriteInd;
extern uint8_t Com1TxReadInd;
extern uint8_t Com1TxWriteInd;
//------------------------------------------------------------------------------
#endif/*EXTERNS_H*/
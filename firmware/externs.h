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
extern uint8_t I2C1State;
extern uint8_t I2C1TransferStatus;
extern uint8_t i2cAddr;
extern uint8_t conversions;
extern uint8_t ADS1115_DIFSIG_Config_Buf[];
extern uint8_t ADS1115_REF_Config_Buf[];
extern uint8_t ADS_READ_Buf[];
//------------------------------------------------------------------------------
extern uint16_t flashParams[];
extern uint8_t flashBuf[];
extern uint16_t defaultParams[];
//------------------------------------------------------------------------------
#endif/*EXTERNS_H*/
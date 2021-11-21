#ifndef MODBUS_H
#define MODBUS_H
//------------------------------------------------------------------------------
#include "modbus_config.h"
//------------------------------------------------------------------------------
#define SWAP16(x) (((uint16_t)x << 8)|((uint16_t)x >> 8))
//------------------------------------------------------------------------------
typedef struct
{
  uint8_t data[TXRX_BUFFER_SIZE];
  uint16_t length;
}ComMessage;
//------------------------------------------------------------------------------
uint16_t* MyMBAddr;
//------------------------------------------------------------------------------
typedef int (*register_cb)(uint16_t regnum);
//------------------------------------------------------------------------------
//return codes
#define MODBUS_PACKET_VALID_AND_PROCESSED               0
#define MODBUS_PACKET_WRONG_ADDR                        1
#define MODBUS_PACKET_WRONG_CRC                         2
#define MODBUS_REGISTER_WRITE_PROTECTED                 3
#define MODBUS_REGISTER_NUMBER_INVALID                  4
#define MODBUS_REGISTER_WRITE_CALLBACK_FAILED           5
//------------------------------------------------------------------------------
#define MB_BROADCAST_ADDR                              0xff
#define MODBUS_03_DATASTART_IND							3
//------------------------------------------------------------------------------
uint8_t process_net_packet(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
uint16_t calc_crc(uint8_t *arr, uint8_t length);
//------------------------------------------------------------------------------
int process_modbus(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
int CmdModbus_03_04(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
int CmdModbus_06(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
int CmdModbus_08(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
int CmdModbus_16(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
#endif /*MODBUS_H*/
//modbus_config.h file should be created and
//MBHR_SPACE_SIZE and
//TXRX_BUFFER_SIZE should be defined in it
//also MODBUS_HR and 
//MODBUS_WRITABLE_MASK array should be added as extern in the progam
//------------------------------------------------------------------------------
#include <stdint.h>
#include <stddef.h>
//------------------------------------------------------------------------------
#include "modbus_config.h"
#include "modbus.h"
//------------------------------------------------------------------------------
uint16_t MODBUS_HR[MBHR_SPACE_SIZE];
uint8_t MODBUS_WRITABLE_MASK[MBHR_SPACE_SIZE/sizeof(uint8_t)+(MBHR_SPACE_SIZE%sizeof(uint8_t))?1:0];
uint16_t* MyMBAddr=NULL; // main program may or may not initialise it
register_cb isregwrtbl_cb = NULL; // main program may or may not initialise it
register_cb regwr_cb = NULL; // main program may or may not initialise it
//------------------------------------------------------------------------------
extern uint8_t modbus_crc16H[];
//------------------------------------------------------------------------------
extern uint8_t modbus_crc16L[];
//------------------------------------------------------------------------------
uint16_t calc_crc(uint8_t *arr, uint8_t length) 
{
  uint8_t ind;
  uint8_t i;
  uint8_t cksumHigh = 0xFF;
  uint8_t cksumLow = 0xFF;
  if(length > 0) 
  {
    for(i=0; i<length; i++) 
    {
      ind = cksumHigh ^ arr[i];
      cksumHigh = cksumLow ^ modbus_crc16H[ind];
      cksumLow = modbus_crc16L[ind];
    }
  }
  return cksumLow |(cksumHigh << 8);
}
//------------------------------------------------------------------------------
uint8_t process_net_packet(ComMessage* inPack, ComMessage* outPack)
{
  if(MyMBAddr != NULL)
  {
    if(inPack->data[0] != *MyMBAddr && inPack->data[0] != MB_BROADCAST_ADDR)
      return MODBUS_PACKET_WRONG_ADDR;
  }
  else if(inPack->data[0] != MB_BROADCAST_ADDR)
    return MODBUS_PACKET_WRONG_ADDR;
  uint16_t tmpCRC = calc_crc(inPack->data, inPack->length - 2);
  if(tmpCRC != *(uint16_t*)&inPack->data[inPack->length - 2])
     return MODBUS_PACKET_WRONG_CRC;
  int res = process_modbus(inPack, outPack);
  if(res == MODBUS_PACKET_VALID_AND_PROCESSED)
  {
    tmpCRC = calc_crc(outPack->data, outPack->length);
    *(uint16_t*)&outPack->data[outPack->length] = tmpCRC;
    outPack->length += 2;
  }
  return res;
}
//------------------------------------------------------------------------------
//=== Анализ Modbus-команды ===//
int process_modbus(ComMessage* inPack, ComMessage* outPack)
{
  int res = MODBUS_PACKET_VALID_AND_PROCESSED;
  outPack->data[0] = inPack->data[0];
  outPack->data[1] = inPack->data[1];
  switch(inPack->data[1])
  {		// Байт команды.
  case 3:		// <03> holding registers read.
  case 4:		// <04> input registers read.
    res = CmdModbus_03_04(inPack, outPack);
    break;
  case 6:		// <06> single holding register write.
    res = CmdModbus_06(inPack, outPack);
    break;
  case 8:   // <08> loopback
    res = CmdModbus_08(inPack, outPack);
    break;
  case 16:		// <16> multiple holding registers write.
    res = CmdModbus_16(inPack, outPack);
    break;
  default:
    break;
  }
  return res;
}
//------------------------------------------------------------------------------
//=== <Modbus_03_04> holding/input registers read ===//
int CmdModbus_03_04(ComMessage* inPack, ComMessage* outPack)
{
  uint16_t Len, addr;
  Len = 2*(inPack->data[5]+((uint16_t)inPack->data[4] << 8));		// bytes to read
  if(Len >= TXRX_BUFFER_SIZE - 3) 
    Len = TXRX_BUFFER_SIZE-4;		// preventing segfault
  addr=((uint16_t)inPack->data[2] << 8) + inPack->data[3];		// first register to read
  if((addr + Len/2) > MBHR_SPACE_SIZE)
    return MODBUS_REGISTER_NUMBER_INVALID;
  outPack->data[2] = Len;		// number of bytes.
  outPack->length = 3 + Len;		// reply length
  //
  for(int i = 0; i < Len; i += 2)
  {		// filling data
    uint16_t val = MODBUS_HR[addr];
    *(uint16_t*)&outPack->data[3+i] = SWAP16(val);		// Big endian here
    addr++;
  }
  return MODBUS_PACKET_VALID_AND_PROCESSED;
}
//------------------------------------------------------------------------------
//=== <Modbus_06> single holding register write ===//
int CmdModbus_06(ComMessage* inPack, ComMessage* outPack)
{
  uint16_t Len, addr;
  uint8_t res = 0;
  outPack->length = 6;		// reply length
  addr=((uint16_t)inPack->data[2] << 8) + inPack->data[3];		// address to write
  if(addr > MBHR_SPACE_SIZE-1) 
    return MODBUS_REGISTER_NUMBER_INVALID;		// preventing segfault  
  int wrtbl=1;
  if(isregwrtbl_cb)
    wrtbl = isregwrtbl_cb(addr);
  if(!wrtbl)
    return MODBUS_REGISTER_WRITE_PROTECTED;
  uint16_t val = ((uint16_t)inPack->data[4] << 8) + inPack->data[5];		// value to write
  MODBUS_HR[addr]= val;
  if(regwr_cb)
    res = regwr_cb(addr);
  if(res)//if callback failed
  {

  }
  for(int i = 1; i < 6; i++) 
    outPack->data[i] = inPack->data[i];		// copy some bytes to reply
  return MODBUS_PACKET_VALID_AND_PROCESSED;
}
//------------------------------------------------------------------------------
//=== <Modbus_08> loopback ===//
int CmdModbus_08(ComMessage* inPack, ComMessage* outPack)
{
  outPack->length = inPack->length-2;
  for(int i = 0; i < outPack->length; i++)
  {
    outPack->data[i] = inPack->data[i];
  }
  return MODBUS_PACKET_VALID_AND_PROCESSED;
}
//------------------------------------------------------------------------------
//=== <Modbus_16> multiple holding registers write ===//
int CmdModbus_16(ComMessage* inPack, ComMessage* outPack)
{
  uint16_t addr;
  int res = 0;
  outPack->length = 6;		// reply length
  addr=((uint16_t)inPack->data[2] << 8) + inPack->data[3];		// first register to copy
  if(addr > MBHR_SPACE_SIZE-1) 
    return MODBUS_REGISTER_NUMBER_INVALID;		// preventing segfault
  uint16_t cnt = inPack->data[4];		// registers number
  if(addr + cnt > MBHR_SPACE_SIZE) 
      return MODBUS_REGISTER_NUMBER_INVALID;    // preventing segfault
  for(int i = 0; i < cnt; i++)
  {		// filling the date.
    int wrtbl=1;
    if(isregwrtbl_cb)
      wrtbl = isregwrtbl_cb(addr);
    if(!wrtbl)
      return MODBUS_REGISTER_WRITE_PROTECTED;
    MODBUS_HR[addr]=((uint16_t)inPack->data[5+2*i]<<8) + inPack->data[6+2*i];		// and another register value
    if(regwr_cb)
      res = regwr_cb(addr);
    if(res)//if callback failed
      return MODBUS_REGISTER_WRITE_CALLBACK_FAILED;
    addr++;
  }
  for(int i = 1; i < 6; i++) 
    outPack->data[i] = inPack->data[i];   // copy to reply
  return MODBUS_PACKET_VALID_AND_PROCESSED;
}
//------------------------------------------------------------------------------

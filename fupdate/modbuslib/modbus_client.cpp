#include "modbus_client.h"
//----------------------------------------------------------------------------------------------------------------------
extern uint8_t modbus_crc16H[];
extern uint8_t modbus_crc16L[];
//----------------------------------------------------------------------------------------------------------------------
uint16_t ModbusClient::calc_crc(uint8_t *arr, uint8_t length) 
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
//----------------------------------------------------------------------------------------------------------------------
std::vector<uint8_t> ModbusClient::build_write_reg_06(uint8_t addr, uint16_t reg,uint16_t val)
{
  std::vector<uint8_t> data;
  data.push_back(addr);
  data.push_back(0x06);
  data.push_back((reg & 0xff00) >> 8);
  data.push_back(reg & 0x00ff);
  data.push_back((val & 0xff00) >> 8);
  data.push_back(val & 0x00ff);
  uint16_t crc = calc_crc(data.data(), data.size());
  data.push_back(((uint8_t*)&crc)[0]);
  data.push_back(((uint8_t*)&crc)[1]);
  return data;
}
//----------------------------------------------------------------------------------------------------------------------
std::vector<uint8_t> ModbusClient::build_loop_08(uint8_t addr)
{
  std::vector<uint8_t> data;
  data.push_back(addr);
  data.push_back(0x08);
  uint16_t crc = calc_crc(data.data(), data.size());
  data.push_back(((uint8_t*)&crc)[0]);
  data.push_back(((uint8_t*)&crc)[1]);
  return data;
}
//----------------------------------------------------------------------------------------------------------------------
std::vector<uint8_t> ModbusClient::build_read_03(uint8_t addr, uint16_t reg,uint16_t cnt)
{
  std::vector<uint8_t> data;
  data.push_back(addr);
  data.push_back(0x03);
  data.push_back((reg & 0xff00) >> 8);
  data.push_back(reg & 0x00ff);
  data.push_back((cnt & 0xff00) >> 8);
  data.push_back(cnt & 0x00ff);
  uint16_t crc = calc_crc(data.data(), data.size());
  data.push_back(((uint8_t*)&crc)[0]);
  data.push_back(((uint8_t*)&crc)[1]);
  return data;
}
//----------------------------------------------------------------------------------------------------------------------
std::vector<uint8_t> ModbusClient::build_write_multreg_16(uint8_t addr, uint16_t startreg,std::vector<uint16_t> vals)
{
  std::vector<uint8_t> data;
  data.push_back(addr);
  data.push_back(0x10);
  data.push_back((startreg & 0xff00) >> 8);
  data.push_back(startreg & 0x00ff);
  data.push_back(vals.size());
  for(auto val : vals)
  {
    data.push_back((val & 0xff00) >> 8);
    data.push_back(val & 0x00ff);
  }
  
  uint16_t crc = calc_crc(data.data(), data.size());
  data.push_back(((uint8_t*)&crc)[0]);
  data.push_back(((uint8_t*)&crc)[1]);
  return data;
}
//----------------------------------------------------------------------------------------------------------------------

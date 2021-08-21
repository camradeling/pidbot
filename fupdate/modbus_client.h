#ifndef MODBUS_CLIENT_H
#define MODBUS_CLIENT_H
//----------------------------------------------------------------------------------------------------------------------
#include <unistd.h>
#include <sys/time.h>
#include "chanlib_export.h"
//----------------------------------------------------------------------------------------------------------------------
#define MB_BROADCAST_ADDR                              0xff
//----------------------------------------------------------------------------------------------------------------------
class ModbusClient
{
public:
	ModbusClient(){}
	~ModbusClient(){}
	vector<uint8_t> build_loop_08(uint8_t addr);
	vector<uint8_t> build_read_03(uint8_t addr, uint16_t reg,uint16_t cnt);
	vector<uint8_t> build_write_reg_06(uint8_t addr, uint16_t reg,uint16_t val);
	vector<uint8_t> build_write_multreg_16(uint8_t addr, uint16_t startreg,vector<uint16_t> vals);
	uint16_t calc_crc(uint8_t *arr, uint8_t length);
};
//----------------------------------------------------------------------------------------------------------------------
#endif /*MODBUS_CLIENT_H*/
#ifndef MODBUS_H
#define MODBUS_H
//------------------------------------------------------------------------------
#include <unistd.h>
#include <sys/time.h>
#include "uart.h"
//------------------------------------------------------------------------------
#define SWAP16(x) (((uint16_t)x << 8)|((uint16_t)x >> 8))
//------------------------------------------------------------------------------
#define MBHR_DISCRETE_INPUTS_LOW                0
#define MBHR_DISCRETE_INPUTS_HIGH               1
#define MBHR_DISCRETE_OUTPUTS_LOW               2
#define MBHR_DISCRETE_OUTPUTS_HIGH              3
#define MBHR_ADS1115_RAW_DIFVAL1                4
#define MBHR_ADS1115_RAW_REF1                   5
#define MBHR_ADS1115_RAW_VAL1_1                 6
#define MBHR_ADS1115_RAW_VAL1_2                 7
#define MBHR_ADS1115_RAW_DIFVAL2                8
#define MBHR_ADS1115_RAW_REF2                   9
#define MBHR_ADS1115_RAW_VAL2_1                 10
#define MBHR_ADS1115_RAW_VAL2_2                 11
#define MBHR_MY_MBADDR                          12
#define MBHR_BAUDRATE_PRESCALER                 13
#define MBHR_ADS1115_CONFIG                     14
#define MBHR_COMMAND_REG                        15
#define MBHR_FLASH_ADDRESS_LOW                  16
#define MBHR_FLASH_ADDRESS_HIGH                 17
#define MBHR_FLASH_BUFFER                       18
#define MBHR_FLASH_BUFFER_END                   145
#define MBHR_TEST_VALUE                         146
#define FLASH_BACKUP_FIRST_ADDR                 MBHR_MY_MBADDR
#define FLASH_BACKUP_LAST_ADDR                  MBHR_ADS1115_CONFIG
#define FLASH_BACKUP_PARAMS_LEN                 (FLASH_BACKUP_LAST_ADDR - FLASH_BACKUP_FIRST_ADDR + 1)
#define MODBUS_HR_SPACE_LAST_ADDR               MBHR_TEST_VALUE
#define MODBUS_HR_SPACE_SIZE                    MODBUS_HR_SPACE_LAST_ADDR+1
//------------------------------------------------------------------------------
//return codes
#define MODBUS_PACKET_VALID_AND_PROCESSED              0
#define MODBUS_PACKET_WRONG_ADDR                       1
#define MODBUS_PACKET_WRONG_CRC                        2
//------------------------------------------------------------------------------
#define MB_BROADCAST_ADDR                              0xff
//------------------------------------------------------------------------------
class ModbusClient
{
public:
	ModbusClient(){}
	~ModbusClient(){}
	void send_loop_08(uint8_t addr);
	void send_writereg_06(uint8_t addr, uint16_t reg,uint16_t val);
	COMPort* com=nullptr;
	uint16_t calc_crc(uint8_t *arr, uint8_t length);
};
//------------------------------------------------------------------------------
/*uint8_t process_net_packet(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
uint16_t calc_crc(uint8_t *arr, uint8_t length);
//------------------------------------------------------------------------------
void analise_modbus(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
void CmdModbus_03_04(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
void CmdModbus_06(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
void CmdModbus_08(ComMessage* inPack, ComMessage* outPack);
//------------------------------------------------------------------------------
void CmdModbus_16(ComMessage* inPack, ComMessage* outPack);*/
//------------------------------------------------------------------------------
#endif /*MODBUS_H*/
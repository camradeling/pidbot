#include <cstring>
#include "modbus.h"
#include "uart.h"
//----------------------------------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	fprintf(stderr, "arg = %s\n",argv[1]);
	int on=0;
	if(argc < 2)
		return -1;
	if(argv == nullptr)
		return -1;
	uint16_t val = strtol(argv[1], NULL, 16);
	ModbusClient* client = new ModbusClient;
	client->com = new COMPort;
	client->com->alias = "/dev/ttyUSB0";
	client->com->init();
	client->send_read_03(MB_BROADCAST_ADDR,0,2);
	fprintf(stderr, "REG 00: 0x%04X\n",*(uint16_t*)&client->rbuf[3]);
	fprintf(stderr, "REG 01: 0x%04X\n",*(uint16_t*)&client->rbuf[5]);
	return 0;
}
//----------------------------------------------------------------------------------------------------------------------
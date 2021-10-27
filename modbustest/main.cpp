#include <cstring>
#include <map>
#include "modbus.h"
#include "uart.h"
//----------------------------------------------------------------------------------------------------------------------
#define	JUMP 0
#define WAIT 1
#define RUN  2
//----------------------------------------------------------------------------------------------------------------------
std::map<int,string> StateStrMap = 
{
	std::pair<int,string>(JUMP,"Jump"),
	std::pair<int,string>(WAIT,"Wait"),
	std::pair<int,string>(RUN,"Run")
};
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
	client->com->alias = "/dev/ttyACM0";
	client->com->init();
	client->send_read_03(MB_BROADCAST_ADDR,0,79);
	uint16_t reg76=0;
	for(int i = 0; i < 79; i++)
	{
		uint16_t reg = *(uint16_t*)&client->rbuf[3+i*2];
		if(i < 10)
			fprintf(stderr, "REG %02d: 0x%04X\n",i,SWAP16(reg));
		if(i==76)
			reg76=SWAP16(reg);
	}
	string str="Unknown";
	std::map<int,string>::iterator iter = StateStrMap.find(reg76);
	if(iter != StateStrMap.end())
		str = iter->second;
	fprintf(stderr, "Firmware state is %d(%s)\n",reg76,str.c_str());
	return 0;
}
//----------------------------------------------------------------------------------------------------------------------
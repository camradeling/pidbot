#ifndef FUPDATER_H
#define FUPDATER_H
//----------------------------------------------------------------------------------------------------------------------
#include "chanlib_export.h"
#include "programthread.h"
#include "modbus_client.h"
#include "modbus_config.h"
//----------------------------------------------------------------------------------------------------------------------
#define BLOCK_SEND_TIMEOUT_MS 		500
#define FIRMWARE_CHECK_TIMEOUT_MS 	1500
//----------------------------------------------------------------------------------------------------------------------
enum WorkState
{
	INITIAL_STATE = 0,
	CHECK_BL_STATUS_SENT,
	SWITCH_TO_BL_SENT,
	FIRMWARE_BLOCK_SENT,
	FIRMWARE_WRITECRC_SENT,
	FIRMWARE_START_SENT,
	CHECK_STATUS_SENT,
	ERROR
};
//----------------------------------------------------------------------------------------------------------------------
typedef struct _Session
{
	weak_ptr<BasicChannel> ch;
	uint32_t fd = 0;
	uint8_t	 deviceOnline = 0;
	timespec_t confirmStamp={0,0};
	timespec_t kASpan = {0,0};
	timespec_t kAStamp = {0,0};
	timespec_t kAStampR = {0,0};
	bool zipflag=0;
	uint32_t InSeq=0;
	uint32_t OutSeq=0;
	string chanaddr="";
}Session;
//----------------------------------------------------------------------------------------------------------------------
class FUpdater : public ProgramThread
{
public:
	FUpdater(mxml_node_t* cnf=nullptr):config(cnf){}
	virtual ~FUpdater(){}
	virtual void init_module();
	virtual void thread_job();
	void process_channel(weak_ptr<BasicChannel> chan);
	vector<uint8_t> get_packet();
	void process_packet(uint8_t* data, int len);
	int send_one_more_block();
	int send_switch_to_bootloader();
	int send_sysreset();
	vector<uint8_t> InStream;
	shared_ptr<ModbusClient> MBCL;
	shared_ptr<ChanPool> CHPL;
	vector<Session> sessionsActive;
	Session* currentSession=nullptr;
	mxml_node_t* config=nullptr;
	std::vector<uint8_t>* firmware=nullptr;
	uint32_t firmwareNextAddr=0;
	timespec_t packSentStamp={0,0};
	WorkState state = INITIAL_STATE;
	WorkState lastState = INITIAL_STATE;
	uint16_t BlStatus=FIRMWARE_RUNNING;
};
//----------------------------------------------------------------------------------------------------------------------
#endif/*FUPDATER_H*/
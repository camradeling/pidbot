#include "fupdater.h"
//----------------------------------------------------------------------------------------------------------------------
void FUpdater::init_module()
{
	CHPL = shared_ptr<ChanPool>(new ChanPool);
    CHPL->chp = CHPL;
	MBCL = shared_ptr<ModbusClient>(new ModbusClient);
	CHPL->init(config);
	if(CHPL->allChan.size() > 1)
	{
		fprintf(stderr, "multiple channels in config, closing...\n");
		exit(-1);
	}
	for(int i = 0; i < CHPL->allChan.size(); i++)
    {
		weak_ptr<BasicChannel> chan = CHPL->allChan.at(i);
		shared_ptr<BasicChannel> schan = chan.lock();
		add_pollable_handler(schan->inCmdQueue.fd(), EPOLLIN, &FUpdater::process_channel, this, chan);
        add_pollable_handler(schan->inQueue.fd(), EPOLLIN, &FUpdater::process_channel, this, chan);
    }
    ProgramThread::init_module();
}
//----------------------------------------------------------------------------------------------------------------------
void FUpdater::process_channel(weak_ptr<BasicChannel> chan)
{
	shared_ptr<BasicChannel> schan = chan.lock();
	if(!schan)
		return;
	std::unique_ptr<MessageBuffer> packet;
	while((packet = schan->inCmdQueue.pop()) && !stop)
	{
		enum MessageType packetType = packet->Type();
		if(1)
		{
			if (packetType == CHAN_OPEN_PACKET)
			{
			    Session ses;
				ses.fd = packet->getfd();
				fprintf(stderr, "assigning session fd %d\n",ses.fd);
				ses.ch = schan;
				ses.deviceOnline = 0;
				if(packet->getChanAddr() != "")
					ses.chanaddr = packet->getChanAddr();
				//здесь надо послать инит мессадж
				sessionsActive.push_back(ses);
				fprintf(stderr, "sessions count %lld\n",sessionsActive.size());
			}
			else if (packetType == CHAN_CLOSE_PACKET)
			{
				for (int j = 0; j < sessionsActive.size(); j++)
				{
					if (sessionsActive.at(j).fd == packet->getfd())
					{
						sessionsActive.erase(sessionsActive.begin() + j);
						break;
					}
				}
				fprintf(stderr, "command channel closed unexpectedly, exiting...\n");
				exit(-1);
			}
		}
	}
	currentSession = nullptr;
    while((packet = schan->inQueue.pop()) && !stop)
	{
		enum MessageType packetType = packet->Type();
    	currentSession = nullptr;
        if (packetType == CHAN_DATA_PACKET)
        {//для других зарезервируем это же значение
        	for (int j = 0; j < sessionsActive.size(); j++)
            {
                if (sessionsActive.at(j).fd == packet->getfd())
                {
                    currentSession = &sessionsActive[j];
                    currentSession->InSeq = packet->seqnum;
                    if(currentSession->InSeq == currentSession->OutSeq)
                    {
                    	process_packet((uint8_t*)packet->Data(), packet->Length());//if ok, state will change to _CONFIRMED
                    }
                    else
                    	state = ERROR;
                    break;
                }
            }
        }
        else
        	fprintf(stderr, "Unknown data packet type %d\n", packetType);
	}
}
//----------------------------------------------------------------------------------------------------------------------
void FUpdater::process_packet(uint8_t* data, int len)
{
	vector<uint16_t> bytes;
	vector<uint8_t> packdata;
	timespec_t curStamp = {0,0};
	uint16_t crc=0;
	clock_gettime(CLOCK_MONOTONIC, &curStamp);
	switch(state)
	{
	case INITIAL_STATE:
		fprintf(stderr, "unexpected packet, exiting...\n");
		exit(-1);
		break;
	case FIRMWARE_BLOCK_SENT:
	{
		packdata = MBCL->build_read_03(MB_BROADCAST_ADDR, MBHR_COMMAND_STATUS,1);
		if(packdata.size() <= 0)
		{
			fprintf(stderr, "error constructing modbus packet, exiting...\n");
			state = ERROR;
		}
		std::unique_ptr<MessageBuffer> buf(new MessageBuffer(currentSession->fd, packdata.size(), CHAN_DATA_PACKET));
		memcpy(buf->Data(),packdata.data(),packdata.size());
		shared_ptr<BasicChannel> schan = currentSession->ch.lock();
		if(!schan)
			state = ERROR;
		schan->send_message_buffer(&schan->outQueue, std::move(buf), true);
		packSentStamp = curStamp;
		lastState = state;
		state = CHECK_STATUS_SENT;
		break;
	}
	case FIRMWARE_WRITECRC_SENT:
	{
		packdata = MBCL->build_write_reg_06(MB_BROADCAST_ADDR, MBHR_REG_COMMAND,CMD_START_FIRMWARE);
		if(packdata.size() <= 0)
		{
			fprintf(stderr, "error constructing modbus packet, exiting...\n");
			state = ERROR;
		}
		std::unique_ptr<MessageBuffer> buf(new MessageBuffer(currentSession->fd, packdata.size(), CHAN_DATA_PACKET));
		memcpy(buf->Data(),packdata.data(),packdata.size());
		shared_ptr<BasicChannel> schan = currentSession->ch.lock();
		if(!schan)
			state = ERROR;
		schan->send_message_buffer(&schan->outQueue, std::move(buf), true);
		packSentStamp = curStamp;
		lastState = state;
		state = FIRMWARE_START_SENT;
		break;
	}
	case FIRMWARE_START_SENT:
	{
		fprintf(stderr, "firmware start command confirmed, checking status...\n");
		packdata = MBCL->build_read_03(MB_BROADCAST_ADDR, MBHR_COMMAND_STATUS,1);
		if(packdata.size() <= 0)
		{
			fprintf(stderr, "error constructing modbus packet, exiting...\n");
			state = ERROR;
		}
		std::unique_ptr<MessageBuffer> buf(new MessageBuffer(currentSession->fd, packdata.size(), CHAN_DATA_PACKET));
		memcpy(buf->Data(),packdata.data(),packdata.size());
		shared_ptr<BasicChannel> schan = currentSession->ch.lock();
		if(!schan)
			state = ERROR;
		schan->send_message_buffer(&schan->outQueue, std::move(buf), true);
		packSentStamp = curStamp;
		lastState = state;
		state = CHECK_STATUS_SENT;
		break;
		break;
	}
	case CHECK_STATUS_SENT:
	{
		switch(lastState)
		{
		case FIRMWARE_BLOCK_SENT:
		{
			if(len*2 < MODBUS_03_DATASTART_IND+4)
			{
				state = ERROR;
				break;
			}
			uint16_t status = (((uint16_t)data[MODBUS_03_DATASTART_IND*2] << 8) & 0xff00) | (((uint16_t)data[MODBUS_03_DATASTART_IND*2+1] >> 8) & 0x00ff);
			if(status != COMMAND_STATUS_OK)
			{
				state = ERROR;
				break;
			}
			if(firmwareNextAddr >= firmware->size())
			{
				bytes.push_back(firmware->size());
				crc = MBCL->calc_crc(firmware->data(),firmware->size());
				bytes.push_back(crc);
				packdata = MBCL->build_write_multreg_16(MB_BROADCAST_ADDR, MBHR_FIRMWARE_FULL_LEN, bytes);
				if(packdata.size() <= 0)
				{
					fprintf(stderr, "error constructing modbus packet, exiting...\n");
					state = ERROR;
				}
				std::unique_ptr<MessageBuffer> buf(new MessageBuffer(currentSession->fd, packdata.size(), CHAN_DATA_PACKET));
				memcpy(buf->Data(),packdata.data(),packdata.size());
				shared_ptr<BasicChannel> schan = currentSession->ch.lock();
				if(!schan)
					state = ERROR;
				schan->send_message_buffer(&schan->outQueue, std::move(buf), true);
				packSentStamp = curStamp;
				lastState = state;
				state = FIRMWARE_WRITECRC_SENT;
			}
			else if(!send_one_more_block())
				state = ERROR;
			break;
		}
		case FIRMWARE_START_SENT:
		{
			fprintf(stderr, "firmware check status after start command shouldnt work, finishing...\n");
			exit(-1);
			break;
		}
		default:
			fprintf(stderr, "unknown state, exiting...\n");
			exit(-1);	
		}  
		break;
	}
	default:
		fprintf(stderr, "unknown state, exiting...\n");
		exit(-1);
		break;	
	}
}
//----------------------------------------------------------------------------------------------------------------------
int FUpdater::send_one_more_block()
{
	vector<uint16_t> data;
	vector<uint8_t> packdata;
	int blocklen = 0;
	uint16_t crc=0;
	timespec_t curStamp = {0,0};
	clock_gettime(CLOCK_MONOTONIC, &curStamp);
	//writing firmware relative addr
	data.push_back(firmwareNextAddr);
	//filling firmware block
	blocklen = (firmwareNextAddr + MAX_FIRMWARE_BLOCK_SIZE < firmware->size())?MAX_FIRMWARE_BLOCK_SIZE:firmware->size()-firmwareNextAddr;
	for(int i = 0; i < blocklen; i+=2)
	{
		if(blocklen-1 == i)
			data.push_back((uint16_t)(*(uint8_t*)&firmware->data()[firmwareNextAddr + i*2]) & 0x00ff);
		else
			data.push_back(*(uint16_t*)&firmware->data()[firmwareNextAddr + i*2]);
	}
	//write firmware block addr
	data.push_back(blocklen);
	//write block crc
	crc=MBCL->calc_crc(&firmware->data()[firmwareNextAddr],blocklen);
	data.push_back(crc);
	data.push_back(CMD_WRITE_FIRMWARE_BLOCK);
	packdata = MBCL->build_write_multreg_16(MB_BROADCAST_ADDR, MBHR_WRITE_FLASH_ADDR, data);
	if(packdata.size() <= 0)
	{
		fprintf(stderr, "error constructing modbus packet, exiting...\n");
		return -1;
	}
	std::unique_ptr<MessageBuffer> buf(new MessageBuffer(currentSession->fd, packdata.size(), CHAN_DATA_PACKET));
	memcpy(buf->Data(),packdata.data(),packdata.size());
	shared_ptr<BasicChannel> schan = currentSession->ch.lock();
	if(!schan)
		return -1;
	schan->send_message_buffer(&schan->outQueue, std::move(buf), true);
	firmwareNextAddr += blocklen;
	packSentStamp = curStamp;
	lastState = state;
	state = FIRMWARE_BLOCK_SENT;
	return 0;
}
//----------------------------------------------------------------------------------------------------------------------
void FUpdater::thread_job()
{
	uint64_t diff=0;
	timespec_t curStamp = {0,0};
	if(currentSession == nullptr)
		return;
	switch(state)
	{
	case INITIAL_STATE:
		if(!send_one_more_block())
			state = ERROR;
		break;
	case FIRMWARE_BLOCK_SENT:
	case FIRMWARE_WRITECRC_SENT:
	case FIRMWARE_START_SENT:
	case CHECK_STATUS_SENT:
		
		clock_gettime(CLOCK_MONOTONIC, &curStamp);
		diff = ((uint64_t)curStamp.tv_sec*1000 + (uint64_t)curStamp.tv_nsec/DECMILLION) - \
                    ((uint64_t)packSentStamp.tv_sec*1000 + (uint64_t)packSentStamp.tv_nsec/DECMILLION);
        if(diff > BLOCK_SEND_TIMEOUT_MS)
        {
        	fprintf(stderr, "timeout error, exiting...\n");
			exit(-1);
        }            
		break;
	case ERROR:		
		fprintf(stderr, "unknown error, exiting...\n");
		exit(-1);
		break;
	default:
		fprintf(stderr, "unknown state, exiting...\n");
		exit(-1);
		break;
	}
}
//----------------------------------------------------------------------------------------------------------------------

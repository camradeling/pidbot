#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED
//----------------------------------------------------------------------------------------------------------------------
#include <string>
#include <termios.h>
#include <sys/epoll.h>
//----------------------------------------------------------------------------------------------------------------------
typedef struct timeval timeval_t;
//----------------------------------------------------------------------------------------------------------------------
#define COM_EDN_PARITY_NONE		    0
#define COM_EDN_PARITY_EVEN			1
#define COM_EDN_PARITY_ODD			2
///----------------------------------------------------------------------------------------------------------------------
using namespace std;
//----------------------------------------------------------------------------------------------------------------------
typedef struct _UARTConfig_t
{
    uint32_t		SPEED=115200;
    uint16_t		WORDLEN=8;
    uint16_t		EDN=COM_EDN_PARITY_NONE;
    uint16_t		STOPBITS=1;
    uint16_t		HARDFLOW=0;
}UARTConfig_t;
//----------------------------------------------------------------------------------------------------------------------
class ChanPool;
//----------------------------------------------------------------------------------------------------------------------
class COMPort
{
public:
    COMPort(){}
    virtual ~COMPort(){tmt.tv_sec =1;tmt.tv_usec=0;}
    virtual int init();
    //virtual void thread_run();
    virtual void config_com();
    //virtual int send_packet(MessageBuffer *packet, enum io_state state, bool zipped=0);
    //virtual int recv_packet(std::unique_ptr<MessageBuffer> *packet, enum io_state state);
    virtual int com_write_chunk(int fd, uint8_t *buf, int nbytes);
    virtual int com_read_chunk(int fd, uint8_t *buf, int nbytes);
public:
    int                     clientfd;
    UARTConfig_t			tcfg;
    struct termios  		Config;
    uint32_t      			SymTO;
    string                  alias="";
    timeval_t               tmt;
};
//----------------------------------------------------------------------------------------------------------------------
#endif // UART_H_INCLUDED

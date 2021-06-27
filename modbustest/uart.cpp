#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
//----------------------------------------------------------------------------------------------------------------------
#include <sys/socket.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
//----------------------------------------------------------------------------------------------------------------------
#include "uart.h"
//----------------------------------------------------------------------------------------------------------------------
int COMPort::init()
{
    if(alias != "")
        clientfd = open (alias.c_str(), O_RDWR | O_NOCTTY  | O_EXCL | O_NDELAY);
    if(clientfd <= 0)
    {
        fprintf (stderr, "Error opening %s\n", alias.c_str());
        exit(-1);
    }
    tcgetattr(clientfd, &Config);
    config_com();
    Config.c_cc[VMIN] = 0;
    Config.c_cc[VTIME] = 0;
    tcsetattr(clientfd, TCSANOW, &Config);
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int COMPort::com_write_chunk(int fd, uint8_t *buf, int nbytes)
{
    timeval_t wt = tmt;
    fd_set rdset,wrset,exset; FD_ZERO(&rdset);FD_ZERO(&wrset);FD_ZERO(&exset);
    FD_SET(fd,&wrset);
    int res = select(fd+1,&rdset,&wrset,&exset, &wt);
    if(res < 0)
    {
        fprintf(stderr, "com select error\n");
    }
    else if(res == 0)
    {
        fprintf(stderr, "com send packet timeout\n");
    }
    else
    {
        if (FD_ISSET(fd, &wrset))
        {
            fprintf(stderr, "TX: ");
            for(int i = 0; i < nbytes; i++)
            {
                fprintf(stderr, "%02X ",buf[i]);
            }
            fprintf(stderr, "\n");
            res = write(fd, buf, nbytes);
        }
    }
    return res;
}
//----------------------------------------------------------------------------------------------------------------------
int COMPort::com_read_chunk(int fd, uint8_t *buf, int nbytes)
{
    timeval_t wt = tmt;
    fd_set rdset,wrset,exset; FD_ZERO(&rdset);FD_ZERO(&wrset);FD_ZERO(&exset);
    FD_SET(fd,&rdset);
    int res = select(fd+1,&rdset,&wrset,&exset, &wt);
    if(res < 0)
    {
        fprintf(stderr, "com select error\n");
    }
    else if(res == 0)
    {
        fprintf(stderr, "com receive packet timeout\n");
    }
    else
    {
        if (FD_ISSET(fd, &rdset))
        {
            res = read(fd, buf, nbytes);
            fprintf(stderr, "RX: ");
            for(int i = 0; i < res; i++)
            {
                fprintf(stderr, "%02X ",buf[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    return res;
}
//----------------------------------------------------------------------------------------------------------------------
void COMPort::config_com()
{
    fprintf(stderr, "Configuring com port\n");
    Config.c_cflag = 0;
    Config.c_iflag = 0;
    Config.c_lflag = 0;
    fprintf(stderr, "config speed = %d\n", Config.c_ispeed);
    if (tcfg.SPEED == 2400)
    {
        cfsetispeed (&(Config), B2400);
        cfsetospeed (&(Config), B2400);
        SymTO = (1000000 / 2400) * 50;
    }
    else if (tcfg.SPEED == 4800)
    {
        cfsetispeed (&(Config), B4800);
        cfsetospeed (&(Config), B4800);
        SymTO = (1000000 / 4800) * 50;
    }
    else if (tcfg.SPEED == 9600)
    {
        cfsetispeed (&(Config), B9600);
        cfsetospeed (&(Config), B9600);
        SymTO = (1000000 / 9600) * 50;
    }
    else if (tcfg.SPEED == 19200)
    {
        cfsetispeed (&(Config), B19200);
        cfsetospeed (&(Config), B19200);
        SymTO = (1000000 / 19200) * 50;
    }
    else if (tcfg.SPEED == 38400)
    {
        cfsetispeed (&(Config), B38400);
        cfsetospeed (&(Config), B38400);
        SymTO = (1000000 / 38400) * 50;
    }
    else if (tcfg.SPEED == 57600)
    {
        cfsetispeed (&(Config), B57600);
        cfsetospeed (&(Config), B57600);
        SymTO = (1000000 / 57600) * 50;
    }
    else if (tcfg.SPEED == 115200)
    {
        cfsetispeed (&(Config), B115200);
        cfsetospeed (&(Config), B115200);
        SymTO = (1000000 / 115200) * 50;
    }
    else if (tcfg.SPEED == 230400)
    {
        cfsetispeed (&(Config), B230400);
        cfsetospeed (&(Config), B230400);
        SymTO = (1000000 / 230400) * 50;
    }
    else if (tcfg.SPEED == 1000000)
    {
        cfsetispeed (&(Config), B1000000);
        cfsetospeed (&(Config), B1000000);
        SymTO = 50;
    }
    else
    {
        cfsetispeed (&(Config), B115200);
        cfsetospeed (&(Config), B115200);
        SymTO = (1000000 / 115200) * 50;
    }
    SymTO *= 5;//иначе не хватает, почему - не знаю. По всем расчетам должно было хватать
    if (tcfg.EDN == COM_EDN_PARITY_NONE)       // none
    {
        Config.c_cflag &= ~PARENB;
    }
    else if (tcfg.EDN == COM_EDN_PARITY_EVEN) // Even
    {
        Config.c_cflag |= PARENB;
        Config.c_cflag &= ~PARODD;
    }
    else if (tcfg.EDN == COM_EDN_PARITY_ODD) // odd
    {
        Config.c_cflag |= PARENB;
        Config.c_cflag |= PARODD;
    }
    else
    {
        Config.c_cflag &= ~PARENB;
    }
    if (tcfg.STOPBITS == 2)       // 2 stop
    {
        Config.c_cflag |= CSTOPB;
    }
    else // 1 stop
    {
        Config.c_cflag &= ~CSTOPB;
    }
    if (tcfg.HARDFLOW != 0)       // Hardware flow control
    {
        Config.c_cflag |= CRTSCTS;
    }
    else // No hardware flow control
    {
        Config.c_cflag &= ~CRTSCTS;
    }
    Config.c_cflag |= (CLOCAL | CREAD);
    Config.c_cflag &= ~CSIZE;
    if (tcfg.WORDLEN == 8)
        Config.c_cflag |= CS8;
    else if (tcfg.WORDLEN == 7)
        Config.c_cflag |= CS7;
    else if (tcfg.WORDLEN == 6)
        Config.c_cflag |= CS6;
    else if (tcfg.WORDLEN == 5)
        Config.c_cflag |= CS5;
    else
        Config.c_cflag |= CS8;
    Config.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | INPCK | ISTRIP | PARMRK | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | IUTF8 | IUCLC);
    Config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    Config.c_oflag &= ~(OPOST);
}
//----------------------------------------------------------------------------------------------------------------------

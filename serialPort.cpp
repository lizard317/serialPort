#include <cstdio>
#include <cstdlib>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

#include <string>
#include <iostream>

#include "serialPort.h"

using namespace std;

serialPort::serialPort()
{
    new (this)serialPort("");
}

serialPort::serialPort(const string &pn)
{
    new (this)serialPort(pn, 115200, 8, no, 1);
}

serialPort::serialPort(const string &pn, const int &br)
{
    new (this)serialPort(pn, br, 8, no, 1);
}

serialPort::serialPort(const string &pn, const int &br, const int &db, const PARITY &p, const int &sb):
	PortName(pn),BaudRate(br),DataBits(db),Parity(p),StopBits(sb)
{
    fd = -1;
    isOpen = false;
    isReading = false;
    isClosing = false;
}

serialPort::~serialPort()
{
    if(isOpen)
        Close();
}

void serialPort::Open()
{
    const char * pn = PortName.c_str();
    fd = open(pn, O_RDWR);// | O_NOCTTY | O_NDELAY
    if (fd == -1)
    {
        //Could not open the port.
        perror("open port failed");
        return;
    }
    else
        fcntl(fd, F_SETFL, 0);

    SetBaudRate(BaudRate);
    SetDataBits(DataBits);
    SetParity(Parity);
    SetStopBits(StopBits);

    struct termios options;
    //Get the current options for the port...
    tcgetattr(fd, &options);
    //Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);
    //关闭硬件流控
    options.c_cflag &= ~CRTSCTS;
    
	// Mask the character size bits 
	//options.c_cflag &= ~CSIZE;
	//输出模式,原始数据输出
	//options.c_oflag &= ~OPOST;
	//原始输入模式
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //
    //options.c_iflag &= ~(ICRNL | IXON);
    //read调用读到数据则立即返回
	options.c_cc[VMIN] = 0;
    //超时1s
	options.c_cc[VTIME] = 10;

    //set raw mode
    cfmakeraw(&options);

    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);

    isOpen = true;
    isReading = false;
}

void serialPort::Close()
{
    if(isOpen)
    {
        isClosing = true;
        close(fd);
        isOpen = false;
        isReading = false;
        isClosing = false;
    }
}

void serialPort::SetBaudRate(int br)
{
    BaudRate = br;
    struct termios options;
    //Get the current options for the port...
    tcgetattr(fd, &options);

    speed_t BR;
    switch(br)
    {
        case 115200:
            BR = B115200;
            break;
        case 57600:
            BR = B57600;
            break;
        case 0:
            BR = B0;
            break;
        default:
            return;
    }
    cfsetispeed(&options, BR);
    cfsetospeed(&options, BR);
    //Enable the receiver and set local mode...
    //options.c_cflag |= (CLOCAL | CREAD);
    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);
}

void serialPort::SetDataBits(int db)
{
    DataBits = db;
    struct termios options;
    //Get the current options for the port...
    tcgetattr(fd, &options);
    
    switch(db)
    {
        case 8:
            options.c_cflag |= CS8;    // Select 8 data bits 
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        default:
            return;
    }
    
    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);
}

void serialPort::SetParity(PARITY p)
{
    Parity = p;
    struct termios options;
    //Get the current options for the port...
    tcgetattr(fd, &options);

    switch(p)
    {
        case no:
        case space:
            options.c_cflag &= ~PARENB;
            break;
        case odd:
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
        case even:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        case unknow:
            return;
    }

    tcsetattr(fd, TCSANOW, &options);
}

void serialPort::SetStopBits(int sb)
{
    StopBits = sb;
    struct termios options;
    //Get the current options for the port...
    tcgetattr(fd, &options);

    switch(sb)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
		    break;
        case 2:
            options.c_cflag |= CSTOPB;
		    break;
        default:
            return;
    }

    tcsetattr(fd, TCSANOW, &options);
}

void serialPort::Write(const char * buff, int offset, int bytesToWrite)
{
	int n = 0;
	buff += offset;
	while(n != bytesToWrite)
		n += write(fd, buff + n, bytesToWrite - n);
}

void serialPort::Read(char * buff, int offset, int bytesToRead)
{
	int n = 0;
	buff += offset;
    isReading = true;
	while(n != bytesToRead)
	{
		n += read(fd, buff + n, bytesToRead - n);
		//cout << n << " " << buff << endl;
	}
	//DiscardInBuffer();
    isReading = false;
    buff[n] = '\0';
    //DiscardOutBuffer();
}

int serialPort::BytesToRead()
{
	int len = 0;//缓冲区数据长度
	ioctl(fd,FIONREAD,&len);
	return len;
}

void serialPort::DiscardInBuffer()
{
    tcflush(fd, TCIFLUSH);
}

void serialPort::DiscardOutBuffer()
{
    tcflush(fd, TCOFLUSH);
}

void serialPort::SendCmd(char * cmd, int count)
{
    while(isReading)
        usleep(10);
    DiscardOutBuffer();
    Write(cmd, 0, count);
}

void CRC16(vector<char> data, int cmd_length, char * CR16Hi, char * CR16Lo)
        {
            char CRC16Lo = 0xFF, CRC16Hi = 0xFF;      //CRC寄存器
            char CL = 0x01, CH = 0xA0;                //多项式码&HA001
            char SaveHi, SaveLo;

            for (int I = 0; I != cmd_length + 1; ++I)
            {
                CRC16Lo ^= data[I];
                for (int Flag = 0; Flag != 8; ++Flag)
                {
                    SaveHi = CRC16Hi;
                    SaveLo = CRC16Lo;
                    CRC16Hi >>= 1;
                    CRC16Lo >>= 1;
                    if ((SaveHi & 0x01) == 0x01)
                        CRC16Lo |= 0x80;
                    if ((SaveLo & 0x01) == 0x01)
                    {
                        CRC16Hi ^= CH;
                        CRC16Lo ^= CL;
                    }
                }
            }
            *CR16Hi = CRC16Hi;
            *CR16Lo = CRC16Lo;
        }





#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <vector>

using namespace std;

enum PARITY {no, even, odd, space, unknow = -1};
//typedef unsigned char byte;

class serialPort
{
    private:
        int fd;
        bool isOpen;
        bool isReading;
        bool isClosing;
        string PortName;
        int BaudRate;
        int DataBits;
        PARITY Parity;
        int StopBits;

    public:
        serialPort();
        serialPort(const string &pn);
        serialPort(const string &pn, const int &baudRate);
        serialPort(const string &portName, const int &baudRate, const int &dataBits, const PARITY &parity, const int &stopBits);
        ~serialPort();
        void Open();        //打开端口
        void Close();       //关闭端口
        bool IsOpen()           {return isOpen;}     //返回端口启用状态
        string GetPortName()    {return PortName;}
	int GetBaudRate()       {return BaudRate;}
	int GetDataBits()       {return DataBits;}
	PARITY GetParity()      {return Parity;}
	int GetStopBits()       {return StopBits;}
        void SetBaudRate(int baudRate);     //将端口速率设置为baudRate
        void SetDataBits(int dataBits);     //将端口数据位设置为dataBits
        void SetParity(PARITY parity);      //将端口校验设置为parity
        void SetStopBits(int stopBits);     //将端口停止位设置为stopBits
	void Write(const char * buff, int offset, int bytesToWrite);    //将buff中offset开始的bytesToWrite个字符写入端口
	void Read(char * buff, int offset, int bytesToRead);            //读取端口缓冲区中bytesToRead个字符，存入buff的offset位置
	int BytesToRead();          //获取输入缓冲区中的字符数量
        void DiscardInBuffer();     //清空输入缓冲区
        void DiscardOutBuffer();    //清空输出缓冲区
        void SendCmd(char * cmd, int count);    //发送一个字符数组的前count个
};

void CRC16(vector<char> data, int cmd_length, char * CR16Hi, char * CR16Lo);

#endif

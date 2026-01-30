#ifndef PTZ_TCPClientSocket_H
#define PTZ_TCPClientSocket_H

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <iterator>
#include <regex>
#include <QString>

class PTZ_TCPClientSocket
{
public:
    PTZ_TCPClientSocket();
    ~PTZ_TCPClientSocket();

private:
    //WSADATA wsaData;
    //WORD wVersionRequested;
    struct sockaddr_in m_srvAddr;//服务端地址
    int m_srvSocket;
    //SOCKADDR_IN m_srvAddr;	//服务器地址
    short m_nSrvPort;		//服务端端口
    char m_chSrvIP[30];		//服务端IP地址

public:
    bool InitSocket();	//初始化套接字
    bool SendData(unsigned char* pDataBuf, int nBufLen);
//    bool SendData(char* pDataBuf, int nBufLen);
    //bool RecvData(char* pDataBuf, int bBufLen);
    bool RecvData(std::string& protoCommad, double &hori_value, double &second_rotate_value,double &rotate_value, double &pitch_value);
    bool IsConnect(void);
    void SetServerInfo(char* chSrvIP, short nSrvPort);
    void Run(void);
    bool Connect(void);
    void sendByteDataToPTZ(QString strData);
    void recovery_camera_to_init_position(QString init_pitch_angle, QString init_rotate_angle);

    bool initlizePTZParam(QString init_pitch_angle, QString init_rotate_angle);
private:
    //bool Connect(void);
    void DoWork(void);

};

#endif // PTZ_TCPClientSocket_H

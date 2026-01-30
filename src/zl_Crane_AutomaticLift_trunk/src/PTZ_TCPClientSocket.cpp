#include "PTZ_TCPClientSocket.hpp"
#include <QWidget>

//接收控制器通讯协议数据
std::string tcp_sign = "0x68";
std::string croll_len = "0x04";
std::string rroll_addcode = "0x83";
std::string rrotate_addcode = "0x84";


PTZ_TCPClientSocket::PTZ_TCPClientSocket()
{
    m_srvSocket = -1;
}


PTZ_TCPClientSocket::~PTZ_TCPClientSocket()
{
    //if (m_srvSocket != NULL)
    {
        //closesocket(m_srvSocket);
        //m_srvSocket = NULL;
    }
    //WSACleanup(); //释放套接字资源
    if( m_srvSocket != -1)
    {
        close( m_srvSocket );
        m_srvSocket = -1;
    }
}

//设置服务器端口、IP地址信息
void PTZ_TCPClientSocket::SetServerInfo(char* chSrvIP, short nSrvPort)
{
    if (chSrvIP && strlen(chSrvIP) > 0)
    {
        strcpy(m_chSrvIP, chSrvIP);
    }
    m_nSrvPort = nSrvPort;
}
//初始化套接字
bool PTZ_TCPClientSocket::Connect(void)
{
    //创建服务器套接字
    m_srvSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    memset(&m_srvAddr, 0 ,sizeof(m_srvAddr));
    //socket函数如调用成功，返回一个新的SOCKET类型的套接字描述符；
    //调用失败，返回INVALID_SOCKET。
    if (-1 == m_srvSocket)
    {
        return false;
    }

    //int nMode = 1;
    //ioctlsocket(m_srvSocket, FIONBIO, (u_long*)&nMode);
    //设置服务器地址
    m_srvAddr.sin_family = AF_INET;
    m_srvAddr.sin_port = htons(m_nSrvPort);
    m_srvAddr.sin_addr.s_addr = inet_addr(m_chSrvIP);
    int nAddrSize = sizeof(m_srvAddr);

    //连接服务器
    if(-1 == connect(m_srvSocket, (struct sockaddr*)(&m_srvAddr), nAddrSize))
    {
        //closesocket(m_srvSocket);
        //m_srvSocket = NULL;
        close(m_srvSocket);
        return false;
    }
    return true;
}

//数据发送
bool PTZ_TCPClientSocket::SendData(unsigned char* pDataBuf, int nBufLen)
{
    if(m_srvSocket == -1)
    {
        return false;
    }

    int send_len = -1;
    for(int num=0; num<3; num++)
    {
       send_len = send(m_srvSocket, pDataBuf, nBufLen, 0);
       if(send_len <= 0)
       {
           close(m_srvSocket);
           Connect();
           continue;
       }
       return  true;
    }
    return true;
}
//字符串分割
std::vector<std::string> s_split(const std::string& in, const std::string& delim) {
    std::regex re{ delim };
    // 调用 std::vector::vector (InputIterator first, InputIterator last,const allocator_type& alloc = allocator_type())
    // 构造函数,完成字符串分割
    return std::vector<std::string> {
        std::sregex_token_iterator(in.begin(), in.end(), re, -1),
                std::sregex_token_iterator()
    };
}
//数据接收
bool PTZ_TCPClientSocket::RecvData(std::string& protoCommad, double &hori_value, double &second_rotate_value,double &rotate_value, double &pitch_value)
{
    unsigned char pDataBuf[24];
    char str[100];

    //设置一秒为超时时间
    struct timeval tv_out;
    tv_out.tv_sec = 0;
    tv_out.tv_usec = 1;
    setsockopt(m_srvSocket,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));

    int bBufLen;
    int revLen = recv(m_srvSocket, pDataBuf, sizeof(pDataBuf), 0);
    if (revLen <= 0)
    {
        std::cout << "没有接收到信息！" << std::endl;
        protoCommad="";
    }
    else
    {
        //需要找到标识符
        int firstInd = -1;
        for(int ind=0; ind<sizeof(pDataBuf); ind++)
        {
            if(pDataBuf[ind] == 0x6A)
            {
                if(pDataBuf[ind + 1] == 0x04)
                {
                    firstInd = ind;
                    break;
                }
                if(pDataBuf[ind + 1] == 0x05)
                {
                    firstInd = ind;
                    break;
                }
                if(pDataBuf[ind + 1] == 0x10)
                {
                    firstInd = ind;
                    break;
                }
            }
            else if(pDataBuf[ind] == 0x6B)
            {
                if(pDataBuf[ind + 1] == 0x012)
                {
                    firstInd = ind;
                    break;
                }
            }
        }
        //处理找不到标识符情况
        if(firstInd == -1)
        {
            std::cout << "接收到的信息不完整！" << std::endl;
            protoCommad="";
            return false;
        }

        //从协议中提取信息
        if(pDataBuf[firstInd+1] == 0x04)
        {
            //云台横滚和俯仰角度解析
            int sum=0;
            unsigned char *pBuf=&pDataBuf[firstInd+4];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            double AngleVal=sum*0.0001;
            if(pDataBuf[firstInd + 3])
            {
                AngleVal=-AngleVal;
            }
            std::string AngleType;

            if(pDataBuf[firstInd + 2] == 0x81)
            {
                AngleType="云台收到第二横滚控制角度为";
            }else{
                AngleType="云台收到第二旋转控制角度为";
            }

            protoCommad=AngleType;
            sprintf(str," : %.4lf°",AngleVal);
            protoCommad.append( str);

        }
        if(pDataBuf[firstInd+1] == 0x05)
        {
            int sum=0;
            unsigned char *pBuf=&pDataBuf[firstInd+4];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            double AngleVal=sum*0.0001;
            if(pDataBuf[firstInd + 3])
            {
                AngleVal=-AngleVal;
            }
            std::string AngleType;

            if(pDataBuf[firstInd + 2] == 0x81)
            {
                AngleType="云台到达第二横滚角度为";
            }else{
                AngleType="云台达到第二旋转角度为";
            }

            protoCommad=AngleType;
            sprintf(str," : %.4lf°",AngleVal);
            protoCommad.append( str);
        }
        if(pDataBuf[firstInd+1] == 0x10)
        {
            int sum=0;
            unsigned char *pBuf=&pDataBuf[firstInd+4];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            double AngleVal=sum*0.0001;
            if(pDataBuf[firstInd + 3])
            {
                AngleVal=-AngleVal;
            }
            std::string AngleType;

            if(pDataBuf[firstInd + 2] == 0x00)
            {
                AngleType="横滚角度";
            }
            else if(pDataBuf[firstInd + 2] == 0x01){
                AngleType="俯仰角度";
            }
            else if(pDataBuf[firstInd + 2] == 0x02){
                AngleType="第二横滚角度";
            }
            else if(pDataBuf[firstInd + 2] == 0x03){
                AngleType="水平旋转角度";
            }

            protoCommad=AngleType;
            sprintf(str," : %.4lf°",AngleVal);
            protoCommad.append( str);
        }
        if(pDataBuf[firstInd] == 0x6B && pDataBuf[firstInd + 1] == 0x12)
        {
            int sum=0,kint=3;
//            double hori_value=0.0,second_rotate_value=0.0,rotate_value=0.0,pitch_value=0.0;
            //解析水平旋转角度
            unsigned char *pBuf=&pDataBuf[firstInd+kint+1];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            hori_value=sum*0.0001;
            if(pDataBuf[firstInd + kint])
            {
                hori_value=-hori_value;
            }

            //解析第二横滚角度
            sum=0;
            kint=8;
            pBuf=&pDataBuf[firstInd+kint+1];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            second_rotate_value=sum*0.0001;
            if(pDataBuf[firstInd + kint])
            {
                second_rotate_value=-second_rotate_value;
            }

            //解析第一横滚角度
            sum=0;
            kint=13;
            pBuf=&pDataBuf[firstInd+kint+1];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            rotate_value=sum*0.0001;
            if(pDataBuf[firstInd + kint])
            {
                rotate_value=-rotate_value;
            }

            //解析俯仰角度
            sum=0;
            kint=18;
            pBuf=&pDataBuf[firstInd+kint+1];
            for(int i=0;i<4;i++)
            {
                sum*=100;
                sum+=(pBuf[i] >> 4)*10 + (pBuf[i]& 0x0f);
            }
            pitch_value=sum*0.0001;
            if(pDataBuf[firstInd + kint])
            {
                pitch_value=-pitch_value;
            }

            protoCommad="第二横滚角度为：" + std::to_string(hori_value) + "水平旋转角度为" + std::to_string(second_rotate_value) +
                    "第一横滚角度为" + std::to_string(rotate_value) + "俯仰角度为：" + std::to_string(pitch_value);
        }
    }

    return true;
}

bool PTZ_TCPClientSocket::IsConnect(void)
{
    if (m_srvSocket == -1)
    {
        return false;
    }
    return true;
}

void PTZ_TCPClientSocket::Run(void)
{
    if (!IsConnect())
    {
        if (false == Connect())
        {
            sleep(1);
            return;
        }
    }
}

bool PTZ_TCPClientSocket::initlizePTZParam(QString init_pitch_angle, QString init_rotate_angle)
{
    QString strPort = "10123";
    bool isConnected = false;
    SetServerInfo("192.168.1.80", strPort.toShort());
    isConnected = Connect();
    if(isConnected)
    {
        sendByteDataToPTZ(init_pitch_angle);
        sendByteDataToPTZ(init_rotate_angle);
        QString autoRecvStr = "5A 02 01 00 00 05 00 AA";          //默认自动上报关
        sendByteDataToPTZ(autoRecvStr);
        return false;
    }
    else
    {
//        QMessageBox::information(this, tr("connect"), QString::fromLocal8Bit("云台连接失败，请重新连接!"));
        return true;
    }
}

void PTZ_TCPClientSocket::sendByteDataToPTZ(QString strData)
{
    strData.remove(QRegExp("\\s"));

    bool ok;
    int len = strData.length()/2;
    uchar byteData[len + 1];


    for(int i = 0; i < len; i++)
    {
        QString strTemp = strData.mid(2 * i, 2);
        byteData[i] = strTemp.toInt(&ok, 16);
    }

    SendData(byteData, len);
}


void PTZ_TCPClientSocket::recovery_camera_to_init_position(QString init_pitch_angle, QString init_rotate_angle)
{
    sendByteDataToPTZ(init_pitch_angle);
    usleep(1000 * 1000);
    sendByteDataToPTZ(init_rotate_angle);
}

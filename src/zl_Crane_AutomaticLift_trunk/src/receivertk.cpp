#include "receivertk.hpp"


receiveRTK::receiveRTK()
{

}

void receiveRTK::init()
{
    //打开串口
   int ret;
   fd =open_port(fd,1);

   if(fd ==-1)
   {
       cerr<<"init fail"<<endl;
       return;
   }

   ret =set_opt(fd,115200, 8, 'N', 1);

   if(ret ==-1)
   {
       cerr<<"set comm error"<<endl;
       return;
   }

    bzero(buf, 100);
}

int receiveRTK::open_port(int fd, int comport)
{
    char *dev[]={"/dev/ttyUSB0","/dev/ttyS1","/dev/ttyS2"};

    if (comport==1)//串口1
    {
        fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
    }
    else if(comport==2)//串口2
    {
        fd = open( "/dev/ttyS1", O_RDWR|O_NOCTTY|O_NDELAY); //没有设置<span style="font-family: Arial, Helvetica, sans-serif;">O_NONBLOCK非阻塞模式，也可以设置为非阻塞模式，两个模式在下一篇博客中具体说明</span>

        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
    }
    else if (comport==3)//串口3
    {
        fd = open( "/dev/ttyS2", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
    }
    /*恢复串口为阻塞状态*/
    if(fcntl(fd, F_SETFL, 0)<0)
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    /*测试是否为终端设备*/
    if(isatty(STDIN_FILENO)==0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n",fd);
    return fd;
}

int receiveRTK::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
   /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
       if  ( tcgetattr( fd,&oldtio)  !=  0) {
           perror("SetupSerial 1");
           printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio));
           return -1;
       }
       bzero( &newtio, sizeof( newtio ) );
   /*步骤一，设置字符大小*/
       newtio.c_cflag  |=  CLOCAL | CREAD;
       newtio.c_cflag &= ~CSIZE;
   /*设置停止位*/
       switch( nBits )
       {
           case 7:
               newtio.c_cflag |= CS7;
               break;
           case 8:
               newtio.c_cflag |= CS8;
               break;
       }
   /*设置奇偶校验位*/
       switch( nEvent )
       {
           case 'o':
           case 'O': //奇数
               newtio.c_cflag |= PARENB;
               newtio.c_cflag |= PARODD;
               newtio.c_iflag |= (INPCK | ISTRIP);
               break;
           case 'e':
           case 'E': //偶数
               newtio.c_iflag |= (INPCK | ISTRIP);
               newtio.c_cflag |= PARENB;
               newtio.c_cflag &= ~PARODD;
               break;
           case 'n':
           case 'N':  //无奇偶校验位
               newtio.c_cflag &= ~PARENB;
               break;
           default:
               break;
       }
       /*设置波特率*/
       switch( nSpeed )
       {
           case 2400:
               cfsetispeed(&newtio, B2400);
               cfsetospeed(&newtio, B2400);
               break;
           case 4800:
               cfsetispeed(&newtio, B4800);
               cfsetospeed(&newtio, B4800);
               break;
           case 9600:
               cfsetispeed(&newtio, B9600);
               cfsetospeed(&newtio, B9600);
               break;
           case 115200:
               cfsetispeed(&newtio, B115200);
               cfsetospeed(&newtio, B115200);
               break;
           case 460800:
               cfsetispeed(&newtio, B460800);
               cfsetospeed(&newtio, B460800);
               break;
           default:
               cfsetispeed(&newtio, B9600);
               cfsetospeed(&newtio, B9600);
               break;
       }
   /*设置停止位*/
       if( nStop == 1 )
           newtio.c_cflag &=  ~CSTOPB;
       else if ( nStop == 2 )
           newtio.c_cflag |=  CSTOPB;
   /*设置等待时间和最小接收字符*/
       newtio.c_cc[VTIME]  = 0;
       newtio.c_cc[VMIN] = 0;
   /*处理未接收字符*/
       tcflush(fd,TCIFLUSH);
   /*激活新配置*/
       if((tcsetattr(fd,TCSANOW,&newtio))!=0)
       {
           perror("com set error");
           return -1;
       }
       printf("set done!\n");
       return 0;
}

vector<float> receiveRTK::rtkDataProcess()
{
    vector<float> result;

    int nret;
    bool flag =false;
    string neu;

    bzero(buf, 100);

    usleep(10*1000);

    result.clear();
    nret=read(fd,buf,100);
    if(nret<0){
        cerr<<"fd ="<<fd<<endl;
        perror("read STDIN_FILENO");
        exit(1);
    }
    neu =buf;
    cerr<<"neu ="<<neu<<endl;

    //cerr<<"neu1 ="<<neu1<<endl;
    std::vector<std::string> ret;
    flag = Split(neu, ret, ",");

    if(flag ==false)
    {
        cerr<<"the input neu is empty"<<endl;
    }

    if(ret.size() !=5)
    {
        cerr<<"the data is error "<<ret.size()<<endl;
        return result;
    }

    result.push_back(std::stof(ret[2]));
    result.push_back(std::stof(ret[3]));

    for(int i=0;i <result.size();i++)
    {
        cerr<<"ret ="<<result[i]<<endl;
    }

    return result;
}

bool receiveRTK::Split(const string &str, std::vector<string> &ret, string sep)
{
    if (str.empty()) {
        return false;
    }

    std::string temp;
    std::string::size_type begin = str.find_first_not_of(sep);
    std::string::size_type pos = 0;

    while (begin != std::string::npos) {
        pos = str.find(sep, begin);
        if (pos != std::string::npos) {
            temp = str.substr(begin, pos - begin);
            begin = pos + sep.length();
        } else {
            temp = str.substr(begin);
            begin = pos;
        }

        if (!temp.empty()) {
            ret.push_back(temp);
            temp.clear();
        }
    }
    return true;
}

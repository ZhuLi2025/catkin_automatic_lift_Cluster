//
// Created by xpp on 2022/6/24.
//

#include "udpRtk.hpp"
#include "zl_common_define.hpp"

extern zl_common::app::CRANE_PARAM *crane_parameter_;
udpRtk::udpRtk() {
    hookData =0.0;
}


udpRtk::~udpRtk() {

}

int udpRtk::init() {
    sfd=socket(AF_INET,SOCK_DGRAM,0);//服务器生成一个socket描述符sfd，采用ipv4和udp协议通信
    if(-1==sfd)
    {
        perror("socket");
        return -1;
    }
    struct sockaddr_in ser;//定义服务器的socket地址结构ser，指明服务器的通信协议、ip地址，端口号
    memset(&ser,0,sizeof(ser));//清空地址结构
    ser.sin_family=AF_INET;//采用ipv4协议
    ser.sin_port=htons(9906);//端口号，将主机字节序的port转换为网络字节序的port
    ser.sin_addr.s_addr=inet_addr("192.168.18.93");//ip，将点分十进制的主机字节序的ip转换为32位二进制数值的网络字节序的ip
    int ret;
    ret=bind(sfd,(struct sockaddr*)&ser,sizeof(ser));//udp进行bind后，端口就打开；将生成的sfd与服务器的socket信息数据绑定
    if(-1==ret)
    {
        perror("bind error");
        return -1;
    }

    //设置阻塞超时
    struct timeval timeout;
    timeout.tv_sec =5;
    timeout.tv_usec =0;
    if(setsockopt(sfd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout)) <0)
    {
        printf("udp time out setting failed\n");
    }

    return 0;
}

void udpRtk::process() {
    int ret;
    bool flag = false;
    vector<double> tempCarHead;
    vector<double> tempArm;

    char buf[1000]={0};
    struct sockaddr_in cli;//定义客户端的socket地址结构，用于接收客户端的ip和port信息
    socklen_t len;
    string neu;

    tempCarHead.clear();
    tempArm.clear();
    memset(&buf,0,sizeof(buf));
    memset(&cli,0,sizeof(cli));
    len=sizeof(cli);
    ret=recvfrom(sfd,buf,sizeof(buf),0,(struct sockaddr*)&cli,&len);
    //ret=recvfrom(sfd,buf,sizeof(buf),0,NULL,NULL);
    if(-1==ret)
    {
        perror("recvfrom");
        return;
    }
    neu =buf;

//    string aa;
//    aa =cli.sin_addr.s_addr

    std::vector<std::string> ret1;
    flag = split(neu, ret1, ",");

//    cerr<<"ret1 size ="<<ret1.size()<<endl;
    if(ret1.size() ==28)//车头数据
    {
        //获取航向角
        //tempCarHead.push_back(stod(ret1[3]));//角度
        tempCarHead.push_back(stod(ret1[3]) - 90.0);//角度  轮胎吊要加90度
        tempCarHead.push_back(stod(ret1[25]));//N
        tempCarHead.push_back(stod(ret1[26]));//E
        tempCarHead.push_back(stod(ret1[14]) - 2.0);//高度

//        tempCarHead.push_back(86.9);//角度
//        tempCarHead.push_back(3123586.060);
//        tempCarHead.push_back(396206.343);
        carHeadData =tempCarHead;
        carHeadingAngel =stod(ret1[3]) + 90.0;
        carHeadingAngel =carHeadingAngel >= 360.0 ? (carHeadingAngel -360.0) : carHeadingAngel;
    }

    if(ret1.size() ==5)//臂尖数据
    {
        tempArm.push_back(stod(ret1[2]));
        tempArm.push_back(stod(ret1[3]));
        armHeadData =tempArm;
        //cerr<<"neu ="<<neu<<endl;
    }

    if(ret1.size() ==24)//吊钩rtk
    {
        hookData =stod(ret1[14]);
//        cerr<<"hookData ="<<hookData<<endl;
    }

    return;
}

bool udpRtk::split(const std::string &str, std::vector<std::string> &ret, std::string sep) {
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
//注意:坐标系旋转和坐标系中点的旋转不同
//下车沿着车身
vector<double> udpRtk::getRotationAmp(vector<double> cuPosition) {
    vector<double> tempRotAmp;

    //正北 正东相对位置
    double Ndist =0;
    double Edist =0;

    tempRotAmp.clear();

    if(cuPosition.size() !=2 || carHeadData.size() !=4)
    {
        cerr<<"the input data is error please inpu again the size is ="<<cuPosition.size()<<endl;
        return tempRotAmp;
    }

    //平移到参考系的坐标
    Ndist =cuPosition[0] - carHeadData[1];
    Edist =cuPosition[1] - carHeadData[2];

    //转到车身坐标系
    //旋转后,正北沿着车头向前,顺时针旋转
    double car_x =0;
    double car_y =0;

    car_y =Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    car_x =Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180);

    //平移到回转中心
    double translate_x =1.275;
    double translate_y =3.36;

    car_y =car_y + translate_y;
    car_x =car_x + translate_x;

    //计算回转变幅长
    double amplitude =0;
    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
        //rotation = 360 - rotation;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
        //rotation = 360 - rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
        //rotation = 360 - rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/fabs(car_x))*180/M_PI;
        rotation =270 + rotation;
        //rotation = 360 - rotation;
    }

    tempRotAmp.clear();
    tempRotAmp.push_back(rotation);
    tempRotAmp.push_back(amplitude);

    //rotationAmp =tempRotAmp;
    //cout<<"amplitude ="<<amplitude<<endl;
    //cout<<"rotation ="<<rotation<<endl;
    return tempRotAmp;
}

//轮胎吊要加90度,装下车底部，垂直车身
vector<double> udpRtk::getRotationAmpTyre(vector<double> cuPosition) {
    vector<double> tempRotAmp;

    //正北 正东相对位置
    double Ndist =0;
    double Edist =0;

    tempRotAmp.clear();

    if(cuPosition.size() !=2 || carHeadData.size() !=4)
    {
        cerr<<"the input data is error please inpu again the size is ="<<cuPosition.size()<<endl;
        return tempRotAmp;
    }

    //平移到参考系的坐标
    Ndist =cuPosition[0] - carHeadData[1];
    Edist =cuPosition[1] - carHeadData[2];

    //转到车身坐标系
    //旋转后,正北沿着车头向前,顺时针旋转
    double car_x =0;
    double car_y =0;

    //轮胎
    car_y =Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    car_x =Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180);

    //平移到回转中心
    double translate_x =0;
    double translate_y =8.1;

    car_y =car_y + translate_y;

    //计算回转变幅长
    double amplitude =0;
    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/fabs(car_x))*180/M_PI;
        rotation =270 + rotation;
    }

    tempRotAmp.clear();
    tempRotAmp.push_back(rotation);
    tempRotAmp.push_back(amplitude);

    //rotationAmp =tempRotAmp;
    //cout<<"amplitude ="<<amplitude<<endl;
    //cout<<"rotation ="<<rotation<<endl;
    return tempRotAmp;
}

//装上车顶部，沿着驾驶室或垂直驾驶室，变换航向角90度
vector<double> udpRtk::getRotationAmp2(vector<double> curPosition, const double carRotation) {

    vector<double> tempRotAmp;

    if (curPosition.size() !=2 || carHeadData.size() !=4)
    {
        cerr<<"the rtk position is error Please check rtk"<<endl;
        return tempRotAmp;
    }
//step 1 大地坐标系平移到rtk基准点为原点的正北正东坐标系

    double Ndist =0;//rtk基准点为原点,正北正东坐标系
    double Edist =0;

    //平移到参考系的坐标
    Ndist =curPosition[0] - carHeadData[1];
    Edist =curPosition[1] - carHeadData[2];

//step 2 rtk基准点的正北正东坐标系旋转航向角到以rtk连线为y轴正方向坐标系
    double carRtk_y =0;
    double carRtk_x =0;

    carRtk_y =Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    carRtk_x =Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180);

//step 3 rtk连线为y轴正方向的旋转回转角相关角度到以车身朝前y轴正方向坐标系
    double carCab_y =0.0;
    double carCab_x =0.0;
    //臂在车身右边
    double boomToCarAngel =0;
    if (carRotation >180.0 && carRotation <=360.0)
    {
        boomToCarAngel =carRotation - 360.0;
    }
    else if(carRotation >=0.0 && carRotation <180.0)  //臂在车身左边
    {
        boomToCarAngel =carRotation;
    }

    carCab_y =carRtk_y*cos(boomToCarAngel*M_PI/180) + carRtk_x*sin(boomToCarAngel*M_PI/180);//这是坐标系的旋转
    carCab_x =carRtk_x*cos(boomToCarAngel*M_PI/180) - carRtk_y*sin(boomToCarAngel*M_PI/180);

//step 4 rtk基准点平移到回转中心
    double car_y =0.0;
    double car_x =0.0;
//    double translate_y = -0.63;
//    double translate_x = -0.93;

    double translate_y = 0.65;
    double translate_x = -1.03;

    car_y =carCab_y + translate_y;
    car_x =carCab_x + translate_x;

    //计算回转变幅长
    double amplitude =0;
    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/fabs(car_x))*180/M_PI;
        rotation =270 + rotation;
    }


    tempRotAmp.clear();
    tempRotAmp.push_back(rotation);
    tempRotAmp.push_back(amplitude);

    //rotationAmp =tempRotAmp;
    //cout<<"amplitude ="<<amplitude<<endl;
    //cout<<"rotation ="<<rotation<<endl;
    return tempRotAmp;
}


vector<double> udpRtk::getRotationAmpEx(vector<double> cuPosition, double dLuffAngle, double dLuffLen){
    vector<double> tempRotAmp;

    //正北 正东相对位置
    double Ndist =0;
    double Edist =0;

    tempRotAmp.clear();

    if(cuPosition.size() !=2 || carHeadData.size() !=4)
    {
        cerr<<"the input data is error please inpu again the size is ="<<cuPosition.size()<<endl;
        return tempRotAmp;
    }

    //平移到参考系的坐标
    Ndist =cuPosition[0] - carHeadData[1];
    Edist =cuPosition[1] - carHeadData[2];

    //转到车身坐标系
    //旋转后,正北沿着车头向前,顺时针旋转
    double car_x =0;
    double car_y =0;

    car_y = Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    car_x = Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180) + 0.85;
//    double car_yy_offset = car_y + dLuffLen + 0.85*cos(_dAngleValue* M_PI/180);

    //平移到回转中心
    double translate_x =0;
    double translate_y =8.1;

    car_y = car_y + translate_y;

    //计算回转变幅长
    double amplitude =0;
    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/fabs(car_x))*180/M_PI;
        rotation =270 + rotation;
    }

    tempRotAmp.clear();
    tempRotAmp.push_back(rotation);
    tempRotAmp.push_back(amplitude);

    //rotationAmp =tempRotAmp;
    //cout<<"amplitude ="<<amplitude<<endl;
    //cout<<"rotation ="<<rotation<<endl;
    return tempRotAmp;
}

//装配重上  连线垂直臂架
vector<double> udpRtk::getCurrRotationAmp(vector<double> currPoint,const double carRotation)
{
    vector<double> tempRotAmp;

    if (currPoint.size() !=2 || carHeadData.size() !=4)
    {
        cerr<<"the rtk position is error Please check rtk"<<endl;
        return tempRotAmp;
    }

    tempRotAmp.clear();
//step 1 大地坐标系平移到rtk基准点为原点的正北正东坐标系

    double Ndist =0;//rtk基准点为原点,正北正东坐标系
    double Edist =0;

    //平移到参考系的坐标
    Ndist =currPoint[0] - carHeadData[1];
    Edist =currPoint[1] - carHeadData[2];

//step 2 rtk基准点的正北正东坐标系旋转航向角到以rtk连线为y轴正方向坐标系,沿这臂架投影方向为y轴正反向
    double carRtk_y =0;
    double carRtk_x =0;

    carRtk_y =Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    carRtk_x =Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180);

//step 3 rtk基准点圆心,沿着臂架投影方向为y轴正方向,平移圆心到回转中心
    double translate_y =3.45;
    double translate_x =1.075;

    carRtk_y =carRtk_y - translate_y;
    carRtk_x =carRtk_x - translate_x;

    //step 4 回转中心为圆心,臂架投影方向为y轴正方向,旋转回转角相关角度到以车身方向为y轴正方向

    double car_y =0.0;
    double car_x =0.0;
    //臂在车身右边
    double boomToCarAngel =0;
    if (carRotation >180.0 && carRotation <=360.0)
    {
        boomToCarAngel =carRotation - 360.0;
    }
    else if(carRotation >=0.0 && carRotation <180.0)  //臂在车身左边
    {
        boomToCarAngel =carRotation;
    }

    car_y =carRtk_y*cos(boomToCarAngel*M_PI/180) + carRtk_x*sin(boomToCarAngel*M_PI/180);//这是坐标系的旋转
    car_x =carRtk_x*cos(boomToCarAngel*M_PI/180) - carRtk_y*sin(boomToCarAngel*M_PI/180);

    //计算回转变幅长
    double amplitude =0;
    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/fabs(car_x))*180/M_PI;
        rotation =270 + rotation;
    }
    //cerr<<"amplitude ="<<amplitude<<endl;
    //cerr<<"rotation ="<<rotation<<endl;

    tempRotAmp.clear();
    tempRotAmp.push_back(rotation);
    tempRotAmp.push_back(amplitude);

    //rotationAmp =tempRotAmp;
    //cout<<"amplitude ="<<amplitude<<endl;
    //cout<<"rotation ="<<rotation<<endl;
    return tempRotAmp;
}

vector<double> udpRtk::rtkTocar(double Ndist,double Edist,double height,const double carRotation)
{
    //正北 正东相对位置
    vector<double> carPosition;
    carPosition.clear();

    if (carHeadData.size() !=4)
    {
        carPosition.push_back(0);
        carPosition.push_back(0);
        carPosition.push_back(0);
        cerr<<"the rtk position is error Please check rtk"<<endl;
        return carPosition;
    }

//step 2 rtk基准点的正北正东坐标系旋转航向角到以rtk连线为y轴正方向坐标系,沿这臂架投影方向为y轴正反向
    double carRtk_y =0;
    double carRtk_x =0;

    carRtk_y =Ndist*cos(carHeadData[0]*M_PI/180) + Edist*sin(carHeadData[0]*M_PI/180);//这是坐标系的旋转
    carRtk_x =Edist*cos(carHeadData[0]*M_PI/180) - Ndist*sin(carHeadData[0]*M_PI/180);

//step 3 rtk基准点圆心,沿着臂架投影方向为y轴正方向,平移圆心到回转中心
    double translate_y =3.36;
    double translate_x =1.1;

    carRtk_y =carRtk_y - translate_y;
    carRtk_x =carRtk_x - translate_x;

//step 4 回转中心为圆心,臂架投影方向为y轴正方向,旋转回转角相关角度到以车身方向为y轴正方向

    double car_y =0.0;
    double car_x =0.0;
    double car_z = 0.0;

    car_z =height - carHeadData[3];

    //臂在车身右边
    double boomToCarAngel =0;
    if (carRotation >180.0 && carRotation <=360.0)
    {
        boomToCarAngel =carRotation - 360.0;
    }
    else if(carRotation >=0.0 && carRotation <180.0)  //臂在车身左边
    {
        boomToCarAngel =carRotation;
    }

    car_y =carRtk_y*cos(boomToCarAngel*M_PI/180) + carRtk_x*sin(boomToCarAngel*M_PI/180);//这是坐标系的旋转
    car_x =carRtk_x*cos(boomToCarAngel*M_PI/180) - carRtk_y*sin(boomToCarAngel*M_PI/180);

    carPosition.push_back(car_y);
    carPosition.push_back(car_x);
    carPosition.push_back(car_z);

    return carPosition;
}

    vector<double> udpRtk::rtkCenter()
    {

    vector<double> groundAxis;

    groundAxis.clear();

    if (carHeadData.size() !=4)
    {
        cerr<<"the rtk position is error Please check rtk"<<endl;
        groundAxis.push_back(0);
        groundAxis.push_back(0);
        groundAxis.push_back(0);
        return groundAxis;
    }

    //step 1 知道rtk基准点为圆心,臂架投影为y轴正方向,右手垂直为x轴正方向下,回转中心的坐标值
    double y =3.36;
    double x =1.1;

    //step 2 旋转航向角反角度到正北 正东坐标系,得到rtk基准点为圆心的正北 正东坐标系

    double Ndist =0.0;
    double Edist =0.0;

    Ndist =y*cos(-carHeadData[0]*M_PI/180) + x*sin(-carHeadData[0]*M_PI/180);//这是坐标系的旋转
    Edist =x*cos(-carHeadData[0]*M_PI/180) - y*sin(-carHeadData[0]*M_PI/180);
    //step 3 平移rtk基准点到大地圆心得到大地坐标系
    double ground_N =0.0;
    double ground_E =0.0;

    ground_N =Ndist + carHeadData[1];
    ground_E =Edist + carHeadData[2];

    groundAxis.push_back(ground_N);
    groundAxis.push_back(ground_E);
    groundAxis.push_back(carHeadData[3]);

    return groundAxis;
}


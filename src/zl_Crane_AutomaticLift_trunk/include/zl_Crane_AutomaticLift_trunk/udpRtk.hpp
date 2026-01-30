//
// Created by xpp on 2022/6/24.
//

#ifndef SEIALCOMM_UDPRTK_H
#define SEIALCOMM_UDPRTK_H

#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <netdb.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

class udpRtk {
public:
    udpRtk();
    ~udpRtk();

    int init();

    void process();

    bool split(const std::string &str, std::vector<std::string> &ret, std::string sep);
    vector<double> getRotationAmp(vector<double> cuPosition);
    vector<double> getRotationAmp2(vector<double> curPosition,const double carRotation);
    vector<double> getRotationAmpEx(vector<double> cuPosition, double dLuffAngle,  double dLuffLen);
    vector<double> getRotationAmpTyre(vector<double> cuPosition);//轮胎吊
    vector<double> getCurrRotationAmp(vector<double> currPoint,const double carRotation);

    vector<double> rtkTocar(double Ndist, double Edist, double height, const double carRotation);
    vector<double> rtkCenter();

private:    

public:
    vector<double> armHeadData;
    vector<double> rotationAmp;
    vector<double> carHeadData;

    double hookData;

    double carHeadingAngel{0.0};

private:
    int sfd;


};


#endif //SEIALCOMM_UDPRTK_H

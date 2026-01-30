#ifndef RECEIVERTK_H
#define RECEIVERTK_H

#include <iostream>
#include<unistd.h>
#include<stdlib.h>
#include<fcntl.h>
#include<termios.h>
#include <string.h>
#include <vector>
#include <QString>
#include <qstring.h>

using namespace std;


class receiveRTK
{
public:
    receiveRTK();

    void init();

    vector<float> rtkDataProcess();

private:
    int open_port(int fd,int comport);

    int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

    bool Split(const std::string &str, std::vector<std::string> &ret, std::string sep);

public:

private:
    int fd;
    char buf[100];

};

#endif // RECEIVERTK_H

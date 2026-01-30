#ifndef CLIENTUSBTOCANCOMMUNICATION_H
#define CLIENTUSBTOCANCOMMUNICATION_H
#include "PCANBasic.h"
#include <QByteArray>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <asm/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <QMessageBox>

#undef UINT64

class clientUsbToCanCommunication
{
public:
    clientUsbToCanCommunication();
    ~clientUsbToCanCommunication();

    int OpenUsbCanDevice();
    int ReceiveCanData();
    bool SendCanData(QString strDataID, QString strData);
    void UpdateCanData(QString &byteValue, int bitPos, int bitValue);
    int analyzeData(int analyzeType, int data[8], int bytePos, int bitPos);
    void SplitCanDataByte(int data, QString &byteValueOne, QString &byteValueTwo);

//    static void signal_handler(int s);
//    static int setup_sig_handler(int signum, void (*f)(int));

public:
    unsigned int pcan_device;
    TPCANStatus Status;

public:
    QList<QString> Data01D0;		//发送ID数据
    QList<QString> Data02D0;
    QList<QString> Data02D1;
    QList<QString> Data03D0;
    QList<QString> Data01D1;
    QList<QString> Data04D0;

    QList<QString> Data01A2;
    QList<QString> Data02A2;
    QList<QString> Data03A2;

    int Data01B7[8];			//接收的ID数据
    int Data048F[8];
    int Data02B7[8];
    int Data03B7[8];
    int Data04B7[8];
    int Data0383[8];

    void clearCommand();
};

#endif // CLIENTUSBTOCANCOMMUNICATION_H

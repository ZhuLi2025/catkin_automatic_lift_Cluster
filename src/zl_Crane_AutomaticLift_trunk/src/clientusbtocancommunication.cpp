#include "clientusbtocancommunication.hpp"
#include <QMessageBox>

#define PCAN_DEVICE     PCAN_USBBUS1

clientUsbToCanCommunication::clientUsbToCanCommunication()
{
    pcan_device = PCAN_DEVICE;

    for (int i = 0; i < 8; i++)
    {
        Data01B7[i] = 0 ;
        Data048F[i] = 0 ;
        Data02B7[i] = 0 ;
        Data03B7[i] = 0 ;
        Data04B7[i] = 0 ;
        Data0383[i] = 0;

        //        Data01D0.append(QString("%i").arg(00));
        //        Data02D0.append(QString("%i").arg(00));
        //        Data03D0.append(QString("%i").arg(00));
        //        Data01D1.append(QString("%i").arg(00));

    }
    Data01D0<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data02D0<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data02D1<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data03D0<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data04D0<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data01D1<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data01A2<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data02A2<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
    Data03A2<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00"<<"00";
}

void clientUsbToCanCommunication::clearCommand()
{
    for (int i = 0; i < 8; i++)
    {
        Data01D0.replace(i,"00");
        Data02D0.replace(i,"00");
        Data02D1.replace(i,"00");
        Data03D0.replace(i,"00");
        Data04D0.replace(i,"00");
        Data01D1.replace(i,"00");
        Data01A2.replace(i,"00");
        Data02A2.replace(i,"00");
        Data03A2.replace(i,"00");
    }
}

clientUsbToCanCommunication::~clientUsbToCanCommunication()
{
    CAN_Uninitialize(pcan_device);
    //关闭USB-CAN设备
    //VCI_CloseDevice(m_DevType, m_DevIndex);
}


//static void signal_handler(int s)
//{
//    printf("Interrupted by SIG%u!\n", s);
//}

//static int setup_sig_handler(int signum, void (*f)(int))
//{
//    struct sigaction act;

//    memset(&act, 0, sizeof act);
//    act.sa_handler = f;

//    // note: siagaction() is thread -safe
//    return sigaction(signum, &act, NULL);
//}

int clientUsbToCanCommunication::OpenUsbCanDevice()
{ 
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pcan_device = PCAN_DEVICE;
    Status = CAN_Initialize(pcan_device, PCAN_BAUD_250K, 0, 0, 0);
    printf("CAN_GetValue(%xh) Status = 0x%x\n", pcan_device, (int) Status);
    if(Status != PCAN_ERROR_OK)
    {
        QMessageBox::information(nullptr, ("错误"),("CAN_Initialize is failed!"));
        CAN_Uninitialize(pcan_device);
        return 1;
    }

    return 0;
}

int clientUsbToCanCommunication::ReceiveCanData()
{
    TPCANMsg Message;
    pcan_device = PCAN_DEVICE;
    Status = CAN_Read(pcan_device, &Message, NULL);

    if (Status == PCAN_ERROR_QRCVEMPTY)
    {
        printf("CAN_Read(%xh) Abnormal PCAN_ERROR_QRCVEMPTY status. Wating 1s before looping (^C to stop)...\n", pcan_device);
        return 1;
    }

    if (Status != PCAN_ERROR_OK)
    {
        printf("CAN_Read(%xh) failure 0x%x\n", pcan_device, (int) Status);
        return 1;
    }

    // emergency exit...
    if (Message.ID == 0xff && Message.LEN == 1 && Message.DATA[0] == 0xff)
    {
        return 1;
    }

    if(Message.ID == 0x01B7)                        //01B7
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data01B7[i] = Message.DATA[i];
        }
    }
    if(Message.ID == 0x048F)                        //048F
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data048F[i] = Message.DATA[i];
        }
    }
    if(Message.ID == 0x02B7)                        //02B7
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data02B7[i] = Message.DATA[i];
        }
    }
    if(Message.ID == 0x03B7)                        //02B7
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data03B7[i] = Message.DATA[i];
        }
    }
    if(Message.ID == 0x04B7)                        //02B7
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data04B7[i] = Message.DATA[i];
        }
    }
    if(Message.ID == 0x0383)                        //02B7
    {
        for (int i = 0; i < Message.LEN; i++)			//数据信息
        {
            Data0383[i] = Message.DATA[i];
        }
    }


    return 0;
}

bool clientUsbToCanCommunication::SendCanData(QString strDataID, QString strData)
{
    TPCANMsg Message;
    bool ok;
    pcan_device = PCAN_DEVICE;

    mlockall(MCL_CURRENT | MCL_FUTURE);

    // be INTRuptible by user
    //    signal(SIGINT, signal_handler);

    Message.ID = strDataID.toInt(&ok,16); // 表示以16进制方式读取字符串
    Message.LEN = 8;
    Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    for(int i = 0; i < Message.LEN; i++)
    {
        QString strTemp = strData.mid(2 * i, 2);
        Message.DATA[i] = strTemp.toInt(&ok, 16);
    }

    if((Status = CAN_Write(pcan_device, &Message)) != PCAN_ERROR_OK)
    {
        printf("CAN_Write(%xh) failure 0x%x\n", pcan_device, (int) Status);
        return 1;
    }

    if (Status != PCAN_ERROR_QXMTFULL)
    {
        printf("CAN_Write(%xh): Error 0x%x\n", pcan_device, (int)Status);
        return 1;
    }

    return 0;
}
//更新CAN发送的数据：byteValue->数组字节，bitPos->数据位，bitValue->数值0或1
void clientUsbToCanCommunication::UpdateCanData(QString &byteValue, int bitPos, int bitValue)
{
    bool ok;
    int nValue = byteValue.toInt(&ok, 16);
    if (bitValue)
    {
        nValue = nValue | (1 << bitPos);
    }
    else
    {
        nValue = nValue &~(1 << bitPos);
    }
    byteValue = QString("%1").arg(nValue,2,16,QLatin1Char('0')).toUtf8();  // 保留2位，不足补零
}

int clientUsbToCanCommunication::analyzeData(int analyzeType, int data[8], int bytePos, int bitPos)
{
    int nResult = 0;
    if (analyzeType == 0)
    {
        nResult = (data[bytePos] >> bitPos) & 1;
    }
    //获取计算公式中的分子部分 bytePos - 地位， bitPos - 数据个数2-3-4
    if (analyzeType == 1)
    {
        if (bitPos == 2)
        {
            nResult = 256 * data[bytePos + 1] + data[bytePos];
        }
        else if (bitPos == 3)
        {
            nResult = 256 * 256 * data[bytePos + 2]+ 256 * data[bytePos + 1] + data[bytePos];
        }
        else if(bitPos == 4)
        {
            nResult = 256 * 256 * 256 * data[bytePos + 3] + 256 * 256 * data[bytePos + 2] + 256 * data[bytePos + 1] + data[bytePos];
        }
    }
    return nResult;
}

void clientUsbToCanCommunication::SplitCanDataByte(int data, QString &byteValueOne, QString &byteValueTwo)
{
    QString tempData = "";
    //    char x[25] = { 0 };
    if (data < 0)
    {
        data = data + 65536;
    }
    tempData = QString("%1").arg(data,4,16,QLatin1Char('0')).toUtf8();  // 保留4位，不足补零

    byteValueOne = tempData.mid(0, 2);
    byteValueTwo = tempData.mid(2, 2);
}

#ifndef PCANMANAGER_H
#define PCANMANAGER_H

#include <ros/ros.h>
#include <PCANBasic.h>
#include <vector>
#include <algorithm> // 用于 std::find
#include <string>
#include <mutex>
#include <thread>
#include <cmath>
#include <atomic>
#include <HookLoadAntiCollisionDetection/CraneStatus.h>
#include <HookLoadAntiCollisionDetection/CraneInfo.h>
#include <map>



class PCANManager
{
public:
    enum VIEHCLE { ZCC2600_2, ZCC2600RCC, ZCC500 };

    PCANManager(ros::NodeHandle& nh, bool toConnect = true);
    ~PCANManager();

    void startReceiveThread();
    void stop();
private:
    std::map<std::string, TPCANHandle> deviceMap = {
        {"USB1", PCAN_USBBUS1},
        {"USB2", PCAN_USBBUS2}
    };

    std::map<std::string, TPCANBaudrate> baudMap = {
        {"250K", PCAN_BAUD_250K},
        {"500K", PCAN_BAUD_500K}
    };
    void thread_task();    // 接收数据的线程
    void monitor_task();   // 监控连接状态的线程
    void updateInfo(std::string canID, const std::vector<int>& payload);
    TPCANStatus ReadMessage();
    void ProcessMessageCan(TPCANMsg msg);
    void publishCraneData() ;// 发布解析后的数据

    int openDevice();
    int closeDevice();
    
    // 基础配置
    std::string pcan_handle = "USB1"; 
    std::string pcan_baud = "250K"; 
    TPCANHandle pcanHandle;
    TPCANBaudrate bitRate;
    const int MAX_PATH   = 255;
    static VIEHCLE viehcle;    // 车型

    // 状态管理
    std::atomic<bool> isConnected {false};
    std::atomic<bool> toFinishTask {false};
    std::mutex dataMutex;
    std::mutex statusMutex;
    
    CraneInfo craneInfo;
    ros::NodeHandle nh_;
    // 消息发布器   
    ros::Publisher crane_pub_;
    std::thread* recvThread = nullptr;
    std::thread* monitorThread = nullptr;

    // 数据临时存储
    std::vector<int> Data01B7;			//接收的ID数据
    std::vector<int> Data048F;
    std::vector<int> Data02B7;
    std::vector<int> Data03B7;
    std::vector<int> Data04B7;
    std::vector<int> Data0383;
};
#endif // PCANMANAGER_H

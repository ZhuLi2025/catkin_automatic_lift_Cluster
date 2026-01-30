#ifndef SOCKET_CAN_MANAGER_H
#define SOCKET_CAN_MANAGER_H

#include <ros/ros.h>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <HookLoadAntiCollisionDetection/CraneStatus.h> // 替换为你的消息头文件

class SocketCANManager {
public:
    SocketCANManager(ros::NodeHandle& nh, std::string interface = "can0");
    ~SocketCANManager();

    void startReceiveThread();
    void stop();

private:
    void receiveThread();
    void processFrame(const struct can_frame& frame);

    // ROS 成员
    ros::NodeHandle nh_;
    ros::Publisher crane_pub_;

    // SocketCAN 成员
    int socket_fd_ = -1;
    std::string interface_;
    std::atomic<bool> is_running_ {false};
    std::thread* recv_thread_ = nullptr;

    // 数据与同步
    std::mutex data_mutex_;
    // 建议：如果你需要多车型支持，可以在这里保留之前的解析逻辑
};

#endif
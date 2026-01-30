#include <ros/ros.h>

// 根据宏定义包含头文件
#ifdef USE_PCAN
    #include <HookLoadAntiCollisionDetection/PCANManager.h>
#else
    #include <HookLoadAntiCollisionDetection/SocketCANManager.h>
#endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_bridge_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // 用于读取私有参数

#ifdef USE_PCAN
    PCANManager canManager(nh);
#else
    // --- SocketCAN 配置 ---
    std::string can_interface;
    private_nh.param<std::string>("can_device", can_interface, "can0");

    ROS_INFO("[CAN_BRIDGE] Using SocketCAN: Device=%s", can_interface.c_str());
    // 传入构造函数
    SocketCANManager canManager(nh, can_interface);
#endif
    canManager.startReceiveThread();
    ros::spin();
    return 0;
}
#include <HookLoadAntiCollisionDetection/PCANManager.h>

// 静态成员初始化
PCANManager::VIEHCLE PCANManager::viehcle = PCANManager::ZCC500;

PCANManager::PCANManager(ros::NodeHandle& nh, bool toConnect) : nh_(nh) {
    // 话题名称为 "crane_info", 队列长度为 10
    crane_pub_ = nh_.advertise<HookLoadAntiCollisionDetection::CraneStatus>("crane_info", 10);
    nh.param<std::string>("pcan_handle",pcan_handle,"USB1");
    nh.param<std::string>("pcan_baud",pcan_baud,"250K");
    pcanHandle = deviceMap[pcan_handle];
    bitRate = baudMap[pcan_baud];
    ROS_INFO("[CAN_BRIDGE] Using PCAN: Handle= %s", pcan_handle.c_str());

    memset(&craneInfo, 0, sizeof(CraneInfo));
    craneInfo.mainArmLength         = 0;
    craneInfo.mainArmAngle          = 0;
    craneInfo.subArmLength          = 0;
    craneInfo.subArmAngle           = 0;
    craneInfo.mainArmToGroundHeight = 0;
    craneInfo.subArmToGroundHeight  = 0;
    craneInfo.rotationAngle         = 0;
    craneInfo.mainHookToGroundHeight= 0;
    craneInfo.subHookToGroundHeight = 0;
    craneInfo.armStatus             = 0;

    for (int i = 0; i < 8; i++)
    {
        Data01B7[i] = 0 ;
        Data048F[i] = 0 ;
        Data02B7[i] = 0 ;
        Data03B7[i] = 0 ;
        Data04B7[i] = 0 ;
        Data0383[i] = 0;
    }

    if (toConnect) openDevice();
}

PCANManager::~PCANManager() {
    stop();
    closeDevice();
}

void PCANManager::startReceiveThread() {
    toFinishTask = false;
    recvThread = new std::thread(&PCANManager::thread_task, this);
    monitorThread = new std::thread(&PCANManager::monitor_task, this);
}

void PCANManager::stop() {
    toFinishTask = true;
    if (recvThread && recvThread->joinable()) recvThread->join();
    if (monitorThread && monitorThread->joinable()) monitorThread->join();
}

int PCANManager::openDevice() {
    CAN_Uninitialize(pcanHandle);
    TPCANStatus res = CAN_Initialize(pcanHandle, bitRate);
    if (res != PCAN_ERROR_OK) {
        ROS_ERROR("PCAN Initialize failed. Code: %Xh", res);
        std::lock_guard<std::mutex> lock(statusMutex);
        isConnected = false;
        return -1;
    }
    ROS_INFO("PCAN Connected Successfully.");
    std::lock_guard<std::mutex> lock(statusMutex);
    isConnected = true;
    return 0;
}

void PCANManager::monitor_task() {
    while (ros::ok() && !toFinishTask.load()) {
        TPCANStatus result = CAN_GetStatus(pcanHandle);
        {
            std::lock_guard<std::mutex> lock(statusMutex);
            if (result != PCAN_ERROR_OK) {
                if (isConnected.load()) ROS_WARN("PCAN Connection lost, retrying...");
                isConnected = false;
                openDevice();
            } else {
                isConnected = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void PCANManager::thread_task() {
    ROS_INFO("PCAN Receive Thread Started.");
    while (ros::ok() && !toFinishTask.load()) {
        bool connected;
        {
            std::lock_guard<std::mutex> lock(statusMutex);
            connected = isConnected.load();
        }
        if (connected) {
            ReadMessage();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

TPCANStatus PCANManager::ReadMessage() {
    TPCANMsg msg;
    TPCANTimestamp ts;
    TPCANStatus res = CAN_Read(pcanHandle, &msg, &ts);
    if (res == PCAN_ERROR_OK) {
        ProcessMessageCan(msg);
    }
    return res;
}

void PCANManager::ProcessMessageCan(TPCANMsg msg) {
    if (msg.MSGTYPE != PCAN_MESSAGE_STANDARD) return;
    
    // 紧急退出逻辑
    if(msg.ID == 0xff && msg.LEN == 1 && msg.DATA[0] == 0xff){
        ROS_WARN("Received Emergency Exit Signal via CAN.");
        return;
    }
    const int dlc = msg.LEN;

    // 按 ID 分发
    if(msg.ID == 0x01B7){
        for (int i=0;i<dlc;i++) Data01B7[i] = (int)(msg.DATA[i]);
        updateInfo("1B7", Data01B7);
        return;
    }
    if(msg.ID == 0x048F){
        for (int i=0;i<dlc;i++) Data048F[i] = (int)(msg.DATA[i]);
        updateInfo("48F", Data048F);
        return;
    }
    if(msg.ID == 0x02B7){
        for (int i=0;i<dlc;i++) Data02B7[i] = (int)(msg.DATA[i]);
        updateInfo("2B7", Data02B7);
        return;
    }
    if(msg.ID == 0x03B7){
        for (int i=0;i<dlc;i++) Data03B7[i] = (int)(msg.DATA[i]);
        updateInfo("3B7", Data03B7);
        return;
    }
    if(msg.ID == 0x04B7){
        for (int i=0;i<dlc;i++) Data04B7[i] = (int)(msg.DATA[i]);
        updateInfo("4B7", Data04B7);
        return;
    }
    if(msg.ID == 0x0383){
        for (int i=0;i<dlc;i++) Data0383[i] = (int)(msg.DATA[i]);
        updateInfo("383", Data0383);
        return;
    }
}

void PCANManager::updateInfo(std::string canID, const std::vector<int>& payload) {
    int status = -1;
    std::lock_guard<std::mutex> lock(dataMutex);

    std::vector<std::string> canIDList = {"1B7", "48F", "2B7", "3B7", "4B7","383"};
    auto it = std::find(canIDList.begin(), canIDList.end(), canID);
    int index = (it != canIDList.end()) ? std::distance(canIDList.begin(), it) : -1;

    switch (index) 
    {
    case 0:  // "1B7"
        break;
    case 1:  // "48F"
        craneInfo.mainArmLength = (payload[0] + payload[1] * 256) / 100.0;		//主臂长
        craneInfo.mainArmAngle = (payload[2] + payload[3] * 256) / 10.0;		//臂架顶端倾斜角度，即变幅角
        craneInfo.mainArmToGroundHeight = (payload[6] + payload[7] * 256) / 100.0;			//臂尖对地高度
        break;
    case 2: // "2B7"
        craneInfo.mainHookToGroundHeight = (payload[2] + payload[3] * 256) / 100.0 - 2;		//吊钩对地高度
        break;
    case 3:  // "3B7"
        craneInfo.rotationAngle =(payload[4] + payload[5] * 256) / 100.0;		//回转角度
        break;
    case 4:  // "4B7"
        break;
    case 5:  // "383"
        break;
    } 
    
    publishCraneData();
}


void PCANManager::publishCraneData() {
    // 1. 实例化消息对象
    HookLoadAntiCollisionDetection::CraneStatus msg;

    // 2. 拷贝数据（从你的 CraneInfo 结构体到 ROS 消息）
    msg.mainArmLength           = craneInfo.mainArmLength;
    msg.mainArmAngle            = craneInfo.mainArmAngle;
    msg.subArmLength            = craneInfo.subArmLength;
    msg.subArmAngle             = craneInfo.subArmAngle;
    msg.mainArmToGroundHeight   = craneInfo.mainArmToGroundHeight;
    msg.subArmToGroundHeight    = craneInfo.subArmToGroundHeight;
    msg.rotationAngle           = craneInfo.rotationAngle;
    msg.mainHookToGroundHeight  = craneInfo.mainHookToGroundHeight;
    msg.subHookToGroundHeight   = craneInfo.subHookToGroundHeight;
    msg.armStatus               = craneInfo.armStatus;
    // ... 填充其他字段

    // 3. 发布数据
    crane_pub_.publish(msg);
}

int PCANManager::closeDevice() {
    return CAN_Uninitialize(pcanHandle);
}
#include <HookLoadAntiCollisionDetection/SocketCANManager.h>

SocketCANManager::SocketCANManager(ros::NodeHandle& nh, std::string interface) 
    : nh_(nh), interface_(interface) {
    
    // 初始化发布者
    crane_pub_ = nh_.advertise<HookLoadAntiCollisionDetection::CraneStatus>("crane_status", 10);

    // 1. 创建 Socket
    if ((socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("SocketCAN: Error while opening socket");
        return;
    }

    // 2. 指定接口 (can0, can1 等)
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_.c_str());
    ioctl(socket_fd_, SIOCGIFINDEX, &ifr);

    // 3. 绑定 Socket
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("SocketCAN: Error in socket bind");
        return;
    }

    ROS_INFO("SocketCAN: %s initialized successfully.", interface_.c_str());
}

SocketCANManager::~SocketCANManager() {
    stop();
    if (socket_fd_ >= 0) close(socket_fd_);
}

void SocketCANManager::startReceiveThread() {
    is_running_ = true;
    recv_thread_ = new std::thread(&SocketCANManager::receiveThread, this);
}

void SocketCANManager::stop() {
    is_running_ = false;
    if (recv_thread_ && recv_thread_->joinable()) {
        recv_thread_->join();
    }
}

void SocketCANManager::receiveThread() {
    struct can_frame frame;
    while (ros::ok() && is_running_.load()) {
        int nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            ROS_WARN_THROTTLE(5, "SocketCAN: Read error or timeout");
            continue;
        }
        if (nbytes < sizeof(struct can_frame)) {
            ROS_WARN("SocketCAN: Incomplete CAN frame");
            continue;
        }

        // 解析并发布
        processFrame(frame);
    }
}

void SocketCANManager::processFrame(const struct can_frame& frame) {
    // 这里复用你之前的解析逻辑 (switch case)
    // 注意：SocketCAN 的 ID 存储在 frame.can_id 中，数据在 frame.data 中
    
    std::vector<int> payload(frame.can_dlc);
    for(int i = 0; i < frame.can_dlc; i++) payload[i] = frame.data[i];

    // 模拟之前 updateInfo 的逻辑
    if (frame.can_id == 0x391) {
        HookLoadAntiCollisionDetection::CraneStatus msg;
        // 填充数据逻辑...
        crane_pub_.publish(msg);
    }
}
#include <QObject>
#include <QMutex>
#include <QThread>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <HookLoadAntiCollisionDetection/processbackend.h>
#include <HookLoadAntiCollisionDetection/common.h>
#include <HookLoadAntiCollisionDetection/CraneStatus.h>

using CollisionResult = CollisionDetResult<pcl::PointXYZI>;
using CloudVisVector = std::vector<std::shared_ptr<CloudVisInfo<pcl::PointXYZI>>>;

class HookLoadAntiCollisionDetector : public QObject {
    Q_OBJECT
public:
    explicit HookLoadAntiCollisionDetector(QObject *parent = nullptr) : QObject(parent) {}
    void stop() { m_stopFlag = true; }
    std::shared_ptr<SharedSyncContext<pcl::PointXYZI>> ctx;

private:
    std::atomic<bool> m_stopFlag{false};
    
public slots:
    void process() {
        // --- 1. 初始化 ROS ---
        int argc = 0;
        char** argv = nullptr;
        // ros::init(argc, argv, "qt_collision_node");
        ros::NodeHandle nh("~");

        // --- 2. 初始化后端 (引用你之前的代码) ---
        ctx = std::make_shared<SharedSyncContext<pcl::PointXYZI>>();

        // 获取目标 ROS 包的绝对路径
        std::string package_path = ros::package::getPath("HookLoadAntiCollisionDetection"); // 替换为那个包的真实名称

        if (package_path.empty()) {
            ROS_ERROR("Could not find the package: other_package_name");
            return;
        }

        // 拼接处完整的配置文件路径
        std::string config_path = package_path + "/config.yaml";
        if(!ctx->fixedParams_.setFixedParams(config_path)) return;
        ProcessBackEnd<pcl::PointXYZI> node(nh, ctx); //实例化后自动开启处理线程

        // --- 3. ROS 订阅总线数据更新线程，替换成从总线线程直接获取 ---
        // ros::Subscriber sub = nh.subscribe<HookLoadAntiCollisionDetection::CraneStatus>(
        //     "crane_info", 10, boost::bind(&statusCallback, _1, ctx));
        
        // ros::AsyncSpinner spinner(1);
        // spinner.start();

        uint64_t last_frame = 0;

        // --- 4. 算法循环 ---
        while (ros::ok() && !m_stopFlag) {
            {
                std::unique_lock<std::mutex> lock(ctx->dataMutex_);
                ctx->cv_.wait(lock, [&] { return ctx->frame_id_ != last_frame; });
                
                // 将计算结果拷贝到 UI 可访问的安全区域（或直接通过信号发送关键结果）
                CollisionDetResult<pcl::PointXYZI> res = ctx->collisionDetResult_;
                auto cloud_info = ctx->CloudVisInfo_;
                
                // 发送结果信号给界面
                emit dataUpdated(res, cloud_info); 
                last_frame = ctx->frame_id_;
            }
            // 避免抢占过多 CPU
            QThread::msleep(10); 
        }
    }

signals:
    // 定义信号，传递检测结果和点云
    void dataUpdated(CollisionResult result, 
                    CloudVisVector visInfo);
};
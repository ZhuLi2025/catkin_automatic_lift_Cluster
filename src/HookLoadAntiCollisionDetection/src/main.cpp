#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <HookLoadAntiCollisionDetection/processbackend.h>
#include <HookLoadAntiCollisionDetection/common.h>
#include <HookLoadAntiCollisionDetection/CraneStatus.h>

// --- 修改点 1: 移除回调函数内部的 while 循环，使其成为真正的回调 ---
void statusCallback(const HookLoadAntiCollisionDetection::CraneStatus::ConstPtr& msg, 
                    std::shared_ptr<SharedSyncContext<pcl::PointXYZI>> ctx) 
{
    bool debug = true; // 建议将 debug 设为成员变量或全局参数
    
    // 加锁更新数据
    std::unique_lock<std::mutex> lock(ctx->dataMutex_);
    
    if (debug) {
        ctx->busData_.rope_len_ = 63.0;
        ctx->busData_.load_exist_flag_ = false;
        ctx->busData_.vx_ = 0;
        ctx->busData_.vy_ = 3;
        ctx->busData_.vz_ = 0;
    } else {
        // 计算绳长：主臂高度 - 吊钩高度
        ctx->busData_.rope_len_ = msg->mainArmToGroundHeight - msg->mainHookToGroundHeight;
        ctx->busData_.load_exist_flag_ = false;
        ctx->busData_.vx_ = 0;
        ctx->busData_.vy_ = 3;
        ctx->busData_.vz_ = 0;
    }
    
    // 这里不需要 wait，因为每当话题有新消息，此函数就会被调用
    // 如果需要通知其他线程数据已更新，可以调用：
    // ctx->cv_.notify_all(); 
}

int main(int argc,char** argv){
    ros::init(argc,argv,"processbackend");
    ros::NodeHandle nh("~");
    std::string config_file_path;
    nh.param<std::string>("config_file_path",config_file_path,std::string(CONFIG_FILE_DIR)+"/config.yaml");

//============================== 处理后端初始化 ==========================================
    auto ctx = std::make_shared<SharedSyncContext<pcl::PointXYZI>>();
    if(!ctx->fixedParams_.setFixedParams(config_file_path)){return 0;}
    ProcessBackEnd<pcl::PointXYZI> node(nh,ctx);

//============================== 开启总线数据接收线程 ==========================================
    // 使用 boost::bind 将 ctx 作为参数传递给回调函数
    ros::Subscriber sub_ = nh.subscribe<HookLoadAntiCollisionDetection::CraneStatus>(
        "crane_info", 
        10, 
        boost::bind(&statusCallback, _1, ctx)
    );
    
    ros::AsyncSpinner spinner(1); 
    spinner.start();

// ================================= visualization ========================================
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    // viewer->initCameraParameters();
    uint64_t last_frame{0};


    std::vector<std::shared_ptr<CloudVisInfo<pcl::PointXYZI>>> local_vis;//场景+目标 点云
    CollisionDetResult<pcl::PointXYZI> collisionDetResult;
    while (ros::ok() && !viewer->wasStopped())
    {
        {
            std::unique_lock<std::mutex> lock(ctx->dataMutex_);
            ctx->cv_.wait(lock, [&] {
                return ctx->frame_id_ != last_frame;
            });
            local_vis = ctx->CloudVisInfo_;  // ⚠️ 只拷贝 vector + shared_ptr
            last_frame = ctx->frame_id_;
            collisionDetResult = ctx->collisionDetResult_;
        } // 立刻释放锁
        for (const auto& vis_ptr : local_vis)
        {
            //-------------------------------- 绘制点云 --------------------------------------------
            const auto& vis = *vis_ptr;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>color(vis.cloud_, vis.r_, vis.g_, vis.b_);
            if (!color.isCapable()) {
                std::cerr << "ColorHandler not capable for " << vis.cloudID_ << std::endl;
            }
            if (!viewer->updatePointCloud(vis.cloud_, color, vis.cloudID_))
            {
                viewer->addPointCloud(vis.cloud_, color, vis.cloudID_);
            }

            //--------------------------------- 绘制直线 -------------------------------------------
            pcl::PointXYZI start_point = collisionDetResult.nearest_point_;
            pcl::PointXYZI end_point;
            end_point.x = start_point.x - collisionDetResult.dir_[0] * collisionDetResult.minDistance_;
            end_point.y = start_point.y - collisionDetResult.dir_[1] * collisionDetResult.minDistance_;
            end_point.z = start_point.z - collisionDetResult.dir_[2] * collisionDetResult.minDistance_;

            // 在可视化器中添加直线
            viewer->removeShape("minDistance");
            viewer->addLine(start_point, end_point, 1.0, 0.0, 0.0, "minDistance");  // Red color
            
        }

        viewer->spinOnce(10);
    }

    ros::shutdown();
    viewer->close();

    return 0;
}


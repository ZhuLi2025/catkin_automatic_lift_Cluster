#pragma once
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <HookLoadAntiCollisionDetection/common.h>
#include <HookLoadAntiCollisionDetection/pipeline.h>

/**
*@class ProcessBackEnd:点云处理
*@brief 点云采集、累积、点云处理
*/
template <typename PointT>
class ProcessBackEnd
{
public:
    explicit ProcessBackEnd(ros::NodeHandle& nh,std::shared_ptr<SharedSyncContext<PointT>> ctx);
    ~ProcessBackEnd();
private:

    void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msgs); // 点云回调
    void processLoop(); //处理线程
    void accumCloud(const typename pcl::PointCloud<PointT>::Ptr& new_cloud); //点云累计

//成员变量
    ros::Subscriber pc_sub_;
    ros::Publisher pub_;
    std::thread preprocess_thread_;
    int accum_frames_{5},frame_rate_{10},frame_count_{0};
    std::mutex pc_mtx_;
    std::condition_variable pc_cv_;
#ifdef ROSBAG_TEST
    std::deque<sensor_msgs::PointCloud2::ConstPtr> pc_buf_;
#else
    std::deque<typename pcl::PointCloud<PointT>::Ptr> pc_buf_;
#endif
    std::string subscrib_topic_,advertise_topic_;

    //流水线
    std::shared_ptr<Pipeline<PointT>> pipeline_;

    //上下文
    std::shared_ptr<SharedSyncContext<PointT>> ctx_;

};
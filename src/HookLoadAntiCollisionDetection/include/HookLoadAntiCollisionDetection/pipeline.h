#pragma once
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include <thread>
#include <mutex>
#include <condition_variable>
// #include "constants.h"
#include <HookLoadAntiCollisionDetection/HookLoadPositionAcquirer.h>
#include <HookLoadAntiCollisionDetection/ICPTracker.h>
#include <HookLoadAntiCollisionDetection/CollisionDetector.h>
#include <HookLoadAntiCollisionDetection/CSF/CSF.h>
#include <HookLoadAntiCollisionDetection/utils.h>

template <typename PointT>
class Pipeline{
    enum Mode{
        DetectMode,
        TrackMode
    };
    public:
        explicit Pipeline(std::shared_ptr<SharedSyncContext<PointT>> ctx);
        bool forward(typename pcl::PointCloud<PointT>::Ptr& cloud);   
        void modeReset(){
            target_det_mode_ =  Mode::DetectMode;
        }
        ~Pipeline();
    private:
        void reset();
        bool tryDetectTarget(typename pcl::PointCloud<PointT>::Ptr& cloud) ;
        void updateVisInfo();

        bool filteringOutGroundPC(const typename pcl::PointCloud<PointT>::Ptr&  ,typename pcl::PointCloud<PointT>::Ptr& );
        //成员变量
        std::shared_ptr<SharedSyncContext<PointT>> ctx_;

        //创建吊钩吊载位置获取类对象
        std::shared_ptr<HookLoadPositionAcquirer<PointT>> hookLoadPositionAcquirer_;

        //体素滤波
        pcl::VoxelGrid<PointT> sor_;

        //中间变量
        ClusterInfo<PointT> targetClusterInfo_last_frame_;
        ClusterInfo<PointT> targetClusterInfo_cur_frame_;
        ClusterInfo<PointT> hookClusterInfo_;
        ClusterInfo<PointT> loadClusterInfo_;
        typename pcl::PointCloud<PointT>::Ptr ROI_cloud_;
        typename pcl::PointCloud<PointT>::Ptr filtered_cloud_;

        //基于ipc的帧间点云匹配实现目标点云跟踪
        std::shared_ptr<ICPTracker<PointT>> icpTrack_;

        //创建碰撞检测类对象
        std::shared_ptr<CollisionDetector<PointT>> collisionDetector_;

        //检测/跟踪模式状态量
        Mode target_det_mode_{Mode::DetectMode};

        //地面点云过滤器
        std::shared_ptr<CSF> csf_;
};
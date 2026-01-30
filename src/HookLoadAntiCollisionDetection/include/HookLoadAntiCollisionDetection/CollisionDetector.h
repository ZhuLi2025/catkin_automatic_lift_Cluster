#pragma once  // 推荐写在文件最开头
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <Eigen/Dense>
#include <mutex>
#include <omp.h>
#include <HookLoadAntiCollisionDetection/utils.h>
template<typename PointT> 
class CollisionDetector{

    public:

        CollisionDetector();
        ~CollisionDetector();

        bool EnvCloudGeometryConstruct(const typename pcl::PointCloud<PointT>::Ptr& load , std::vector<OBB<PointT>>& target_OBB);

        bool VirtualHookGeometryConstruct();

        bool computeMinDistance(    const typename pcl::PointCloud<PointT>::ConstPtr& env_cloud,
                                    const OBB<PointT>& obb,
                                    CollisionDetResult<PointT>&  collision_det_result,
                                    float sample_resolution = 0.1f,
                                    int threads = 0);
        
        void getEnvCloud(   typename pcl::PointCloud<PointT>::Ptr source,
                            const FixedParams& config,
                            const BusData& busdata,
                            const Eigen::Vector3f& centerPoint,
                            typename pcl::PointCloud<PointT>::Ptr env_cloud);

        void getEnvCloud(   typename pcl::PointCloud<PointT>::Ptr source,
                            const FixedParams& config,
                            const BusData& busdata,
                            const OBB<PointT>& hookloadobb,
                            typename pcl::PointCloud<PointT>::Ptr env_cloud);

        bool computeMinDistance(    const typename pcl::PointCloud<PointT>::ConstPtr& env_cloud,
                                    const Eigen::Vector3f& hook_position,
                                    CollisionDetResult<PointT>&  collision_det_result);

        std::vector<Eigen::Vector3f> sampleOBBSurface(const OBB<PointT>& obb, float resolution=0.2f);

    private:
        OBB<PointT> target_OBB;
        std::vector<OBB<PointT>> env_OBB;

        inline Eigen::Vector3f localToWorld(const OBB<PointT>& obb, const Eigen::Vector3f& local_pt);

};
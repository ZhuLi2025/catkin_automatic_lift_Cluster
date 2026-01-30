#pragma once  // 推荐写在文件最开头
#include <ros/ros.h>
#include <vector>
#include <pcl/point_cloud.h>       // pcl::PointCloud
#include <pcl/point_types.h>       // PointT
#include <pcl/PointIndices.h>      // pcl::PointIndices
#include <pcl/kdtree/kdtree_flann.h> // pcl::KdTreeFLANN
#include <pcl/common/common.h>     // pcl::isFinite
#include <pcl/common/point_tests.h>     // pcl::isFinite
#include <pcl/features/moment_of_inertia_estimation.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <HookLoadAntiCollisionDetection/common.h>
// ========== 定义 OBB 结构 ==========
template <typename PointT>
struct OBB {
    Eigen::Vector3f min_point;
    Eigen::Vector3f max_point;
    Eigen::Vector3f position;
    Eigen::Matrix3f rotation;

    OBB() 
        : min_point(Eigen::Vector3f::Zero()),
        max_point(Eigen::Vector3f::Zero()),
        position(Eigen::Vector3f::Zero()),
        rotation(Eigen::Matrix3f::Identity()) {}

    // ========== 计算点云的 OBB ==========
    OBB(const typename pcl::PointCloud<PointT>::Ptr& cloud){
        pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();

        PointT min_point_OBB, max_point_OBB, position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        min_point = Eigen::Vector3f(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        max_point = Eigen::Vector3f(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        position  = Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);
        rotation  = rotational_matrix_OBB;
    }
};

// ========== 定义 点簇 结构 ==========
template <typename PointT>
struct ClusterInfo {
    int id;
    typename pcl::PointCloud<PointT>::Ptr cloud;
    Eigen::Vector3f centroid;
    float distance;  // 质心到直线的距离
    std::vector<int> indices;
    OBB<PointT> obb; // 新增：该簇对应的 OBB

    // 默认构造函数
    ClusterInfo()
        : id(-1),
          cloud(new pcl::PointCloud<PointT>),
          centroid(Eigen::Vector3f::Zero()),
          distance(-1.0f),
          obb() {}   // 默认初始化为空 OBB


    // ========== 深拷贝构造函数 ==========
    ClusterInfo(const ClusterInfo& other)
        : id(other.id),
          cloud(new pcl::PointCloud<PointT>(*other.cloud)), // 复制点云内容
          centroid(other.centroid),
          distance(other.distance),
          indices(other.indices),
          obb(other.obb) {}

    // ========== 深拷贝赋值运算符 ==========
    ClusterInfo& operator=(const ClusterInfo& other) {
        if (this != &other) {
            id = other.id;
            // 正确管理智能指针
            if (other.cloud) {
                cloud.reset(new pcl::PointCloud<PointT>(*other.cloud));
            } else {
                cloud.reset();
            }
            centroid = other.centroid;
            distance = other.distance;
            indices = other.indices;
            obb = other.obb;
        }
        return *this;
    }

    bool exist() const {
        return cloud&&!cloud->empty();
    }

    void clearAll() {
        id = -1;
        if(cloud) cloud->clear();
        centroid = Eigen::Vector3f::Zero();
        distance = -1.0f;
        obb = OBB<PointT>(); // 重置 OBB
    }

    void clearCloud() {
        if(cloud) cloud->clear();
        obb = OBB<PointT>(); // 清空时也同步重置 OBB
    }

    // 计算并更新 OBB
    void computeOBB() {
        if (cloud && !cloud->empty()) {
            obb = OBB<PointT>(cloud);
        }
    }
};



/**
 * @brief 根据目标点云（target）与输入点云（cloud）之间的距离，
 *        聚类出所有与目标距离小于阈值的点索引。
 * 
 * @param[in]  cloud               输入点云（待检测）
 * @param[in]  target              目标点云（作为参考区域）
 * @param[in]  distance_threshold  距离阈值（米）
 * @param[out] inliers_out         输出符合距离条件的索引（相对于 cloud）
 * @return bool                    是否找到符合条件的点（true=找到，false=无）
 * 
 * @note
 *  - 使用 KD-tree 最近邻搜索；
 *  - 若目标或输入点云为空，或阈值无效，则直接返回 false；
 *  - 若启用 OpenMP，可实现加速；
 * 
 * @example
 *  pcl::PointIndices::Ptr indices;
 *  bool ok = clusterPointCloudByTargetDistance(cloud, target, 0.5f, indices);
 */
inline bool clusterPointCloudByTargetDistance(
    const  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const  pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    float distance_threshold,
    pcl::PointIndices::Ptr& inliers_out)
{
    // -------------------------
    // 输入有效性检查
    // -------------------------
    if (!cloud || cloud->empty() || !target || target->empty() || distance_threshold <= 0.0f) {
        inliers_out.reset(new pcl::PointIndices);
        return false;
    }

    try {
        // -------------------------
        // 初始化 KD-tree
        // -------------------------
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target);

        inliers_out.reset(new pcl::PointIndices);
        inliers_out->indices.reserve(cloud->size());

        std::vector<int> nn_indices(1);
        std::vector<float> nn_sqr_dists(1);
        const float thr2 = distance_threshold * distance_threshold;

        // -------------------------
        // 最近邻搜索
        // -------------------------
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto& pt = cloud->points[i];
            if (!pcl::isFinite(pt))
                continue; // 跳过无效点

            if (kdtree.nearestKSearch(pt, 1, nn_indices, nn_sqr_dists) > 0)
            {
                if (nn_sqr_dists[0] <= thr2)
                    inliers_out->indices.push_back(static_cast<int>(i));
            }
        }

        return !inliers_out->indices.empty();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[clusterPointCloudByTargetDistance] Exception: " << e.what() << std::endl;
        inliers_out.reset(new pcl::PointIndices);
        return false;
    }
};

/**
 * @brief 根据目标点云（target）与输入点云（cloud）之间的距离，
 *        聚类出所有与目标距离小于阈值的点索引。
 * 
 * @param[in]  cloud               输入点云（待检测）
 * @param[in]  target              目标点云（作为参考区域）
 * @param[in]  distance_threshold  距离阈值（米）
 * @param[out] inliers_out         输出符合距离条件的索引（相对于 cloud）
 * @return bool                    是否找到符合条件的点（true=找到，false=无）
 * 
 * @note
 *  - 使用 KD-tree 最近邻搜索；
 *  - 若目标或输入点云为空，或阈值无效，则直接返回 false；
 *  - 若启用 OpenMP，可实现加速；
 * 
 * @example
 *  pcl::PointIndices::Ptr indices;
 *  bool ok = clusterPointCloudByTargetDistance(cloud, target, 0.5f, indices);
 */
inline bool clusterPointCloudByTargetDistance(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    float distance_threshold,
    pcl::PointIndices::Ptr& inliers_out)
{
    // -------------------------
    // 输入有效性检查
    // -------------------------
    if (!cloud || cloud->empty() || !target || target->empty() || distance_threshold <= 0.0f) {
        inliers_out.reset(new pcl::PointIndices);
        return false;
    }

    try {
        // -------------------------
        // 初始化 KD-tree
        // -------------------------
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(target);

        inliers_out.reset(new pcl::PointIndices);
        inliers_out->indices.reserve(cloud->size());

        std::vector<int> nn_indices(1);
        std::vector<float> nn_sqr_dists(1);
        const float thr2 = distance_threshold * distance_threshold;

        // -------------------------
        // 最近邻搜索
        // -------------------------
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto& pt = cloud->points[i];
            if (!pcl::isFinite(pt))
                continue; // 跳过无效点

            if (kdtree.nearestKSearch(pt, 1, nn_indices, nn_sqr_dists) > 0)
            {
                if (nn_sqr_dists[0] <= thr2)
                    inliers_out->indices.push_back(static_cast<int>(i));
            }
        }

        return !inliers_out->indices.empty();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[clusterPointCloudByTargetDistance] Exception: " << e.what() << std::endl;
        inliers_out.reset(new pcl::PointIndices);
        return false;
    }
};


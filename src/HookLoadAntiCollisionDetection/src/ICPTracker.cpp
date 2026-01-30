#include <HookLoadAntiCollisionDetection/ICPTracker.h>
template<typename PointT>
ICPTracker<PointT>::ICPTracker(){}

template<typename PointT>
ICPTracker<PointT>::~ICPTracker(){}

template<typename PointT>
float ICPTracker<PointT>::computeOverlap(
    const typename pcl::PointCloud<PointT>::Ptr& aligned,  // ICP后对齐的模板点云
    const typename pcl::PointCloud<PointT>::Ptr& source,   // 当前帧点云
    float dist_thresh ) // 阈值，取决于点云分辨率（这里假设 20cm）
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(source);

    int matched = 0;
    for (const auto& p : aligned->points)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_dist(1);
        if (kdtree.nearestKSearch(p, 1, indices, sqr_dist) > 0)
        {
            if (sqr_dist[0] < dist_thresh * dist_thresh)
                matched++;
        }
    }
    return static_cast<float>(matched) / static_cast<float>(aligned->size());
}

/**
 * @brief   提取目标（吊钩/吊载）在当前帧中的点云，并生成安全区域环境点云。
 *
 * @tparam  PointT 点云类型（如 pcl::PointXYZI）。
 * @param   source                当前帧完整点云。
 * @param   aligned               ICP配准后的目标模板点云（已对齐到source坐标系）。
 * @param   target_in_source      输出：当前帧中对应目标点云及质心信息。
 * @param   safe_region_env_cloud 输出：剔除目标及车体后用于碰撞检测的环境点云。
 * 
 * @details
 * 1. 计算目标包围盒（BBox），裁剪出目标区域点云；
 * 2. 根据距离聚类得到吊钩/吊载点；
 * 3. 从原始点云中剔除目标与车体点云；
 * 4. 提取目标附近一定范围的安全检测区域。
 * 
 * @note
 * - 输入点云需为同一坐标系；
 * - 输出安全区用于后续障碍物检测；
 * - 过程包含异常捕获，确保鲁棒性。
 */
template<typename PointT> 
bool  ICPTracker<PointT>::getTargetCluster( 
    typename pcl::PointCloud<PointT>::Ptr& source,
    const typename pcl::PointCloud<PointT>::Ptr& aligned,
    ClusterInfo<PointT>& target_in_source)
{
    try
    {
        // ---------- Step 0: 输入检查 ----------
        if (!source || source->empty())
        {
            ROS_WARN("[=== TRACK ===] Source cloud is empty or null.");
            return false;
        }
        if (!aligned || aligned->empty())
        {
            ROS_WARN("[=== TRACK ===] Aligned cloud is empty or null.");
            return false;
        }

        // ---------- Step 1: 提取目标点云区域 ----------
        typename pcl::PointCloud<PointT>::Ptr hook_region(new pcl::PointCloud<PointT>);

        // 1.1 计算 aligned 点云包围盒（扩大2m以确保包含目标）
        Eigen::Vector4f bbMin( std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(), 1.0f );
        Eigen::Vector4f bbMax( std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(),
                               std::numeric_limits<float>::lowest(), 1.0f );

        for (const auto& point : aligned->points)
        {
            bbMin.head<3>() = bbMin.head<3>().cwiseMin(Eigen::Vector3f(point.x, point.y, point.z));
            bbMax.head<3>() = bbMax.head<3>().cwiseMax(Eigen::Vector3f(point.x, point.y, point.z));
        }
        bbMin = bbMin.array() - 2.0f;
        bbMax = bbMax.array() + 2.0f;

        // 1.2 裁剪当前帧source中包围盒区域
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(source);
        crop.setMin(bbMin);
        crop.setMax(bbMax);
        std::vector<int> region_indices;
        crop.filter(region_indices);

        if (region_indices.empty())
        {
            ROS_WARN("[=== TRACK ===] FAILED . CropBox returned empty region.");
            return false;
        }

        pcl::ExtractIndices<PointT> extract;
        pcl::PointIndices::Ptr region_point_indices(new pcl::PointIndices);
        region_point_indices->indices = region_indices;
        extract.setInputCloud(source);
        extract.setIndices(region_point_indices);
        extract.setNegative(false);
        extract.filter(*hook_region);

        // 1.3 根据距离聚类分离吊钩/吊载点
        constexpr float distance_threshold = 1.0f;
        pcl::PointIndices::Ptr inliers_local(new pcl::PointIndices);
        clusterPointCloudByTargetDistance(hook_region, aligned, distance_threshold,inliers_local);

        pcl::PointIndices::Ptr inliers_in_source(new pcl::PointIndices);
        for (int idx : inliers_local->indices)
        {
            if (idx >= 0 && idx < static_cast<int>(region_indices.size()))
                inliers_in_source->indices.push_back(region_indices[idx]);
        }

        extract.setIndices(inliers_in_source);
        extract.setNegative(false);
        extract.filter(*target_in_source.cloud);
        return true;
        // // ---------- Step 2: 剔除吊钩吊载与车体 ----------
        // typename pcl::PointCloud<PointT>::Ptr env_cloud(new pcl::PointCloud<PointT>);
        // extract.setNegative(true);
        // extract.filter(*env_cloud);

        // // 2.1 剔除车体区域点云（通过y、z范围过滤）
        // pcl::PassThrough<PointT> pass_y;
        // pass_y.setInputCloud(env_cloud);
        // pass_y.setFilterFieldName("y");
        // pass_y.setFilterLimits(0.0f, 5.0f);
        // pass_y.setNegative(true);
        // std::vector<int> indices_y;
        // pass_y.filter(indices_y);

        // pcl::PassThrough<PointT> pass_z;
        // pass_z.setInputCloud(env_cloud);
        // pass_z.setFilterFieldName("z");
        // pass_z.setFilterLimits(-5.0f, 100.0f);
        // pass_z.setNegative(true);
        // std::vector<int> indices_z;
        // pass_z.filter(indices_z);

        // std::sort(indices_y.begin(), indices_y.end());
        // std::sort(indices_z.begin(), indices_z.end());

        // std::vector<int> union_indices;
        // std::set_union(indices_y.begin(), indices_y.end(),
        //                indices_z.begin(), indices_z.end(),
        //                std::back_inserter(union_indices));

        // pcl::PointIndices::Ptr crane_indices(new pcl::PointIndices);
        // crane_indices->indices = union_indices;

        // extract.setInputCloud(env_cloud);
        // extract.setIndices(crane_indices);
        // extract.setNegative(false);
        // extract.filter(*env_cloud);

        // // ---------- Step 3: 筛选安全区点云 ----------
        // Eigen::Vector4f centroid;
        // pcl::compute3DCentroid(*target_in_source.cloud, centroid);
        // target_in_source.centroid = centroid.head<3>();

        // // 定义检测区域范围（m）
        // constexpr float range_x = 30.0f;
        // constexpr float range_y = 50.0f;
        // constexpr float range_z = 50.0f;

        // Eigen::Vector4f min_point(centroid[0] - range_x / 2,
        //                           centroid[1] - range_y / 2,
        //                           centroid[2] - range_z / 2, 1.0f);
        // Eigen::Vector4f max_point(centroid[0] + range_x / 2,
        //                           centroid[1] + range_y / 2,
        //                           centroid[2] + range_z / 2, 1.0f);

        // pcl::CropBox<PointT> crop_box;
        // crop_box.setInputCloud(env_cloud);
        // crop_box.setMin(min_point);
        // crop_box.setMax(max_point);
        // crop_box.setNegative(false);
        // crop_box.filter(*safe_region_env_cloud);
        // return true;
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== TRACK ===] PCLException: %s", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[=== TRACK ===] std::exception: %s", e.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR("[=== TRACK ===] Unknown exception caught.");
        return false;
    }
}

/**
 * @brief   使用ICP算法在当前帧中追踪目标点云位置。
 * 
 * @tparam  PointT 点云数据类型（如 pcl::PointXYZ 或 pcl::PointXYZI）。
 * @param   source              当前帧点云。
 * @param   target              模板点云（上一帧识别出的吊钩或吊载）。
 * @param   target_in_source    ICP对齐后的模板点云。
 * 
 * @return  true    ICP匹配成功并且结果可信；
 * @return  false   ICP未收敛、得分过高或发生异常。
 * 
 * @note    
 * - 函数内部包含异常捕获与日志输出，避免崩溃；
 * - 使用VoxelGrid降采样加速；
 * - 收敛判断同时考虑 score 与 overlap。
 */
template<typename PointT>
bool ICPTracker<PointT>::targetTrack(
    typename pcl::PointCloud<PointT>::Ptr& source,   
    ClusterInfo<PointT>& target,    
    ClusterInfo<PointT>& target_in_source)  
{
    try{
        // ---------------- 1: 输入合法性检测 ----------------
        if(!source||source->empty()){
            ROS_WARN("[=== TRACK ===] input cloud is empty or null ");
            return false;
        }
        if (!target.cloud || target.cloud->empty()) {
            ROS_WARN("[=== TRACK ===] target cloud is empty or null");
            return false;
        }

        // // ---------------- 2. 降采样（减少点数，提高ICP速度） ----------------
        // pcl::VoxelGrid<PointT> voxel_filter;
        // voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // 根据实际场景调整
        // typename pcl::PointCloud<PointT>::Ptr src_ds(new pcl::PointCloud<PointT>);
        // typename pcl::PointCloud<PointT>::Ptr tgt_ds(new pcl::PointCloud<PointT>);
        // voxel_filter.setInputCloud(source);
        // voxel_filter.filter(*src_ds);
        // voxel_filter.setInputCloud(target.cloud);
        // voxel_filter.filter(*tgt_ds);
        // if (tgt_ds->empty() || src_ds->empty())
        // {
        //     ROS_WARN("[=== TRACK ===] Downsampled cloud empty after VoxelGrid.");
        //     return false;
        // }

        // ---------------- 3. ICP 匹配----------------
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(target.cloud);
        icp.setInputTarget(source);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        icp.setSearchMethodSource(tree);
        icp.setSearchMethodTarget(tree);
        icp.setMaximumIterations(100);           // 最大迭代次数（减少过度计算）
        icp.setTransformationEpsilon(1e-6);     // 收敛判据（变换矩阵收敛）
        icp.setEuclideanFitnessEpsilon(1e-6);   // 收敛判据（误差收敛）
        typename pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>() );
        icp.align(*aligned,last_transform);

        // ---------------- 4. 检查匹配结果----------------
        if (!icp.hasConverged()) {
            ROS_WARN("[=== TRACK ===] FAILED . ICP did not converge.");
            return false;
        }

        float score = icp.getFitnessScore();
        float overlap =computeOverlap(aligned, source, 0.2f);
        if (score < 0.1 && overlap > 0.6f) { // 双条件约束
            last_transform = icp.getFinalTransformation();
            if(getTargetCluster(source, aligned,target_in_source)){
                ROS_INFO("[=== TRACK ===] SUCCESSFUL . score= %f , overlap= %f , num target points=%d ",
                            score,overlap,(int)target_in_source.cloud->points.size());
                return true;
            }
            else{
                return false;
            }
        }
        
        ROS_WARN("[=== TRACK ===] FAILED . overlap too low.");
        return false;

    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== TRACK ===] PCLException: %s", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[=== TRACK ===] std::exception: %s", e.what());
        return false;
    }
    catch (...)
    {
        ROS_ERROR("[=== TRACK ===] Unknown exception caught.");
        return false;
    }
}

template class ICPTracker<pcl::PointXYZ>;
template class ICPTracker<pcl::PointXYZI>;
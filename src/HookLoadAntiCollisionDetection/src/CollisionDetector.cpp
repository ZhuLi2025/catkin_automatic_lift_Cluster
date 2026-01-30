#include <HookLoadAntiCollisionDetection/CollisionDetector.h>

template <typename PointT>
CollisionDetector<PointT>::CollisionDetector(){

}
template <typename PointT>
CollisionDetector<PointT>::~CollisionDetector(){
    
}

//获取环境障碍物包围盒
template<typename PointT> 
bool CollisionDetector<PointT>::EnvCloudGeometryConstruct(const typename pcl::PointCloud<PointT>::Ptr& env_cloud , std::vector<OBB<PointT>>& obstaclesOBB){
    //欧式聚类
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(env_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.5);  // 聚类半径50cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(env_cloud);
    ec.extract(cluster_indices);

    std::vector<OBB<PointT>> obb_list;
    int cluster_id = 0;
    if(cluster_indices.size()==0){
        ROS_INFO("[Collision Detecte] No obstacle cluster found.");
        return false;
    }
    else{
        ROS_INFO("[Collision Detecte] Found %d obstacle clusters.",(int)cluster_indices.size());
        for (auto& indices : cluster_indices) {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
            for (int idx : indices.indices) {
                cluster_cloud->push_back((*env_cloud)[idx]);
            }
            OBB<PointT> obb(cluster_cloud);
            obb_list.push_back(obb);
        }
        return true;
    }

}
//获取虚拟吊钩包围盒
template<typename PointT> 
bool CollisionDetector<PointT>::VirtualHookGeometryConstruct(){
}


// 将 OBB 局部坐标系的点转换到世界坐标系
template<typename PointT>
inline Eigen::Vector3f CollisionDetector<PointT>::localToWorld(const OBB<PointT>& obb, const Eigen::Vector3f& local_pt) {
    // world = R * local + position
    return obb.rotation * local_pt + obb.position;
}

// 在 OBB 的表面采样（包含 6 个面），返回世界坐标系下的采样点
// resolution: 采样格距，单位同点云（例如 0.1 = 每 10cm 一个点）
template<typename PointT>
std::vector<Eigen::Vector3f> CollisionDetector<PointT>::sampleOBBSurface(const OBB<PointT>& obb, float resolution)
{
    std::vector<Eigen::Vector3f> samples;
    // 局部坐标下的 min/max
    const Eigen::Vector3f& pmin = obb.min_point;
    const Eigen::Vector3f& pmax = obb.max_point;

    // 面的范围：在局部坐标系下，每个面是两个坐标维度的矩形
    // 我们对 6 个面逐一采样： (x,y) 面 at z=min/max, (x,z) 面 at y=min/max, (y,z) 面 at x=min/max
    auto sampleRect = [&](int dim_fixed, float fixed_val, int dim_a, int dim_b) {
        float a_min = (dim_a==0? pmin.x() : (dim_a==1? pmin.y() : pmin.z()));
        float a_max = (dim_a==0? pmax.x() : (dim_a==1? pmax.y() : pmax.z()));
        float b_min = (dim_b==0? pmin.x() : (dim_b==1? pmin.y() : pmin.z()));
        float b_max = (dim_b==0? pmax.x() : (dim_b==1? pmax.y() : pmax.z()));

        // 保证至少有一个采样点在每个方向（防止分母为0）
        int na = std::max(1, static_cast<int>(std::ceil((a_max - a_min) / resolution)));
        int nb = std::max(1, static_cast<int>(std::ceil((b_max - b_min) / resolution)));

        for (int ia = 0; ia <= na; ++ia) {
            float a = a_min + (a_max - a_min) * (static_cast<float>(ia) / static_cast<float>(na));
            for (int ib = 0; ib <= nb; ++ib) {
                float b = b_min + (b_max - b_min) * (static_cast<float>(ib) / static_cast<float>(nb));
                Eigen::Vector3f local_pt;
                // fill by dimension
                for (int d = 0; d < 3; ++d) {
                    if (d == dim_fixed) {
                        local_pt[d] = fixed_val;
                    } else if (d == dim_a) {
                        local_pt[d] = a;
                    } else { // dim_b
                        local_pt[d] = b;
                    }
                }
                samples.push_back(localToWorld(obb, local_pt));
            }
        }
    };

    // faces: z = min, z = max  (dim_fixed = 2, dim_a=0, dim_b=1)
    sampleRect(2, pmin.z(), 0, 1);
    sampleRect(2, pmax.z(), 0, 1);
    // faces: y = min, y = max  (dim_fixed = 1, dim_a=0, dim_b=2)
    sampleRect(1, pmin.y(), 0, 2);
    sampleRect(1, pmax.y(), 0, 2);
    // faces: x = min, x = max  (dim_fixed = 0, dim_a=1, dim_b=2)
    sampleRect(0, pmin.x(), 1, 2);
    sampleRect(0, pmax.x(), 1, 2);

    // optional: 去重（相同角/边在多个面可能重复采样）——简单去重（基于坐标）
    std::vector<Eigen::Vector3f> uniq;
    uniq.reserve(samples.size());
    const float eps = 1e-6f;
    for (const auto& p : samples) {
        bool found = false;
        for (const auto& q : uniq) {
            if ((p - q).squaredNorm() < eps) { found = true; break; }
        }
        if (!found) uniq.push_back(p);
    }
    return uniq;
}


/**
 * @brief 计算 OBB 表面到环境点云的最近距离。
 *
 * @tparam PointT         点云类型（如 pcl::PointXYZ）
 * @param env_cloud        输入环境点云（已滤波、裁剪）
 * @param obb              输入目标 OBB
 * @param[out] collision_det_result 碰撞检测包含的所有结果信息：最小距离（单位：m）；最小距离对应的障碍点坐标；方向向量
 * @param sample_resolution OBB 表面采样间距（m）
 * @param threads           并行线程数（<=0 表示自动）
 * @return bool             true 表示计算成功，false 表示失败（如点云为空或异常）
 */
template<typename PointT>
bool CollisionDetector<PointT>::computeMinDistance(
    const typename pcl::PointCloud<PointT>::ConstPtr& env_cloud,
    const OBB<PointT>& obb,
    CollisionDetResult<PointT>& collision_det_result,
    float sample_resolution,
    int threads)
{
    collision_det_result.minDistance_ = std::numeric_limits<float>::infinity();
    collision_det_result.dir_ = Eigen::Vector3f::Zero();

    try {
        // 输入检查
        if (!env_cloud || env_cloud->empty()) {
            ROS_WARN("[=== COLLISION DET ===] FAILED .input cloud is null or empty");
            return false;
        }

        // 1) 生成采样点（世界坐标）
        std::vector<Eigen::Vector3f> samples = sampleOBBSurface(obb, sample_resolution);
        if (samples.empty()) {
            ROS_WARN("[=== COLLISION DET ===] FAILED .sample is  empty");
            return false;}

        float global_min_dist2 = std::numeric_limits<float>::infinity();
        Eigen::Vector3f global_min_dir = Eigen::Vector3f::Zero();
        PointT global_min_point ;

#ifdef USE_OPENMP
        if (threads > 0) omp_set_num_threads(threads);
#endif

#pragma omp parallel
        {
            // ===== 每线程私有数据 =====
            pcl::KdTreeFLANN<PointT> local_kdtree;
            local_kdtree.setInputCloud(env_cloud);

            float local_min_dist2 = std::numeric_limits<float>::infinity();
            Eigen::Vector3f local_min_dir = Eigen::Vector3f::Zero();
            PointT local_min_point;   // 每线程自己的最近点

#pragma omp for schedule(static)
            for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
                const Eigen::Vector3f& sp = samples[i];
                PointT query_pt;
                query_pt.x = sp.x();
                query_pt.y = sp.y();
                query_pt.z = sp.z();

                std::vector<int> nn_idx(1);
                std::vector<float> nn_dist2(1);

                if (local_kdtree.nearestKSearch(query_pt, 1, nn_idx, nn_dist2) <= 0)
                    continue;


                if ( nn_dist2[0]< local_min_dist2) {
                    local_min_dist2 = nn_dist2[0];

                    const auto& np = env_cloud->points[nn_idx[0]];
                    local_min_point = np;   //  保存最近点

                    Eigen::Vector3f vec(np.x, np.y, np.z);
                    vec -= sp;

                    float nrm = vec.norm();
                    if (nrm > 1e-9f) vec /= nrm;
                    else vec.setZero();

                    local_min_dir = vec;
                }
            }

#pragma omp critical
            {
                if (local_min_dist2 < global_min_dist2) {
                    global_min_dist2 = local_min_dist2;
                    global_min_dir = local_min_dir;
                    global_min_point = local_min_point;   //  归并最近点
                }
            }
        } // end parallel

        if (!std::isfinite(global_min_dist2)) {
            ROS_WARN("[=== COLLISION DET ===] FAILED .mindistance  is finite");
            return false;}

        collision_det_result.minDistance_ = std::sqrt(global_min_dist2);
        collision_det_result.dir_ = global_min_dir;
        collision_det_result.nearest_point_ = global_min_point;
        ROS_INFO("[=== COLLISION DET ===] SUCCESSFUL .mindistance  is %f , direction is [%f,%f,%f]",collision_det_result.minDistance_,global_min_dir[0],global_min_dir[1],global_min_dir[2]);
        return true;
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .PCLException: %s", e.what());
        return false;
    }
    catch (const std::exception& e) {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .Exception: %s",  e.what());
        return false;
    }
    catch (...) {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .Unknown exception occurred.");
        return false;
    }
}

/**
 * @brief 计算吊钩虚拟点到环境点云的最近距离。
 *
 * @tparam PointT          点云类型（如 pcl::PointXYZ）
 * @param env_cloud         输入环境点云（已滤波、裁剪）
 * @param hook_position     吊钩虚拟点坐标（世界坐标系）
 * @param[out] collision_det_result 碰撞检测包含的所有结果信息：最小距离（单位：m）；最小距离对应的障碍点坐标；方向向量
 * @return bool             true 表示计算成功，false 表示失败（如点云为空或异常）
 */
template<typename PointT>
bool CollisionDetector<PointT>::computeMinDistance(
    const typename pcl::PointCloud<PointT>::ConstPtr& env_cloud,
    const Eigen::Vector3f& hook_position,
    CollisionDetResult<PointT>& collision_det_result)
{
    collision_det_result.minDistance_ = std::numeric_limits<float>::infinity();
    collision_det_result.dir_ = Eigen::Vector3f::Zero();
    PointT min_point;
    try {
        if (!env_cloud || env_cloud->empty()) {
            ROS_WARN("[=== COLLISION DET ===] FAILED .input cloud is null or empty");
            return false;
        }

        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(env_cloud);

        PointT query_pt;
        query_pt.x = static_cast<float>(hook_position.x());
        query_pt.y = static_cast<float>(hook_position.y());
        query_pt.z = static_cast<float>(hook_position.z());
        std::vector<int> nn_idx(1);
        std::vector<float> nn_dist2(1);

        if (kdtree.nearestKSearch(query_pt, 1, nn_idx, nn_dist2) <= 0) {
            ROS_WARN("[=== COLLISION DET ===] FAILED .kdtree.nearestKSearch failed");
            return false;
        }

        const auto& np = env_cloud->points[nn_idx[0]];
        Eigen::Vector3f nearest(np.x, np.y, np.z);
        min_point = np;   //  保存最近点
        Eigen::Vector3f vec = nearest - hook_position;
        float nrm = vec.norm();
        if (nrm > 1e-9f) vec /= nrm;
        else vec.setZero();

        collision_det_result.dir_ = vec;
        collision_det_result.minDistance_ = std::sqrt(nn_dist2[0]);
        collision_det_result.nearest_point_ = min_point;
        ROS_INFO("[=== COLLISION DET ===] SUCCESSFUL .mindistance  is %f , direction is [%f,%f,%f]",collision_det_result.minDistance_,vec[0],vec[1],vec[2]);
        return true;
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .PCLException: %s", e.what());
        return false;
    }
    catch (const std::exception& e) {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .Exception: %s",  e.what());
        return false;
    }
    catch (...) {
        ROS_ERROR("[=== COLLISION DET ===] FAILED .Unknown exception occurred.");
        return false;
    }
}



/**
 * @brief                   获取虚拟吊钩质心周围区域点云
 * @param source             输入点云
 * @param config            输入参数
 * @param busdata           输入总线参数
 * @param centerPoint       输入虚拟吊钩质心点
 * @param[out] env_cloud    输出环境点云    
*/
template<typename PointT>
void CollisionDetector<PointT>::getEnvCloud(
    typename pcl::PointCloud<PointT>::Ptr source,
    const FixedParams& config,
    const BusData& busdata,
    const Eigen::Vector3f& centerPoint,
    typename pcl::PointCloud<PointT>::Ptr env_cloud)
{
    try
    {
        // ---------- Step 0: 输入检查 ----------
        if (!source || source->empty())
        {
            ROS_WARN("[=== GET ENV CLOUD ===] Source cloud is empty or null.");
            return;
        }
        if (config.arm_range_y_[0]==config.arm_range_y_[1]||config.arm_range_z_[0]==config.arm_range_z_[1])
        {
            ROS_WARN("[=== GET ENV CLOUD ===] config uninitialize");
            return;
        }
        // ---------- Step 1: 剔除车体点云 ----------
        // 2.1 剔除车体区域点云（通过y、z范围过滤）
        pcl::PassThrough<PointT> pass_y;
        pass_y.setInputCloud(source);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(config.arm_range_y_[0], config.arm_range_y_[1]);
        pass_y.setNegative(true);
        std::vector<int> indices_y;
        pass_y.filter(indices_y);

        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud(source);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(config.arm_range_z_[0],config.arm_range_z_[1]);
        pass_z.setNegative(true);
        std::vector<int> indices_z;
        pass_z.filter(indices_z);

        std::sort(indices_y.begin(), indices_y.end());
        std::sort(indices_z.begin(), indices_z.end());

        std::vector<int> union_indices;
        std::set_union(indices_y.begin(), indices_y.end(),
                       indices_z.begin(), indices_z.end(),
                       std::back_inserter(union_indices));

        pcl::PointIndices::Ptr crane_indices(new pcl::PointIndices);
        crane_indices->indices = union_indices;

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(source);
        extract.setIndices(crane_indices);
        extract.setNegative(false);
        extract.filter(*env_cloud);

        // ---------- Step 3: 筛选安全区点云 ----------
        Eigen::Vector4f min_point(centerPoint[0] - config.bounding_box_size_[0] / 2,
                                  centerPoint[1] - config.bounding_box_size_[1] / 2,
                                  centerPoint[2] - config.bounding_box_size_[2] / 2, 1.0f);

        Eigen::Vector4f max_point(centerPoint[0] + config.bounding_box_size_[0] / 2,
                                  centerPoint[1] + config.bounding_box_size_[1] / 2,
                                  centerPoint[2] + config.bounding_box_size_[2] / 2, 1.0f);

        if(busdata.vx_>0){
            max_point[0]+=busdata.vx_*6;
        }else{
            min_point[0]+=busdata.vx_*6;
        }
        if(busdata.vy_>0){
            max_point[1]+=busdata.vy_*6;
        }else{
            min_point[1]+=busdata.vy_*6;
        }        
        if(busdata.vz_>0){
            max_point[2]+=busdata.vz_*6;
        }else{
            min_point[2]+=busdata.vz_*6;
        }
        pcl::CropBox<PointT> crop_box;
        crop_box.setInputCloud(env_cloud);
        crop_box.setMin(min_point);
        crop_box.setMax(max_point);
        crop_box.setNegative(false);
        crop_box.filter(*env_cloud);
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] PCLException: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] std::exception: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] Unknown exception caught.");
    }
};


/**
 * @brief                   获取吊钩/吊载周围区域点云
 * @param source            输入点云
 * @param config            输入参数
 * @param busdata           输入总线数据
 * @param hookloadobb       输入OBB
 * @param[out] env_cloud    输出环境点云    
*/
template<typename PointT>
void CollisionDetector<PointT>::getEnvCloud(
    typename pcl::PointCloud<PointT>::Ptr source,
    const FixedParams& config,
    const BusData& busdata,
    const OBB<PointT>& hookloadobb,
    typename pcl::PointCloud<PointT>::Ptr env_cloud)
{
    try
    {
        // ---------- Step 0: 输入检查 ----------
        if (!source || source->empty())
        {
            ROS_WARN("[=== GET ENV CLOUD ===] Source cloud is empty or null.");
            return;
        }
        if (config.arm_range_y_[0]==config.arm_range_y_[1]||config.arm_range_z_[0]==config.arm_range_z_[1])
        {
            ROS_WARN("[=== GET ENV CLOUD ===] config uninitialize");
            return;
        }
        // ---------- Step 1: 剔除车体点云 ----------
        // 2.1 剔除车体区域点云（通过y、z范围过滤）
        pcl::PassThrough<PointT> pass_y;
        pass_y.setInputCloud(source);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(config.arm_range_y_[0], config.arm_range_y_[1]);
        pass_y.setNegative(true);
        std::vector<int> indices_y;
        pass_y.filter(indices_y);

        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud(source);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(config.arm_range_z_[0],config.arm_range_z_[1]);
        pass_z.setNegative(true);
        std::vector<int> indices_z;
        pass_z.filter(indices_z);

        std::sort(indices_y.begin(), indices_y.end());
        std::sort(indices_z.begin(), indices_z.end());

        std::vector<int> union_indices;
        std::set_union(indices_y.begin(), indices_y.end(),
                       indices_z.begin(), indices_z.end(),
                       std::back_inserter(union_indices));

        pcl::PointIndices::Ptr crane_indices(new pcl::PointIndices);
        crane_indices->indices = union_indices;

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(source);
        extract.setIndices(crane_indices);
        extract.setNegative(false);
        extract.filter(*env_cloud);

        // ---------- Step 3: 剔除自身点云 ----------
        pcl::CropBox<PointT> crop_box;
        crop_box.setInputCloud(env_cloud);
        crop_box.setMin(Eigen::Vector4f(hookloadobb.min_point[0], hookloadobb.min_point[1], hookloadobb.min_point[2], 1.0f));
        crop_box.setMax(Eigen::Vector4f(hookloadobb.max_point[0], hookloadobb.max_point[1], hookloadobb.max_point[2], 1.0f));
        Eigen::Affine3f obb_transform = Eigen::Affine3f::Identity();
        obb_transform.linear() = hookloadobb.rotation;
        obb_transform.translation() = hookloadobb.position;
        crop_box.setTransform(obb_transform);

        crop_box.setNegative(true);
        crop_box.filter(*env_cloud);
        // ---------- Step 4: OBB的最小包络AABB求取 ----------
        std::vector<Eigen::Vector3f> local_corners = {
            {hookloadobb.min_point[0], hookloadobb.min_point[1], hookloadobb.min_point[2]}, {hookloadobb.min_point[0], hookloadobb.min_point[1], hookloadobb.max_point[2]},
            {hookloadobb.min_point[0], hookloadobb.max_point[1], hookloadobb.min_point[2]}, {hookloadobb.min_point[0], hookloadobb.max_point[1], hookloadobb.max_point[2]},
            {hookloadobb.max_point[0], hookloadobb.min_point[1], hookloadobb.min_point[2]}, {hookloadobb.max_point[0], hookloadobb.min_point[1], hookloadobb.max_point[2]},
            {hookloadobb.max_point[0], hookloadobb.max_point[1], hookloadobb.min_point[2]}, {hookloadobb.max_point[0], hookloadobb.max_point[1], hookloadobb.max_point[2]}
        };

        // 2. 将顶点变换到世界坐标系，并初始化 AABB 的极值
        Eigen::Vector3f aabb_min(FLT_MAX, FLT_MAX, FLT_MAX);
        Eigen::Vector3f aabb_max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (const auto& l_v : local_corners) {
            // 世界坐标 = 旋转矩阵 * 局部坐标 + 中心位置
            Eigen::Vector3f world_v = hookloadobb.rotation * l_v + hookloadobb.position;
            
            // 更新 AABB 的边界
            for (int i = 0; i < 3; ++i) {
                if (world_v[i] < aabb_min[i]) aabb_min[i] = world_v[i];
                if (world_v[i] > aabb_max[i]) aabb_max[i] = world_v[i];
            }
        }

        // ---------- Step 5: 静态情况下的防碰撞范围 ----------
        Eigen::Vector4f min_point(aabb_min[0] - config.bounding_box_size_[0] / 2,
                                  aabb_min[1] - config.bounding_box_size_[1] / 2,
                                  aabb_min[2] - config.bounding_box_size_[2] / 2, 1.0f);
        Eigen::Vector4f max_point(aabb_max[0] + config.bounding_box_size_[0] / 2,
                                  aabb_max[1] + config.bounding_box_size_[1] / 2,
                                  aabb_max[2] + config.bounding_box_size_[2] / 2, 1.0f);
        ROS_INFO("min_point:[%f,%f,%f]  ,max_point:[%f,%f,%f]",min_point[0],min_point[1],min_point[2],max_point[0],max_point[1],max_point[2]);
        // ---------- Step 6: 动态情况下的防碰撞拓展范围 ----------
        if(busdata.vx_>0){
            max_point[0]+=busdata.vx_*6;
        }else{
            min_point[0]+=busdata.vx_*6;
        }
        if(busdata.vy_>0){
            max_point[1]+=busdata.vy_*6;
        }else{
            min_point[1]+=busdata.vy_*6;
        }        
        if(busdata.vz_>0){
            max_point[2]+=busdata.vz_*6;
        }else{
            min_point[2]+=busdata.vz_*6;
        }
                // pcl::io::savePCDFileASCII("env_cloud.pcd", *env_cloud);

        // ---------- Step 7: 筛选出碰撞范围内的点云 ----------
        pcl::CropBox<PointT> final_crop; // 使用新的实例最安全
        final_crop.setInputCloud(env_cloud);
        final_crop.setMin(min_point);
        final_crop.setMax(max_point);
        final_crop.setNegative(false);
        final_crop.filter(*env_cloud);
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] PCLException: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] std::exception: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("[=== GET ENV CLOUD ===] Unknown exception caught.");
    }
};

// 只写一次
template class CollisionDetector<pcl::PointXYZ>;
template class CollisionDetector<pcl::PointXYZI>;
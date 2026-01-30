#include <HookLoadAntiCollisionDetection/pipeline.h>
template<typename PointT>
Pipeline<PointT>::Pipeline(std::shared_ptr<SharedSyncContext<PointT>> ctx):ctx_(ctx){
    //pass through filter
    //创建吊钩吊载位置获取类对象
    hookLoadPositionAcquirer_ =std::make_shared<HookLoadPositionAcquirer<PointT>>();

    //基于ipc的帧间点云匹配实现目标点云跟踪
    icpTrack_ =std::make_shared<ICPTracker<PointT>>();

    //创建碰撞检测类对象
    collisionDetector_ = std::make_shared<CollisionDetector<PointT>>();

    //体素滤波器
    sor_.setLeafSize(0.3f, 0.3f, 0.3f);
    
    //过滤器参数初始化
    csf_ = std::make_shared<CSF>();
    csf_->params.bSloopSmooth = true;
    csf_->params.class_threshold = 0.3;
    csf_->params.cloth_resolution = 0.75;
    csf_->params.interations = 500;
    csf_->params.rigidness = 3;
    csf_->params.time_step = 0.65;
    ROS_INFO("[Pipeline] Pipeline instance constructed");
}

template<typename PointT>
Pipeline<PointT>::~Pipeline(){
}

template<typename PointT>
bool Pipeline<PointT>::filteringOutGroundPC(const typename pcl::PointCloud<PointT>::Ptr& cloud ,  typename pcl::PointCloud<PointT>::Ptr& off_ground_cloud )
{
    try
    {
    // 坐标变换（CSF要求-Z轴方向为重力方向，改变点云坐标系）
        Eigen::Vector3f current_gravity(1.0f, 0.0f, 0.0f);        // 当前坐标系的重力方向：X轴（即(1, 0, 0)）
        Eigen::Vector3f target_gravity(0.0f, 0.0f, -1.0f);        // 目标坐标系的重力方向：Z轴（即(0, 0, 1)）
        Eigen::Vector3f axis = current_gravity.cross(target_gravity);        // 计算旋转轴：即current_gravity与target_gravity的叉积
        axis.normalize();
        float angle = acos(current_gravity.dot(target_gravity));        // 计算旋转角度：即current_gravity与target_gravity的点积反余弦
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = Eigen::AngleAxisf(angle, axis); // 使用角轴表示法
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(rotation_matrix);

        // 旋转点云
        typename pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        std::vector<int> ground_indices, off_ground_indices;

        csf_->setPointCloud(transformed_cloud);

        csf_->do_filtering_v2(ground_indices, off_ground_indices, false);
        if(off_ground_indices.empty()){
            ROS_WARN("[=== CSF ===] No off_ground_cloud");
            return false;
        }
        // 提取地面/非地面点云
        pcl::PointIndices::Ptr obstacle_idx(new pcl::PointIndices);
        obstacle_idx->indices = off_ground_indices;
        // obstacle
        pcl::ExtractIndices<PointT> extract_obstacle;
        extract_obstacle.setInputCloud(transformed_cloud);
        extract_obstacle.setIndices(obstacle_idx);
        extract_obstacle.setNegative(false);
        extract_obstacle.filter(*off_ground_cloud);
        if(!off_ground_cloud||off_ground_cloud->empty()){
            ROS_WARN("[=== CSF ===] off_ground_cloud is null or empty");
            return false;
        }
        // 计算逆变换矩阵（后续吊钩吊载检测坐标系为未转换的坐标系，因此转换回去）
        Eigen::Affine3f inverse_transform = transform.inverse();
        pcl::transformPointCloud(*off_ground_cloud,*off_ground_cloud,inverse_transform);
        return true;
    }
    catch (const pcl::PCLException& e)
    {
        ROS_ERROR("[=== CSF ===] PCLException: %s", e.what());
        return false;
    }
    catch(const std::exception &e){
        ROS_ERROR("[=== CSF ===] exception: %s", e.what());
        return false;
    }
    catch(...){
        ROS_ERROR("[=== CSF ===] unknown exception occurred.");
        return false;
    }
}

template<typename PointT>
bool Pipeline<PointT>::tryDetectTarget(typename pcl::PointCloud<PointT>::Ptr& cloud) {
    typename pcl::PointCloud<PointT>::Ptr off_ground_cloud(new pcl::PointCloud<PointT>);
    if(!filteringOutGroundPC(cloud,off_ground_cloud)){
        ROS_WARN("[=== CSF ===] FAILED ");
        return false;
    }
    ROS_INFO("[=== CSF ===] SUCCESSFULL ");


    if (hookLoadPositionAcquirer_->getHookLoadCluster(off_ground_cloud, ctx_->busData_,hookClusterInfo_, loadClusterInfo_)) {
        //成功检测到吊钩或吊载点云
        if (loadClusterInfo_.exist() || hookClusterInfo_.exist()) {
            targetClusterInfo_last_frame_ = loadClusterInfo_.exist() ? loadClusterInfo_ : hookClusterInfo_;
            loadClusterInfo_.clearAll();
            hookClusterInfo_.clearAll();
            targetClusterInfo_last_frame_.computeOBB();
            collisionDetector_->getEnvCloud(cloud,  ctx_->fixedParams_, ctx_->busData_, targetClusterInfo_last_frame_.obb,  ROI_cloud_);
            collisionDetector_->computeMinDistance(ROI_cloud_, targetClusterInfo_last_frame_.obb,ctx_->collisionDetResult_, 0.2f, 4);
            target_det_mode_ = Mode::TrackMode;
            ROS_INFO("[=== HOOK/LOAD DETECTION ===] SUCCESSFULL ");
            return true;
        }
    }

    //getHookLoadCluster未完整执行，中途退出，
    targetClusterInfo_last_frame_ =  hookClusterInfo_;
    if(targetClusterInfo_last_frame_.centroid != Eigen::Vector3f::Zero()){
        collisionDetector_->getEnvCloud(cloud,  ctx_->fixedParams_,    ctx_->busData_,  targetClusterInfo_last_frame_.centroid, ROI_cloud_);
        collisionDetector_->computeMinDistance( ROI_cloud_, targetClusterInfo_last_frame_.centroid,ctx_->collisionDetResult_);
    }
    ROS_WARN("[=== HOOK/LOAD DETECTION ===] FAILED ");
    return false;
}

template<typename PointT>
bool Pipeline<PointT>::forward(typename pcl::PointCloud<PointT>::Ptr& cloud){
    Pipeline::reset();

    //体素滤波
    sor_.setInputCloud(cloud);
    sor_.filter(*filtered_cloud_);

    switch (target_det_mode_) {
        case Mode::TrackMode:
            if (targetClusterInfo_last_frame_.exist()) {    // 存在上一帧识别的吊钩/吊载模板，直接退回检测模式
                if (icpTrack_->targetTrack(filtered_cloud_,   targetClusterInfo_last_frame_, targetClusterInfo_cur_frame_)) {
                    // ICP成功 → 更新目标
                    targetClusterInfo_last_frame_ = targetClusterInfo_cur_frame_;
                    targetClusterInfo_last_frame_.computeOBB();
                    collisionDetector_->getEnvCloud(filtered_cloud_,  ctx_->fixedParams_, ctx_->busData_,  targetClusterInfo_last_frame_.obb,  ROI_cloud_);
                    collisionDetector_->computeMinDistance(ROI_cloud_,targetClusterInfo_last_frame_.obb,ctx_->collisionDetResult_,0.2f, 4);
                    break;
                }
            }

        default:
            tryDetectTarget(filtered_cloud_);
            break;
    }

    //-------------------------------------信息更新---------------------------------------------------------------
    {
        std::lock_guard<std::mutex>lock(ctx_->dataMutex_);
        ctx_->frame_id_++;
        updateVisInfo();
    }
    ctx_->cv_.notify_all();
    return true;
}

template<typename PointT>
void Pipeline<PointT>::reset(){
    //中间变量清理
    if (ROI_cloud_) {
        ROI_cloud_->clear();
    } else {
        ROI_cloud_.reset(new pcl::PointCloud<PointT>());   
    }
    if (filtered_cloud_) {
        filtered_cloud_->clear();
    } else {
        filtered_cloud_.reset(new pcl::PointCloud<PointT>());   
    }
    targetClusterInfo_cur_frame_.clearAll();
    hookClusterInfo_.clearAll();
    loadClusterInfo_.clearAll();

}

template<typename PointT>
void Pipeline<PointT>::updateVisInfo(){
    ctx_->clear();
    
    //场景信息
    std::shared_ptr<CloudVisInfo<PointT>> filtered_cloud_vis_info = std::make_shared<CloudVisInfo<PointT>>(filtered_cloud_,"filtered_cloud_color");
    ctx_->CloudVisInfo_.push_back(filtered_cloud_vis_info);


    //检测区域点云环境信息
    std::shared_ptr<CloudVisInfo<PointT>> ROI_vis_info = std::make_shared<CloudVisInfo<PointT>>(ROI_cloud_,"ROI_cloud",0,255,0);
    ctx_->CloudVisInfo_.push_back(ROI_vis_info);

    //hook/load目标信息
    if (targetClusterInfo_last_frame_.exist()) {
        if(target_det_mode_ == Mode::DetectMode){
            //Detect->Detect
            std::shared_ptr<CloudVisInfo<PointT>> target_vis_info = std::make_shared<CloudVisInfo<PointT>>(targetClusterInfo_last_frame_.cloud,"target_cloud",255,0,0);
            ctx_->CloudVisInfo_.push_back(target_vis_info);
        }
        else{
            if(targetClusterInfo_cur_frame_.exist()){
                //Track->Track
                std::shared_ptr<CloudVisInfo<PointT>> target_vis_info = std::make_shared<CloudVisInfo<PointT>>(targetClusterInfo_cur_frame_.cloud,"target_cloud",255,0,0);
                ctx_->CloudVisInfo_.push_back(target_vis_info);
            }
            else{
                //Detect->Track
                std::shared_ptr<CloudVisInfo<PointT>> target_vis_info = std::make_shared<CloudVisInfo<PointT>>(targetClusterInfo_last_frame_.cloud,"target_cloud",255,0,0);
                ctx_->CloudVisInfo_.push_back(target_vis_info);
            }
        }
    }

    //障碍物信息


}
template class Pipeline<pcl::PointXYZ>;
template class Pipeline<pcl::PointXYZI>;
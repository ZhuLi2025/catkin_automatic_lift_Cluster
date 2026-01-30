#pragma once
#include <mutex>
#include <condition_variable>
#include <pcl/point_cloud.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief AABB包围盒
 * 用于存储目标点云的基本尺寸信息
 * @struct AABB
 */
struct AABB{
    Eigen::Vector3f translation;
    Eigen::Quaternionf identity_rotation ;
    float Len_x;
    float Len_y;
    float Len_z;
    std::string box_id;
    AABB(Eigen::Vector3f trans,Eigen::Quaternionf rotation,
        float len_x,
        float len_y,
        float len_z,
        std::string id)
    :translation(trans),identity_rotation(rotation),Len_x(len_x),Len_y(len_y),Len_z(len_z),box_id(id)
    {}
};
 
/**
 * @brief 可视化点云信息
 * 用于存储用于可视化的信息
 * @struct CloudVisInfo
 */
template<typename PointT>
struct CloudVisInfo {
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    int r_{255},g_{255},b_{255};
    std::string cloudID_;

    CloudVisInfo(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        std::string cloudID,
        int r=255,
        int g=255,
        int b=255
    ):cloud_(cloud),r_(r),g_(g),b_(b),cloudID_(cloudID){}

    // 深拷贝构造函数
    CloudVisInfo(const CloudVisInfo& other)
    {
        // 深拷贝点云数据
        cloud_ = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*other.cloud_));
        r_=other.r_;
        g_=other.g_;
        b_=other.b_;
        cloudID_ = other.cloudID_;
    }
};

/**
 * @brief 总线数据
 * 用于存储实时从总线获取的数据信息，作为先验约束辅助点云处理算法
 * @struct BusData
 */
struct BusData {
    //从总线数据解析线程获取
    float rope_len_;
    bool load_exist_flag_;
    float vx_{0.0},vy_{3.0},vz_{0.0};
    bool b_updated_{false};
    // 默认构造函数
    BusData() = default;

    // 带参构造函数
    BusData(float rope_len,
           bool load_exist_flag,
           float vx,
           float vy,
           float vz,
           bool b_updated)
        : rope_len_(rope_len),
          load_exist_flag_(load_exist_flag),
          vx_(vx),vy_(vy),vz_(vz),
          b_updated_(b_updated)
          {}
};

/**
 * @brief 固定的参数
 * 用于存储初始化系统的参数，从参数配置文件读取
 * @struct FixedParams
 */
struct FixedParams {
    //CAN总线
    std::string can_port_;

    //起重机车体臂架大致范围
    std::array<float, 2> arm_range_y_{ {0.0f, 0.0f} };
    std::array<float, 2> arm_range_z_{ {0.0f, 0.0f} };
    //碰撞检测空间尺寸
    std::array<float, 3> bounding_box_size_{ {30.0f, 50.0f, 50.0f} };

    // 默认构造函数
    FixedParams() = default;

    // 带参构造函数
    FixedParams(
           const std::array<float, 2>& arm_range_y,
           const std::array<float, 2>& arm_range_z,
           const std::array<float, 3>& bounding_box_size)
        :arm_range_y_(arm_range_y),
        arm_range_z_(arm_range_z),
        bounding_box_size_(bounding_box_size) {}
            
    bool setFixedParams(const std::string& file_path)
    {
        try {
            YAML::Node config = YAML::LoadFile(file_path);

            if (config["arm_range_y"])
                arm_range_y_ = {
                    config["arm_range_y"][0].as<float>(),
                    config["arm_range_y"][1].as<float>()
                };

            if (config["arm_range_z"])
                arm_range_z_ = {
                    config["arm_range_z"][0].as<float>(),
                    config["arm_range_z"][1].as<float>()
                };

            if (config["bounding_box_size"])
                bounding_box_size_ = {
                    config["bounding_box_size"][0].as<float>(),
                    config["bounding_box_size"][1].as<float>(),
                    config["bounding_box_size"][2].as<float>()
                };
            ROS_INFO("[setFixedParams] FixedParams loaded successfully from: %s",file_path.c_str());
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("[setFixedParams] Failed to load FixedParams: %s",e.what());
            return false;
        }
    }
};

/**
 * @brief 碰撞检测结果信息
 * 用于存储吊钩/吊载与周围区域内点云的距离检测结果，其中包含：OBB包络盒表面点与周围障碍点的最近的距离、方向、对应障碍点的坐标
 * @struct CollisionDetResult
 */
template < typename PointT>  
struct CollisionDetResult{
    Eigen::Vector3f dir_;
    float minDistance_;
    PointT nearest_point_;
    /**
     * @brief 构造函数，初始化成员变量
     * @param dir 直线方向单位向量
     * @param minDistance 最小距离
     * @param nearest_point 最近障碍点坐标
     */
    CollisionDetResult(const Eigen::Vector3f& dir, float minDistance, const PointT& nearest_point)
        : dir_(dir), minDistance_(minDistance), nearest_point_(nearest_point) {}

    // 默认构造函数
    CollisionDetResult() : dir_(Eigen::Vector3f::Zero()), minDistance_(0.0f), nearest_point_(PointT()) {}

    // 复制构造函数
    CollisionDetResult(const CollisionDetResult& other)
        : dir_(other.dir_), minDistance_(other.minDistance_), nearest_point_(other.nearest_point_) {}

    // 赋值操作符重载
    CollisionDetResult& operator=(const CollisionDetResult& other) {
        if (this != &other) {
            dir_ = other.dir_;
            minDistance_ = other.minDistance_;
            nearest_point_ = other.nearest_point_;
        }
        return *this;
    }
};


/**
 * @brief 上下文信息
 * SharedSyncContext    同步上下文，用于子线程与主线程之间数据传递
 *  |-Config            传递给子线程处理需要的参数
 *  |-CollisionDetResult    传递子线程处理的碰撞检测结果
 *  |-dataCond           条件变量，当子线程处理完毕与主线程同步
 *  |-CloudVisInfo       点云可视化信息
 *  |-BoxInfo           重点关注对象的bounding box
 * @struct SharedSyncContext
*/
template<typename PointT>
struct SharedSyncContext {
    //--固定参数--
    FixedParams fixedParams_;

    //--总线参数--
    BusData busData_;

    //--结果数据--
    CollisionDetResult<PointT> collisionDetResult_;
    std::vector<std::shared_ptr<CloudVisInfo<PointT>>> CloudVisInfo_;
    std::vector<AABB> BoxInfo_;

    //--同步机制--
    std::mutex dataMutex_;
    std::condition_variable cv_;
    uint64_t frame_id_;


    // --构造函数--
    SharedSyncContext(){
        busData_ = BusData(63.0,false,0.0,3.0,3.0,false);
    };

    void clear(){
        BoxInfo_.clear();
        CloudVisInfo_.clear();
    }
};

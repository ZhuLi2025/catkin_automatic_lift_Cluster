//#pragma once
#ifndef _OBSTACLE_DETECTION_ALG_H
#define _OBSTACLE_DETECTION_ALG_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>       //平面分割
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>

#include <boost/thread/thread.hpp>                   //线程

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>

#include "opencv2/calib3d/calib3d.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <zl_manipulation/AutoCraneArmManipulation.h>
typedef pcl::PointXYZ PointType;

//滤波参数
struct FilterParameter{
  double zLimitMin;
  double zLimitMax;
  float leafSize;
  unsigned MinNumVoxel;
};

//聚类分割参数
struct SegParameter{
  double ecClusterTolerance;
  int ecMinCluster;
  int ecMaxCluster;
};

//Ŀ���������Ϣ
struct ObstacleInformation{
  std::vector<Eigen::Vector3f> ObLoc;
  std::vector<int> ObSize;
  std::vector<Eigen::Vector4f> ObBound;
};

//障碍物3DBox表示
struct Obstacle3DBoxStr
{
  float width_x; //长
  float height_y;  //宽
  float depth_z; //高
};

//起重机臂尖状态信息
struct CraneStateStr
{
  float hzAngle;//回转角度
  float bcAngle;//臂夹角
  float bcLength;//臂长
  float bfLength;//变幅长
  int focus;//相机焦距
};

//障碍物输出信息
struct ObstacleInforStr
{
  PointType centerPt;//中心点坐标
  cv::Point3f pts[4];//俯视角矩形上表面4个顶点
  Obstacle3DBoxStr box3d;//包围盒尺寸（长l、宽w、高h）
  cv::Rect rect2D;//图像显示的矩形框
  float orientation;//方向
  float distance;//距离
  int obstacleType;//障碍物类型（0：吊钩/吊载物，1：障碍物）
  float yaw;
  float pitch;
  float roll;
};

struct Obstacle_state_Information
{
  Eigen::Vector3d coordinate_point[8];
};

struct DepthType
{
  int index;
  float value;
};

inline bool DepthComp(DepthType &a, DepthType &b)
{
  return a.value < b.value;
}

class ObstacleDetection
{
public:
  ObstacleDetection();
  ~ObstacleDetection() = default;
  void InitialParaConfig(const char* cfg_file_path);
  //int ObstacleDetected(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const cv::Mat& rgbImg, CraneStateStr craneState, std::vector<ObstacleInforStr>& outObjectInfo,float& outPlaneHight);
  int ObstacleDetected(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CraneStateStr craneState,std::vector<ObstacleInforStr>& outObInfors,
                                          double &laser_hook_or_lift_height, double ground_height, double crane_hook_height);
  void Convert3Dto2D(std::vector<cv::Point3f> objectPoints, int focal, std::vector<cv::Point2f>& projectedPoints);
  void PositionTranslate(const PointType radarPos,const float armAngle,PointType& armPos);//雷达坐标点到臂尖坐标点转换
  void GetObstaclesValue(std::vector<ObstacleInforStr>& outObjectInfors);
  void GetHookHightValue(float& outHookHight);
  int GetPlaneHight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double& outPlaneHight);

  //障碍物坐标转换
  void obstacleTranstion();

public:
  double m_dpreHookHeight;
//  cv::Mat _depthImg;
  FilterParameter _FilterPara;
  SegParameter _SegPara;

  ObstacleInformation OriginObInfo;
  ObstacleInformation SortObInfo;
  std::vector<int> rangValue;
  int DepthMergeRange;
  double wh_rate;
  int roise_len;
  int roise_size;
  double ROIOffset;
  double biasSum;
  double zStableThr;

  double upHookThresh;
  double downHookThresh;
  double zoneCluster_y_step,zoneCluster_z_step,hookZoneX_min,hookZoneX_max,hookZoneY_min,hookZoneY_max;
  double m_hook_height, m_object_height, m_object_len, m_object_width, m_rope_height, m_goods_height;
  int m_nMode;
  double m_laser_hook_or_lift_height;

  void GetObjectsSize(double hook_height, double obj_width, double obj_length, double obj_height, double sling_height, double lift_Height,int lift_mode);
private:
//  void depth2pc();//点云格式转换
  void CloudsPtFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS);//点云滤波降噪
  int cloud_LFS(const pcl::PointCloud<pcl::PointXYZ> &cloudIn, const std::vector<int> &indices, float minZ, float maxZ, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut);
  void ObInfoSort();
  bool BoundIsOverlap(Eigen::Vector4f &aBound, Eigen::Vector4f &bBound);
  int ObInfoMerge(ObstacleInformation &outObInfo);
  int ObInfoMerge_W(ObstacleInformation &outObInfo);
  //void PlaneSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtS);//地面提取
  void PlaneSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS);//地面提取
  int Obstacle3DBBox(std::vector<ObstacleInforStr>& outObjectInfors, double laser_hook_or_lift_height, double ground_height);//障碍物3Dbox显示及几何尺寸计算
  int Obstacle3DBBox1(std::vector<Obstacle_state_Information>& obstacles_in_livox_coordinate);

  int Obstacle3DBBoxZoneHook(std::vector<ObstacleInforStr> &outObjectInfors, pcl::PointCloud<pcl::PointXYZ>::Ptr zoneCloud, double ground_height);
  int Obstacle3DBBoxZone(std::vector<ObstacleInforStr> &outObjectInfors, pcl::PointCloud<pcl::PointXYZ>::Ptr zoneCloud, double ground_height);
  void zoneCluster(std::vector<ObstacleInforStr> &outObinfors, double ground_height);
  void filterZoneCluser(std::vector<ObstacleInforStr> &outObinfors, double ground_height);
  void zoneGetPlaneHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &outPlaneHight);
  void getHookHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr &hookCloud, double ground_height);

  void CloudsPtSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS);//点云聚类分割
  int RecongnizeHook();//吊钩识别及吊钩高度计算
  void RemainHookData(pcl::PointIndices cluster,pcl::PointCloud<pcl::PointXYZ> cloud_hk);//空中吊钩点云数据存储

  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudT;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudF;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudPE;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudObs;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudPE1;
  pcl::PointCloud<pcl::PointXYZ> cloud_hook_;
  std::vector<pcl::PointIndices> _clusterS;
  //std::vector< std::vector<PointType> > object_upsurface_pts;
  double translateMat[3];
  double rotatoMat[9];
  float plane_hight_;
  float hook_hight_;
  std::vector<ObstacleInforStr> obstacle_infors_;
  Eigen::Vector4f hookmin_p;//3DBox各顶点的x y z坐标对应的最小值
  Eigen::Vector4f hookmax_p;//3DBox各顶点的x y z坐标对应的最大值
  std::mutex h_mutex_;
  int frame_num_;

  ompl::app::AutoCraneArmManipulation auto_crane_manipulation_1; //motion planning

  std::vector<Eigen::Vector4f> allMin_p;
  std::vector<Eigen::Vector4f> allMax_p;
  //std::vector<int> rangValue;
};


void read_pcd(std::string pcd_path,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
#endif

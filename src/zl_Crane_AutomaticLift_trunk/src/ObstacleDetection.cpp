#include "ObstacleDetection.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "/home/zrtan/zl_vision/include/zl_vision/pcviewer.h"
#include "zl_common_define.hpp"
#define HW_RATE 1.1
#define NOISE_LEN 3
#define NOISE_SIZE 200
#define MAX_PATH 260
#define DEBUG_MODE 0   //打印调试窗口

#define zoneMode 1  //1 区域聚类模式 0 全局聚类

extern zl_common::app::CRANE_PARAM *crane_parameter_;

//2021.08.07 add by xupeipei 实时检测吊钩高度变量声明 start
ObstacleInforStr hookInfo;
//2021.08.07  add by xupeipei 实时检测吊钩高度变量声明 end

bool readConfigFile(const char* cfg_file_path,const std::string &key,std::string &val);
//void showColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<pcl::PointIndices::Ptr> clusters);
void showColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<pcl::PointIndices> clusters);

const double g_intriMax[12][9] = {{9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1},
                                  {9.297818e+002,0,0,0,9.143735e+002,0,4.881188e+002,4.050004e+002,1}
                                 };//多（12）级变倍焦距对应的相机内参值

//pcl::visualization::PCLVisualizer viewer("3DBbox");
//boost::shared_ptr<PCViewer> viewer(new PCViewer());

ObstacleDetection::ObstacleDetection()
{
  _cloudT = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  _cloudF = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  _cloudPE = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  _cloudObs = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  _cloudPE1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  _cloudT->points.clear();
  _cloudF->points.clear();
  _cloudPE->points.clear();
  _cloudObs->points.clear();
  obstacle_infors_.clear();

  _FilterPara = {0, 0, 0, 1};
  _SegPara = {0, 0, 0};

  DepthMergeRange = 0;
  wh_rate = 1.1;
  roise_len = 0;
  roise_size = 0;
  ROIOffset = 0.17;
  biasSum = 10.0;
  zStableThr = 10.0;

  plane_hight_ = 0.0;
  hook_hight_ = 0.0;
  frame_num_ = 0;

  m_dpreHookHeight = 0.0;
  m_laser_hook_or_lift_height = 0.0;
}

/************************************************************************/
/*读取配置文件内容                                               */
/************************************************************************/
bool readConfigFile(const char* cfg_file_path,const std::string &key,std::string &val)
{
  std::fstream _cfgFile;
  _cfgFile.open(cfg_file_path);
  if (!_cfgFile.is_open())
  {
    //std::cerr<<"cout"<<std::endl;
    return false;
  }
  char tmp[1000];
  while (!_cfgFile.eof())
  {
    _cfgFile.getline(tmp,1000);
    std::string line(tmp);
    size_t pos = line.find("=");
    size_t pos_end = line.find("#");

    if (pos == std::string::npos)
    {
      return false;
    }
    std::string _tmpKey = line.substr(0,pos);
    if (key == _tmpKey)
    {
      val = line.substr(pos+1,pos_end-pos-1);
      return true;
    }
  }
  return false;
}

/************************************************************************/
/*通过配置文件初始化系统参数                            */
/************************************************************************/
void ObstacleDetection::InitialParaConfig(const char* cfg_file_path)
{
  std::string str_ZLimitMin;
  std::string str_ZlimitMax;
  std::string str_leafSize;
  std::string str_MinNumVoxel;
  std::string str_ecClusterTolerance;
  std::string str_ecMinCluster;
  std::string str_ecMaxCluster;
  std::string str_rMat;
  std::string str_tVec;
  std::string str_upHookThresh;
  std::string str_downHookThresh;
  std::string str_zoneCluster_y_step;
  std::string	str_zoneCluster_z_step;
  std::string	str_hookZoneX_min;
  std::string	str_hookZoneX_max;
  std::string	str_hookZoneY_min;
  std::string	str_hookZoneY_max;

  if (!readConfigFile(cfg_file_path,"zLimitMin",str_ZLimitMin))
  {
    _FilterPara.zLimitMin = 1;
  }
  else
  {
    _FilterPara.zLimitMin = atof(str_ZLimitMin.c_str());
  }

  if (!readConfigFile(cfg_file_path,"zLimitMax",str_ZlimitMax))
  {
    _FilterPara.zLimitMax = 150.0;
  }
  else
  {
    _FilterPara.zLimitMax = atof(str_ZlimitMax.c_str());
  }


  if (!readConfigFile(cfg_file_path,"leafSize",str_leafSize))
  {
    _FilterPara.leafSize = 0.05;
  }
  else
  {
    _FilterPara.leafSize = atof(str_leafSize.c_str());
  }

  if (!readConfigFile(cfg_file_path,"MinNumVoxel",str_MinNumVoxel))
  {
    _FilterPara.MinNumVoxel = 1;
  }
  else
  {
    _FilterPara.MinNumVoxel = atof(str_MinNumVoxel.c_str());
  }

  if (!readConfigFile(cfg_file_path,"ecClusterTolerance",str_ecClusterTolerance))
  {
    _SegPara.ecClusterTolerance = 0.5;
  }
  else
  {
    _SegPara.ecClusterTolerance = atof(str_ecClusterTolerance.c_str());
  }

  std::cout << "ecClusterTolerance" << _SegPara.ecClusterTolerance << endl;
  if (!readConfigFile(cfg_file_path,"ecMinCluster",str_ecMinCluster))
  {
    _SegPara.ecMinCluster = 5;
  }
  else
  {
    _SegPara.ecMinCluster = atof(str_ecMinCluster.c_str());
  }

  if (!readConfigFile(cfg_file_path,"ecMaxCluster",str_ecMaxCluster))
  {
    _SegPara.ecMaxCluster = 1000000;
  }
  else
  {
    _SegPara.ecMaxCluster = atof(str_ecMaxCluster.c_str());
  }

  if (!readConfigFile(cfg_file_path,"upHookThresh",str_upHookThresh))
  {
    upHookThresh = 2.6;
  }
  else
  {
    upHookThresh = atof(str_upHookThresh.c_str());
  }

  if (!readConfigFile(cfg_file_path,"downHookThresh",str_downHookThresh))
  {
    downHookThresh = 1.0;
  }
  else
  {
    downHookThresh = atof(str_downHookThresh.c_str());
  }

  if (!readConfigFile(cfg_file_path,"zoneCluster_y_step",str_zoneCluster_y_step))
  {
    zoneCluster_y_step = 3.0;
  }
  else
  {
    zoneCluster_y_step = atof(str_zoneCluster_y_step.c_str());
  }

  if (!readConfigFile(cfg_file_path,"zoneCluster_z_step",str_zoneCluster_z_step))
  {
    zoneCluster_z_step = 3.0;
  }
  else
  {
    zoneCluster_z_step = atof(str_zoneCluster_z_step.c_str());
  }

  if (!readConfigFile(cfg_file_path,"hookZoneX_min",str_hookZoneX_min))
  {
    hookZoneX_min = -1.5;
  }
  else
  {
    hookZoneX_min = atof(str_hookZoneX_min.c_str());
  }


  if (!readConfigFile(cfg_file_path,"hookZoneX_max",str_hookZoneX_max))
  {
    hookZoneX_max = 1.5;
  }
  else
  {
    hookZoneX_max = atof(str_hookZoneX_max.c_str());
  }

  if (!readConfigFile(cfg_file_path,"hookZoneY_min",str_hookZoneY_min))
  {
    hookZoneY_min = -1.5;
  }
  else
  {
    hookZoneY_min = atof(str_hookZoneY_min.c_str());
  }

  if (!readConfigFile(cfg_file_path,"hookZoneY_max",str_hookZoneY_max))
  {
    hookZoneY_max = 1.5;
  }
  else
  {
    hookZoneY_max = atof(str_hookZoneY_max.c_str());
  }

  //旋转矩阵
  for(int i=0; i<9; i++)
  {
    char str_mat[20];
    std::string tempMat;
    sprintf(str_mat,"rMat_%d",i);
    tempMat = str_mat;

    if (!readConfigFile(cfg_file_path,tempMat,str_rMat))
    {
      if((i==0) || (i==4) || (i==8))
      {
        rotatoMat[i] = 1.0;
      }
      else
      {
        rotatoMat[i] = 0.0;
      }

    }
    else
    {
      rotatoMat[i] = atof(str_rMat.c_str());
    }
  }

  //平移向量
  for(int i=0; i<3; i++)
  {
    char str_vec[20];
    std::string tempVec;
    sprintf(str_vec,"rVec_%d",i);
    tempVec = str_vec;

    if (!readConfigFile(cfg_file_path,tempVec,str_tVec))
    {
      translateMat[i] = 0;
    }
    else
    {
      translateMat[i] = atof(str_tVec.c_str());
    }
  }

}


//void ObstacleDetection::depth2pc()
//{
//  //float FocalLength = 250;//���ࣨ���ص�λ��
//  float offsetRate = ROIOffset;// 0.33*0.5;//��Զ����15�ף�ˮƽ�ӳ���90�㣬�������?10�׼���

//  for (int i = 0+int(_depthImg.rows*offsetRate); i < int(_depthImg.rows*(1-offsetRate)); i++)
//  {
//    for (int j = 0+int(_depthImg.cols*offsetRate); j < int(_depthImg.cols*(1 - offsetRate)); j++)
//    {
//      pcl::PointXYZ point;
//      if (_depthImg.at<ushort>(i, j) == 0)
//      {
//        continue;
//      }
//      else
//      {
//        //point.x = double(i / FocalLength * img.at<ushort>(i, j));
//        //point.y = double(j / FocalLength * img.at<ushort>(i, j));
//        point.x = float(j);
//        point.y = float(i);
//        point.z = float(floor(_depthImg.at<ushort>(i, j)/10));
//      }
//      _cloudT->points.push_back(point);
//    }
//  }
//  _cloudT->width = int(_cloudT->points.size());  _cloudT->height = 1;
//}

void ObstacleDetection::CloudsPtFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS)
{
  //pcl::visualization::PCLVisualizer viewer("3DBbox");
  // 创建体素网格对象
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  // 输入点云
  vg.setInputCloud(cloudPtS);
  // x轴范围
  //vg.setFilterFieldName("x");
  //vg.setFilterLimits(_FilterPara.zLimitMin, _FilterPara.zLimitMax);

  //    // y轴范围
  //    vg.setFilterFieldName("y");
  //    vg.setFilterLimits(0, 20);

  // z轴范围
  //    vg.setFilterFieldName("z");
  //    vg.setFilterLimits(0, 100);

  // 下采样比例
  vg.setLeafSize(_FilterPara.leafSize, _FilterPara.leafSize, _FilterPara.leafSize);
  // 单个体素中最小点云个数
  vg.setMinimumPointsNumberPerVoxel(_FilterPara.MinNumVoxel);
  // 输出点云
  vg.filter(*_cloudF);

  int ret = 0;


  //    viewer.addPointCloud<pcl::PointXYZ>(_cloudF);

  //    while (!viewer.wasStopped())
  //    {
  //        viewer.spinOnce();
  //    }


}

void ObstacleDetection::CloudsPtSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS)
{
  // Creating the KdTree object for the search method of the extraction
  if(_cloudPE->points.size() == 0)
  {
    //判断滤波后是否存在点云数据
    return ;
  }
  //std::cerr<<"seg after point clouds size is:"<<_cloudPE->points.size()<<std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(_cloudPE);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(_SegPara.ecClusterTolerance); //紧邻点聚类精度容差，紧邻搜索半径，单位基于欧式空间
  ec.setMinClusterSize(_SegPara.ecMinCluster);
  ec.setMaxClusterSize(_SegPara.ecMaxCluster);
  ec.setSearchMethod(tree);
  ec.setInputCloud(_cloudPE);
  ec.extract(_clusterS);

  //std::cerr << "Seg clusters size " << _clusterS.size() << std::endl;

}

// Low Frequency Suppression����Ƶ����
int ObstacleDetection::cloud_LFS(const pcl::PointCloud<pcl::PointXYZ> &cloudIn, const std::vector<int> &indices, float minZ, float maxZ,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOut)
{
  // �������z���������ڷֲ�������������ֵ
  cv::Mat ClusterMat(indices.size(), 1, CV_16UC1);
  for (size_t i = 0; i < indices.size(); i++)
  {
    ClusterMat.at<ushort>(i, 0) = cloudIn[indices[i]].z;
  }
  // ����ֱ��ͼ����ĳ���?
  int histSize = (maxZ - minZ) / 20;
  float range[] = { minZ-1, maxZ+1 };
  const float *histRange = { range };
  cv::Mat hist;
  cv::calcHist(&ClusterMat, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);
  // maxLoc.yΪ��Ӧ�ķֲ���������z����ֵ
  double maxHist;
  cv::Point maxLoc;
  cv::minMaxLoc(hist, nullptr, &maxHist, nullptr, &maxLoc);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloudIn);
  pass.setInputCloud(cloudPtr);
  boost::shared_ptr<std::vector<int>> IndicesPtr = boost::make_shared<std::vector<int>>(indices);
  pass.setIndices(IndicesPtr);
  pass.setFilterFieldName("z");
  // ����������z����Ϊ���ģ��������������?50%���ָ�ԭʼ����
  pass.setFilterLimits(minZ + 20 * (maxLoc.y - histSize/4), minZ + 20 * (maxLoc.y + 1 + histSize/4));
  pass.filter(*cloudOut);

  return 0;
}

// �жϱ߽��Ƿ��ص�
bool ObstacleDetection::BoundIsOverlap(Eigen::Vector4f &aBound, Eigen::Vector4f &bBound)
{
  cv::Rect a;
  a.x = aBound[0];
  a.y = aBound[1];
  a.width = aBound[2] - aBound[0];
  a.height = aBound[3] - aBound[1];
  cv::Rect b;
  b.x = bBound[0];
  b.y = bBound[1];
  b.width = bBound[2] - bBound[0];
  b.height = bBound[3] - bBound[1];

  bool isOverlap = false;
  // �ж�a��b�Ƿ��ص�
  cv::Rect overlapArea = a & b;
  if (overlapArea.width != 0 || overlapArea.height != 0)
  {
    isOverlap = true;
  }

  return isOverlap;
}

// ObInfo����
void ObstacleDetection::ObInfoSort()
{
  // �������±�����ֵ����
  std::vector<DepthType> depthVec;
  for (auto i = 0; i < OriginObInfo.ObLoc.size(); i++)
  {
    depthVec.push_back({ i, OriginObInfo.ObLoc[i][2] });
  }
  // ���ֵ��������?
  std::sort(depthVec.begin(), depthVec.end(), DepthComp);
  // �����������±����ObInfo
  for (auto i = 0; i < OriginObInfo.ObLoc.size(); i++)
  {
    SortObInfo.ObLoc.emplace_back(OriginObInfo.ObLoc[depthVec[i].index]);
    SortObInfo.ObBound.emplace_back(OriginObInfo.ObBound[depthVec[i].index]);
    SortObInfo.ObSize.emplace_back(OriginObInfo.ObSize[depthVec[i].index]);
  }
}


int ObstacleDetection::ObInfoMerge_W(ObstacleInformation &outObInfo)
{

  ObInfoSort();

  if (DepthMergeRange == 0) return 1;

  std::vector<std::vector<int>> MergeIndexVec;

  std::vector<int> NearIndexVec;

  std::vector<bool> processed(SortObInfo.ObBound.size(), false);

  unsigned int i = 0;
  while (i < SortObInfo.ObBound.size())
  {

    if (processed[i]) {
      i++;
      continue;
    }

    NearIndexVec.push_back(i);
    processed[i] = true;


    auto j = i + 1;
    while (j < SortObInfo.ObBound.size())
    {

      if (processed[j]) {
        j++;
        continue;
      }

      if (abs(SortObInfo.ObLoc[i][2] - SortObInfo.ObLoc[j][2]) < DepthMergeRange &&
          BoundIsOverlap(SortObInfo.ObBound[i], SortObInfo.ObBound[j]))
      {
        NearIndexVec.push_back(j);
        processed[j] = true;
      }
      j++;
    }

    MergeIndexVec.push_back(NearIndexVec);
    NearIndexVec.clear();
    i++;
  }

  for (unsigned int i = 0; i < MergeIndexVec.size(); i++)
  {
    float DepthMulSizeX = 0, DepthMulSizeY = 0, DepthMulSizeZ = 0;
    float MinX = 2000, MinY = 2000, MaxX = 0, MaxY = 0;
    int SizeSum = 0;
    for (unsigned int j = 0; j < MergeIndexVec[i].size(); j++)
    {
      SizeSum += SortObInfo.ObSize[MergeIndexVec[i][j]];
      DepthMulSizeX += SortObInfo.ObLoc[MergeIndexVec[i][j]][0] * SortObInfo.ObSize[MergeIndexVec[i][j]];
      DepthMulSizeY += SortObInfo.ObLoc[MergeIndexVec[i][j]][1] * SortObInfo.ObSize[MergeIndexVec[i][j]];
      DepthMulSizeZ += SortObInfo.ObLoc[MergeIndexVec[i][j]][2] * SortObInfo.ObSize[MergeIndexVec[i][j]];
      MinX = MIN(MinX, SortObInfo.ObBound[MergeIndexVec[i][j]][0]);
      MinY = MIN(MinY, SortObInfo.ObBound[MergeIndexVec[i][j]][1]);
      MaxX = MAX(MaxX, SortObInfo.ObBound[MergeIndexVec[i][j]][2]);
      MaxY = MAX(MaxY, SortObInfo.ObBound[MergeIndexVec[i][j]][3]);
    }


    if ((((MaxX - MinX) / (MaxY - MinY)) >= wh_rate))
      continue;
    if (((DepthMulSizeZ / SizeSum < roise_len) || (DepthMulSizeZ / SizeSum > 15))
        && (SizeSum < roise_size))
      continue;

    outObInfo.ObLoc.emplace_back(DepthMulSizeX / SizeSum, DepthMulSizeY / SizeSum, DepthMulSizeZ / SizeSum);
    outObInfo.ObSize.push_back(SizeSum);
    outObInfo.ObBound.emplace_back(MinX, MinY, MaxX, MaxY);
  }
  return 0;
}

int ObstacleDetection::ObInfoMerge(ObstacleInformation &outObInfo)
{
  outObInfo.ObBound.clear();
  outObInfo.ObLoc.clear();
  outObInfo.ObSize.clear();

  int ret = 1;
  int i = 0;
  int rang = 0;
  int sizeSum = 0;

  while (i < OriginObInfo.ObBound.size())
  {
    int j = 0;
    std::vector<Eigen::Vector3f>::iterator ObLoc_itor = outObInfo.ObLoc.begin();
    std::vector<int>::iterator ObSize_itor = outObInfo.ObSize.begin();
    std::vector<Eigen::Vector4f>::iterator ObBound_itor = outObInfo.ObBound.begin();


    while ((j < outObInfo.ObSize.size())
           && ((abs(OriginObInfo.ObLoc[i][2] - outObInfo.ObLoc[j][2]) > DepthMergeRange)    //������?
               || (!BoundIsOverlap(OriginObInfo.ObBound[i], outObInfo.ObBound[j]))))     //û���ص�
    {
      j++;
      ObLoc_itor++;
      ObSize_itor++;
      ObBound_itor++;
    }

    if (j < outObInfo.ObSize.size())
    {

      int min_x = MIN(OriginObInfo.ObBound[i][0], outObInfo.ObBound[j][0]);
      int min_y = MIN(OriginObInfo.ObBound[i][1], outObInfo.ObBound[j][1]);
      int max_x = MAX(OriginObInfo.ObBound[i][2], outObInfo.ObBound[j][2]);
      int max_y = MAX(OriginObInfo.ObBound[i][3], outObInfo.ObBound[j][3]);
      OriginObInfo.ObBound[i][0] = min_x;
      OriginObInfo.ObBound[i][1] = min_y;
      OriginObInfo.ObBound[i][2] = max_x;
      OriginObInfo.ObBound[i][3] = max_y;

      sizeSum += OriginObInfo.ObSize[i];

      if (rang <= abs(outObInfo.ObLoc[j][0] - OriginObInfo.ObLoc[i][0]))
      {
        rang = abs(outObInfo.ObLoc[j][0] - OriginObInfo.ObLoc[i][0]);
      }

      if (OriginObInfo.ObLoc[i][2] > outObInfo.ObLoc[j][2])
      {
        OriginObInfo.ObLoc[i][0] = outObInfo.ObLoc[j][0];
        OriginObInfo.ObLoc[i][1] = outObInfo.ObLoc[j][1];
        OriginObInfo.ObLoc[i][2] = outObInfo.ObLoc[j][2];
      }


      outObInfo.ObLoc.erase(ObLoc_itor);
      outObInfo.ObBound.erase(ObBound_itor);
      outObInfo.ObSize.erase(ObSize_itor);
      rangValue.erase(rangValue.begin()+j);
    }
    else
    {
      outObInfo.ObLoc.push_back(OriginObInfo.ObLoc[i]);
      outObInfo.ObBound.push_back(OriginObInfo.ObBound[i]);
      //outObInfo.ObSize.push_back(OriginObInfo.ObSize[i]);
      outObInfo.ObSize.push_back(sizeSum);
      rangValue.push_back(rang);
      i++;
      sizeSum = 0;
      rang = 0;
    }
  }

  for (i=0; i<outObInfo.ObBound.size(); i++)
  {
    if ((outObInfo.ObBound[i][2]-outObInfo.ObBound[i][0])/
        (outObInfo.ObBound[i][3] - outObInfo.ObBound[i][1]) >= wh_rate)
    {

      outObInfo.ObLoc.erase(outObInfo.ObLoc.begin()+i);
      outObInfo.ObBound.erase(outObInfo.ObBound.begin() + i);
      outObInfo.ObSize.erase(outObInfo.ObSize.begin() + i);
    }

    if ((outObInfo.ObBound[i][2] < NOISE_LEN) &&
        (outObInfo.ObSize[i] >= NOISE_SIZE))
    {
      outObInfo.ObLoc.erase(outObInfo.ObLoc.begin() + i);
      outObInfo.ObBound.erase(outObInfo.ObBound.begin() + i);
      outObInfo.ObSize.erase(outObInfo.ObSize.begin() + i);
    }
  }

  return ret;
}

//3D世界坐标转2D图像坐标
void ObstacleDetection::Convert3Dto2D(std::vector<cv::Point3f> objectPoints, int focal, std::vector<cv::Point2f>& projectedPoints)
{
  projectedPoints.clear();
  cv::Mat intrisicMat(3, 3, cv::DataType<float>::type); //摄像机内参矩阵
  intrisicMat.at<float>(0, 0) = g_intriMax[focal][0];//9.297818e+002;//5.657809e+002;//3.677001e+002;
  intrisicMat.at<float>(1, 0) = g_intriMax[focal][1];//0;
  intrisicMat.at<float>(2, 0) = g_intriMax[focal][2];//0;

  intrisicMat.at<float>(0, 1) = g_intriMax[focal][3];//0;
  intrisicMat.at<float>(1, 1) = g_intriMax[focal][4];//9.143735e+002;//5.577476e+002;//3.70396e+002;
  intrisicMat.at<float>(2, 1) = g_intriMax[focal][5];//0;

  intrisicMat.at<float>(0, 2) = g_intriMax[focal][6];//4.881188e+002;//7.190149e+002;//3.209868e+002;
  intrisicMat.at<float>(1, 2) = g_intriMax[focal][7];//4.050004e+002;//1.121602e+002;//2.388401e+002;
  intrisicMat.at<float>(2, 2) = g_intriMax[focal][8];//1;

  cv::Mat rVec(3, 1, cv::DataType<float>::type); // 旋转向量bn
  cv::Mat rMat(3, 3, cv::DataType<float>::type);
  rMat.at<float>(0, 0) = 1; //rotatoMat[0];
  rMat.at<float>(0, 1) = 0; //rotatoMat[1];
  rMat.at<float>(0, 2) = 0; //rotatoMat[2];
  rMat.at<float>(1, 0) = 0; //rotatoMat[3];
  rMat.at<float>(1, 1) = 1; //rotatoMat[4];
  rMat.at<float>(1, 2) = 0; //rotatoMat[5];
  rMat.at<float>(2, 0) = 0; //rotatoMat[6];
  rMat.at<float>(2, 1) = 0; //rotatoMat[7];
  rMat.at<float>(2, 2) = 1; //rotatoMat[8];
  cv::Rodrigues(rMat,rVec);

  cv::Mat tVec(3, 1, cv::DataType<float>::type); // 平移向量
  tVec.at<float>(0) = 0.05;                                                                                                                                                         ;//100mm //translateMat[0]
  tVec.at<float>(1) = -0.5; //translateMat[1]
  tVec.at<float>(2) = 0; //translateMat[2]

  cv::Mat distCoeffs(5, 1, cv::DataType<float>::type);   //摄像机畸变因子
  distCoeffs.at<float>(0) = 0;//-7.9134632415085826e-001;
  distCoeffs.at<float>(1) = 0;//1.5623584435644169e+000;
  distCoeffs.at<float>(2) = 0;//-3.3916502741726508e-002;
  distCoeffs.at<float>(3) = 0;//-1.3921577146136694e-002;
  distCoeffs.at<float>(4) = 0;//1.1430734623697941e+002;

  //std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, projectedPoints);
}

void ObstacleDetection::PlaneSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtS)
{
  printf("*************plane seg points size:%d******************\n",_cloudF->points.size());

  if(_cloudF->points.size() == 0)
  {
    //判断滤波后是否存在点云数据
    return ;
  }
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> sac;
  sac.setInputCloud(_cloudF);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setModelType(pcl::SACMODEL_PLANE);
  printf("*************plane seg 1******************\n");

  sac.setDistanceThreshold(1.0);
  sac.setMaxIterations(100);
  sac.setProbability(0.95);
  sac.segment(*inliers, *coefficients);
  printf("*************plane seg 2******************\n");

  //std::vector<pcl::PointIndices> clusters;
  //clusters.clear();
  //clusters.push_back(*inliers);
  //showColoredCloud(cloudPtS, clusters);

  //去掉地面点云
  //pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudPE = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(_cloudF);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*_cloudPE);

  //    int ret = 0;
  //    ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/removePlane_pcd.pcd",*_cloudPE);
  //    std::cerr<<"保存点云是否成功："<<ret<<std::endl;

  //    std::cerr<<"filter plane seg points size"<<_cloudPE->points.size()<<std::endl;

  //计算地面平均高度
  Eigen::Vector4f min_p;
  Eigen::Vector4f max_p;
  printf("*************plane seg 3******************\n");

  pcl::getMinMax3D(*cloudPtS, *inliers, min_p, max_p);
  plane_hight_ = max_p(0);//0.5f*(min_p(0) + max_p(0));
  //    std::cerr<<"++++++++++++++++++++++++++++++++++++++"<<std::endl;
  //    std::cerr<<"plane min_z:"<<min_p(0)<<"plane max_z:"<<max_p(0)<<std::endl;
  //    printf("*************plane seg 4******************\n");

  //    int ret = 0;
  //    std::string file_name = "/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/plane_filter_points_"+std::to_string(frame_num_)+".pcd";
  //    if(frame_num_<10)
  //    {
  //        ret = pcl::io::savePCDFileASCII(file_name,*_cloudPE);
  //    }

  //    viewer.addPointCloud(_cloudPE);
  //    while (!viewer.wasStopped())
  //    {
  //        viewer.spinOnce();
  //    }

  if(DEBUG_MODE)
  {
    pcl::visualization::PCLVisualizer viewer("FilterPlaneViewer");
    viewer.addPointCloud<pcl::PointXYZ>(_cloudPE);
    //viewer.addCoordinateSystem(100, 0, 0, 0);
    while (!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
  }

}

void ObstacleDetection::RemainHookData(pcl::PointIndices cluster,pcl::PointCloud<pcl::PointXYZ> cloud_hk)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ> temp_cloud_hook;
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(cluster));

  extract.setInputCloud(_cloudPE);
  extract.setIndices(inliers_ptr);
  extract.setNegative(false);
  extract.filter(temp_cloud_hook);

  cloud_hook_ += temp_cloud_hook;
}

void ObstacleDetection::GetHookHightValue(float& outHookHight)
{
  outHookHight = hook_hight_;
  std::cout<<"hook hight is:"<<outHookHight<<std::endl;
}

int ObstacleDetection::RecongnizeHook()
{
  //pcl::visualization::PCLVisualizer viewer_hook_3d("hook_3D");
  int res = 0;
  float leaf = 0.1;
  hook_hight_ = 0.0;

  std::cout<<"hook cloud size:"<<cloud_hook_.size()<<std::endl;
  if(cloud_hook_.size() == 0)
  {
    res = 0;
    return res;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;

  vg.setInputCloud(cloud_hook_.makeShared());
  vg.setFilterFieldName("x");
  vg.setFilterLimits(_FilterPara.zLimitMin, _FilterPara.zLimitMax);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setMinimumPointsNumberPerVoxel(4);
  vg.filter(*cloudT);

  if(DEBUG_MODE)
  {
    pcl::visualization::PCLVisualizer hook_filter("hook_filter");
    hook_filter.addPointCloud<pcl::PointXYZ>(cloudT);
    //viewer.addCoordinateSystem(100, 0, 0, 0);
    while (!hook_filter.wasStopped())
    {
      hook_filter.spinOnce();
    }
  }

  std::vector<pcl::PointIndices> clusterS;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud(cloudT);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.3); //œôÁÚµãŸÛÀàŸ«¶ÈÈÝ²î£¬œôÁÚËÑË÷°ëŸ¶£¬µ¥Î»»ùÓÚÅ·Êœ¿ÕŒä
  ec.setMinClusterSize(4);
  ec.setMaxClusterSize(_SegPara.ecMaxCluster);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudT);
  ec.extract(clusterS);

  std::vector<float> hooks_distance;
  if(!clusterS.empty())
  {
    std::cout<<"clusters is not empty!"<<std::endl;
    int clusterNum = 0;
    std::string cubuStrNum = "hook";
    pcl::PointCloud<pcl::PointXYZ> temp_pointcloud;
    std::vector<ObstacleInforStr> hooks;
    std::vector<Eigen::Vector4f> hooks_min_p;
    std::vector<Eigen::Vector4f> hooks_max_p;
    hooks.clear();
    hooks_min_p.clear();
    hooks_max_p.clear();

    for(auto cluster : clusterS)
    {
      ObstacleInforStr tempOb;
      Eigen::Vector4f min_p;//3DBoxž÷¶¥µãµÄx¡¢y¡¢z×ø±ê¶ÔÓŠµÄ×îÐ¡??
      Eigen::Vector4f max_p;//3DBoxž÷¶¥µãµÄx¡¢y¡¢z×ø±ê¶ÔÓŠµÄ×îŽó??
      Eigen::Vector3f centroid_p;//3DBoxµÄÖÊÐÄ×ø??
      pcl::getMinMax3D(*cloudT, cluster, min_p, max_p);

      centroid_p(0) = 0.5f*(min_p(1) + max_p(1));	  //x??
      centroid_p(1) = 0.5f*(min_p(2) + max_p(2));	  //y??
      centroid_p(2) = 0.5f*(min_p(0) + max_p(0));	  //z??

      Eigen::Vector3f whd;
      whd(0) = max_p(1) - min_p(1);
      whd(1) = max_p(2) - min_p(2);
      whd(2) = max_p(0) - min_p(0);

      std::cout << "hook whd:\n" << whd << std::endl;
      if((whd(0)<0.1 || whd(0)>0.5)
         ||whd(1)<0.1 || whd(1)>0.5)
      {
        continue;
      }
      tempOb.box3d.width_x = whd(0);
      tempOb.box3d.height_y = whd(1);
      tempOb.box3d.depth_z = whd(2);
      tempOb.centerPt.x = centroid_p(0);
      tempOb.centerPt.y = centroid_p(1);
      tempOb.centerPt.z = centroid_p(2);

      hooks.push_back(tempOb);
      hooks_min_p.push_back(min_p);
      hooks_max_p.push_back(max_p);
      // if((whd(0)>0.1 && whd(0)<0.3)
      // &&(whd(1)>0.1 && whd(1)<0.3))
    }

    float max_dis = 0.0;
    int index = 0,i = 0;
    for(auto hk : hooks)
    {
      if(max_dis < hk.centerPt.z)
      {
        max_dis > hk.centerPt.z;
        index = i;
      }
      i++;
    }
    hook_hight_ = hooks[index].centerPt.z;

    //		viewer_hook_3d.addCube(hooks_min_p[index](0), hooks_max_p[index](0),hooks_min_p[index](1), hooks_max_p[index](1),hooks_min_p[index](2), hooks_max_p[index](2), 0, 1, 0, cubuStrNum);
    //		viewer_hook_3d.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubuStrNum);

    //		std::cout<<"print hook_box!"<<std::endl;
    //		viewer_hook_3d.addPointCloud<pcl::PointXYZ>(_cloudF);
    //		while (!viewer_hook_3d.wasStopped())
    //		{
    //			viewer_hook_3d.spinOnce();
    //		}

  }
  return res;
}

int ObstacleDetection::Obstacle3DBBox(std::vector<ObstacleInforStr>& outObjectInfors, double laser_hook_or_lift_height, double ground_height)
{
  //visualization
  int res = 0;
  int number =0;
  float max_depth =0;
  //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  //pcl::visualization::PCLVisualizer viewer("3DBbox");

  if(_clusterS.empty())
  {
    std::cerr<<"********************************clusters is empty***************************"<<std::endl;
  }


  hookInfo.centerPt.z = 0;

  if(!_clusterS.empty())
  {
    pcl::PointCloud<pcl::PointXYZ> temp_pointcloud;
    temp_pointcloud.clear();
    for(auto cluster : _clusterS)
    {
      ObstacleInforStr tempInfo;
      ObstacleInforStr tempInfo1;
      Eigen::Vector4f pcaCentroid;
      pcl::compute3DCentroid(*_cloudPE, cluster, pcaCentroid);

      Eigen::Matrix3f covariance;
      pcl::computeCovarianceMatrixNormalized(*_cloudPE, cluster, pcaCentroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

      eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
      eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
      eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

      Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
      tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
      tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
      tm_inv = tm.inverse();

      pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
      pcl::transformPointCloud(*_cloudPE, *transformedCloud, tm);

      Eigen::Vector4f min_p;//3DBox各顶点的x、y、z坐标对应的最小值
      Eigen::Vector4f max_p;//3DBox各顶点的x、y、z坐标对应的最大值
      Eigen::Vector3f centroid_p;//3DBox的质心坐标
      pcl::getMinMax3D(*_cloudPE, cluster, min_p, max_p);


      //保存上表面矩形4个坐标点
      tempInfo.pts[0].x = min_p(1);//y
      tempInfo.pts[0].y = min_p(2);//z
      tempInfo.pts[0].z = max_p(0);//x
      tempInfo.pts[1].x = max_p(1);
      tempInfo.pts[1].y = min_p(2);
      tempInfo.pts[1].z = max_p(0);
      tempInfo.pts[2].x = max_p(1);
      tempInfo.pts[2].y = max_p(2);
      tempInfo.pts[2].z = max_p(0);
      tempInfo.pts[3].x = min_p(1);
      tempInfo.pts[3].y = max_p(2);
      tempInfo.pts[3].z = max_p(0);

      Eigen::Vector3f whd;
      whd(0) = max_p(1) - min_p(1);
      whd(1) = max_p(2) - min_p(2);
      whd(2) = max_p(0) - min_p(0);

      //点云坐标系排序（x、y可能存在问题）
      //8.13 更改x\y方向定义（雷达与相机坐标系保持一致，雷达的x与y方向均相反）
      centroid_p(0) = 0.5f*(min_p(1) + max_p(1));	  //x值
      centroid_p(1) = 0.5f*(min_p(2) + max_p(2));	  //y值
      centroid_p(2) = 0.5f*(min_p(0) + max_p(0));	  //z值

      tempInfo.centerPt.x = centroid_p(0);
      tempInfo.centerPt.y = centroid_p(1);
      tempInfo.centerPt.z = centroid_p(2);

      tempInfo.box3d.width_x = whd(0);
      tempInfo.box3d.height_y = whd(1);
      tempInfo.box3d.depth_z = whd(2);
      tempInfo.obstacleType = 1;
      tempInfo.distance = sqrt(pow(centroid_p(0),2) + pow(centroid_p(1),2))-0.5*sqrt(pow(whd(0),2) + pow(whd(1),2));//增加安全区域
      tempInfo.orientation = 0;

      if(tempInfo.box3d.depth_z >max_depth)
      {
        max_depth =tempInfo.box3d.depth_z;
      }

      number++;
      //保留待识别吊钩区域内点云
      if(((tempInfo.centerPt.x>-0.5) && (tempInfo.centerPt.x<2.5))&&((tempInfo.centerPt.y>-1.5) && (tempInfo.centerPt.y<1.5)))
      {
        tempInfo.obstacleType = 0;
        //RemainHookData(cluster,temp_pointcloud);

        //去除贴近地面目标
        if(fabs(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z/2 + 0.5)) <0.8)
        {
          continue;
        }


        if((tempInfo.box3d.width_x < 0.2) && (tempInfo.box3d.height_y <0.2))
        {
          continue;
        }


        if (tempInfo.centerPt.z >hookInfo.centerPt.z)
        {
          hookInfo = tempInfo;
          hookmax_p = max_p;
          hookmin_p = min_p;
        }

        //viewer.addCube(min_p(0), max_p(0), min_p(1), max_p(1), min_p(2), max_p(2), 1, 0, 0, std::to_string(number));
        //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));
        continue;
      }
      if((tempInfo.box3d.depth_z<0.2)
         || (tempInfo.box3d.width_x<0.2)
         || (tempInfo.box3d.height_y<0.2))
      {
        continue;//2020.11.21
      }

      if((tempInfo.box3d.width_x <1.5) && (tempInfo.box3d.height_y <1.5) &&
         (fabs(ground_height - tempInfo.centerPt.z - tempInfo.box3d.depth_z/2) >3.5))//滤除悬空晃动的钩子和臂架节
      {
        continue;
      }

      //滤掉晃动的吊载
      if((m_nMode ==1) && (fabs(tempInfo.box3d.width_x - m_object_width) <1) && (fabs(tempInfo.box3d.height_y - m_object_len) <1) &&
         (fabs(tempInfo.centerPt.z + tempInfo.box3d.depth_z/2 - m_laser_hook_or_lift_height) <1.5))
      {
        continue;
      }

      //去除贴近地面目标
      if(fabs(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z/2 + 0.5)) <1.0)
      {
        continue;
      }
      //viewer.addCube(min_p(0), max_p(0), min_p(1), max_p(1), min_p(2), max_p(2), 1, 0, 0, std::to_string(number));
      //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));

      outObjectInfors.push_back(tempInfo);
      res = 1;
    }

    int ret;
    //ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/_cloudPE.pcd",*_cloudPE);
  }

  //钩子索引
  //viewer.addCube(hookmin_p(0), hookmax_p(0), hookmin_p(1), hookmax_p(1), hookmin_p(2), hookmax_p(2), 1.0, 0.0, 0.0, std::to_string(number), 0);
  //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));

  //viewer.addPointCloud<pcl::PointXYZ>(cloud_hook_.makeShared());
  //viewer.addPointCloud<pcl::PointXYZ>(_cloudPE);

  //    while (!viewer.wasStopped())
  //    {
  //        viewer.spinOnce();
  //    }

  return res;
}

int ObstacleDetection::GetPlaneHight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double& outPlaneHight)
{
  int res = 1;
  //object_upsuface_pts.clear();
  plane_hight_ = 0;

  if (cloud->points.size() == 0)
  {
    res = -1;
    return res;
  }

  _cloudT->points.clear();
  _cloudF->points.clear();
  _clusterS.clear();

  //体素化处理（滤波及下采样）
  CloudsPtFilter(cloud);

  //	//地面分割及祛除
  PlaneSeg(cloud);
  outPlaneHight = plane_hight_;

  return res;
}

int ObstacleDetection::Obstacle3DBBoxZone(std::vector<ObstacleInforStr> &outObjectInfors, pcl::PointCloud<pcl::PointXYZ>::Ptr zoneCloud, double ground_height)
{
  int res = 0;
  int number = 0;
  if (!_clusterS.empty())
  {
    for (auto cluster : _clusterS)
    {
      ObstacleInforStr tempInfo;

      Eigen::Vector4f min_p;//3DBox各顶点的x、y、z坐标对应的最小值
      Eigen::Vector4f max_p;//3DBox各顶点的x、y、z坐标对应的最大值
      Eigen::Vector3f centroid_p;//3DBox的质心坐标
      pcl::getMinMax3D(*zoneCloud, cluster, min_p, max_p);


      //保存上表面矩形4个坐标点
      tempInfo.pts[0].x = min_p(1);
      tempInfo.pts[0].y = min_p(2);
      tempInfo.pts[0].z = max_p(0);
      tempInfo.pts[1].x = max_p(1);
      tempInfo.pts[1].y = min_p(2);
      tempInfo.pts[1].z = max_p(0);
      tempInfo.pts[2].x = max_p(1);
      tempInfo.pts[2].y = max_p(2);
      tempInfo.pts[2].z = max_p(0);
      tempInfo.pts[3].x = min_p(1);
      tempInfo.pts[3].y = max_p(2);
      tempInfo.pts[3].z = max_p(0);

      Eigen::Vector3f whd;
      whd(0) = max_p(1) - min_p(1);
      whd(1) = max_p(2) - min_p(2);
      whd(2) = max_p(0) - min_p(0);

      //点云坐标系排序（x、y可能存在问题）
      //8.13 更改x\y方向定义（雷达与相机坐标系保持一致，雷达的x与y方向均相反）
      centroid_p(0) = 0.5f*(min_p(1) + max_p(1));	  //x值
      centroid_p(1) = 0.5f*(min_p(2) + max_p(2));	  //y值
      centroid_p(2) = 0.5f*(min_p(0) + max_p(0));	  //z值

      tempInfo.centerPt.x = centroid_p(0);
      tempInfo.centerPt.y = centroid_p(1);
      tempInfo.centerPt.z = centroid_p(2);

      tempInfo.box3d.width_x = whd(0);
      tempInfo.box3d.height_y = whd(1);
      tempInfo.box3d.depth_z = whd(2);
      tempInfo.obstacleType = 1;
      tempInfo.distance = sqrt(pow(centroid_p(0), 2) + pow(centroid_p(1), 2)) - 0.5*sqrt(pow(whd(0), 2) + pow(whd(1), 2));//增加安全区域
      tempInfo.orientation = 0;

      number++;
      if ((tempInfo.box3d.depth_z<0.2)
          || (tempInfo.box3d.width_x<0.2)
          || (tempInfo.box3d.height_y<0.2))
      {
        continue;//2020.11.21
      }

      if ((tempInfo.box3d.width_x <2) && (tempInfo.box3d.height_y <2) && (fabs(ground_height - tempInfo.centerPt.z - tempInfo.box3d.depth_z / 2) >1.5))//滤除悬空晃动的钩子和臂架节
      {
        continue;
      }

      //去除贴近地面目标
      if (fabs(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2 + 0.5)) <1.0)
      {
        continue;
      }
      outObjectInfors.push_back(tempInfo);
      res = 1;
      allMin_p.push_back(min_p);
      allMax_p.push_back(max_p);
    }

    int ret;
    //ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/_cloudPE.pcd",*_cloudPE);
  }

  return res;
}

void ObstacleDetection::getHookHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr &hookCloud, double ground_height)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(hookCloud);//设置输入点云
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(1.0);//紧邻点聚类精度容差，紧邻搜索半径，单位基于欧式空间
  ec.setMinClusterSize(_SegPara.ecMinCluster);
  ec.setMaxClusterSize(_SegPara.ecMaxCluster);
  ec.setSearchMethod(tree);
  ec.setInputCloud(hookCloud);
  ec.extract(_clusterS);

  int number = 0;

  hookInfo.centerPt.z =0;

  if (!_clusterS.empty())
  {
    for (auto cluster : _clusterS)
    {
      ObstacleInforStr tempInfo;

      Eigen::Vector4f min_p;//3DBox各顶点的x、y、z坐标对应的最小值
      Eigen::Vector4f max_p;//3DBox各顶点的x、y、z坐标对应的最大值
      Eigen::Vector3f centroid_p;//3DBox的质心坐标
      pcl::getMinMax3D(*hookCloud, cluster, min_p, max_p);


      //保存上表面矩形4个坐标点
      tempInfo.pts[0].x = min_p(1);
      tempInfo.pts[0].y = min_p(2);
      tempInfo.pts[0].z = max_p(0);
      tempInfo.pts[1].x = max_p(1);
      tempInfo.pts[1].y = min_p(2);
      tempInfo.pts[1].z = max_p(0);
      tempInfo.pts[2].x = max_p(1);
      tempInfo.pts[2].y = max_p(2);
      tempInfo.pts[2].z = max_p(0);
      tempInfo.pts[3].x = min_p(1);
      tempInfo.pts[3].y = max_p(2);
      tempInfo.pts[3].z = max_p(0);

      Eigen::Vector3f whd;
      whd(0) = max_p(1) - min_p(1);
      whd(1) = max_p(2) - min_p(2);
      whd(2) = max_p(0) - min_p(0);

      //点云坐标系排序（x、y可能存在问题）
      //8.13 更改x\y方向定义（雷达与相机坐标系保持一致，雷达的x与y方向均相反）
      centroid_p(0) = 0.5f*(min_p(1) + max_p(1));	  //x值
      centroid_p(1) = 0.5f*(min_p(2) + max_p(2));	  //y值
      centroid_p(2) = 0.5f*(min_p(0) + max_p(0));	  //z值

      tempInfo.centerPt.x = centroid_p(0);
      tempInfo.centerPt.y = centroid_p(1);
      tempInfo.centerPt.z = centroid_p(2);

      tempInfo.box3d.width_x = whd(0);
      tempInfo.box3d.height_y = whd(1);
      tempInfo.box3d.depth_z = whd(2);
      tempInfo.obstacleType = 1;
      tempInfo.distance = sqrt(pow(centroid_p(0), 2) + pow(centroid_p(1), 2)) - 0.5*sqrt(pow(whd(0), 2) + pow(whd(1), 2));//增加安全区域
      tempInfo.orientation = 0;

      number++;

      //空钩去掉低于吊载的目标
      if ((m_nMode ==0) && ((ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <m_object_height))
      {
        std::cerr <<"ground_target ="<<(ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <<std::endl;
        continue;
      }

      //吊载模式去掉低于就位点的目标
      if ((m_nMode ==1) && ((ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <m_goods_height))
      {
        std::cerr <<"ground_target ="<<(ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <<std::endl;
        continue;
      }

      //去掉紧贴悬空地面的目标
      if ((ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2)) <1.0)
      {
        std::cerr <<"ground_target ="<<(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2)) <<std::endl;
        continue;
      }

      //去掉紧贴地面上表面小于某个阈值的目标
      if ((ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <1.0)
      {
        std::cerr <<"ground_target ="<<(ground_height - tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2) <<std::endl;
        continue;
      }

      if ((tempInfo.box3d.width_x < 0.2) && (tempInfo.box3d.height_y <0.2))
      {
        continue;
      }

      if((m_nMode ==0) && ((tempInfo.box3d.width_x >1) || (tempInfo.box3d.height_y >1)))//空钩模式
      {
        continue ;
      }

      if((m_nMode ==0) && (tempInfo.box3d.depth_z > 1.5))//空钩模式
      {
        continue ;
      }

      if((m_nMode ==1) && ((tempInfo.box3d.width_x < m_object_width*0.5) || ((tempInfo.box3d.width_x < m_object_len*0.5))))//吊载模式
      {
        //std::cerr <<"222222222222222"<<std::endl;
        continue;
      }

      if ((tempInfo.centerPt.z >hookInfo.centerPt.z))
      {
        hookInfo = tempInfo;
        hookmax_p = max_p;
        hookmin_p = min_p;
      }

    }
  }
  _clusterS.clear();
}

//区域得到3D box
int ObstacleDetection::Obstacle3DBBoxZoneHook(std::vector<ObstacleInforStr> &outObjectInfors, pcl::PointCloud<pcl::PointXYZ>::Ptr zoneCloud, double ground_height)
{
  int res = 0;
  //可视化
  //pcl::visualization::PCLVisualizer viewer("3DBbox");
  /*viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();*/

  ObstacleInforStr hookInfo;
  hookInfo.centerPt.z = 0;

  int number = 0;
  if (!_clusterS.empty())
  {
    for (auto cluster : _clusterS)
    {
      ObstacleInforStr tempInfo;

      Eigen::Vector4f min_p;//3DBox各顶点的x、y、z坐标对应的最小值
      Eigen::Vector4f max_p;//3DBox各顶点的x、y、z坐标对应的最大值
      Eigen::Vector3f centroid_p;//3DBox的质心坐标
      pcl::getMinMax3D(*zoneCloud, cluster, min_p, max_p);

      //保存上表面矩形4个坐标点
      tempInfo.pts[0].x = min_p(1);//y
      tempInfo.pts[0].y = min_p(2);//z
      tempInfo.pts[0].z = max_p(0);//x
      tempInfo.pts[1].x = max_p(1);
      tempInfo.pts[1].y = min_p(2);
      tempInfo.pts[1].z = max_p(0);
      tempInfo.pts[2].x = max_p(1);
      tempInfo.pts[2].y = max_p(2);
      tempInfo.pts[2].z = max_p(0);
      tempInfo.pts[3].x = min_p(1);
      tempInfo.pts[3].y = max_p(2);
      tempInfo.pts[3].z = max_p(0);

      Eigen::Vector3f whd;
      whd(0) = max_p(1) - min_p(1);
      whd(1) = max_p(2) - min_p(2);
      //whd(2) = (max_p(0) - min_p(0))>4.5?4.5:(max_p(0) - min_p(0));
      whd(2) = max_p(0) - min_p(0);

      //点云坐标系排序（x、y可能存在问题）
      //8.13 更改x\y方向定义（雷达与相机坐标系保持一致，雷达的x与y方向均相反）
      centroid_p(0) = 0.5f*(min_p(1) + max_p(1));	  //x值
      centroid_p(1) = 0.5f*(min_p(2) + max_p(2));	  //y值
      centroid_p(2) = 0.5f*(min_p(0) + max_p(0));	  //z值

      tempInfo.centerPt.x = centroid_p(0);
      tempInfo.centerPt.y = centroid_p(1);
      tempInfo.centerPt.z = centroid_p(2);

      tempInfo.box3d.width_x = whd(0);
      tempInfo.box3d.height_y = whd(1);
      tempInfo.box3d.depth_z = whd(2);
      tempInfo.obstacleType = 1;
      tempInfo.distance = sqrt(pow(centroid_p(0), 2) + pow(centroid_p(1), 2)) - 0.5*sqrt(pow(whd(0), 2) + pow(whd(1), 2));//增加安全区域
      tempInfo.orientation = 0;

      number++;
      //保留待识别吊钩区域内点云
      if (((tempInfo.centerPt.x>-3.5) && (tempInfo.centerPt.x<3.5))&& ((tempInfo.centerPt.y>-3.5) && (tempInfo.centerPt.y<3.5)))
      {
        tempInfo.obstacleType = 0;
        //RemainHookData(cluster,temp_pointcloud);

        //去除贴近地面目标
        if (fabs(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2 + 0.5)) <0.8)
        {
          continue;
        }

        if ((tempInfo.box3d.width_x < 0.2) && (tempInfo.box3d.height_y <0.2))
        {
          continue;
        }


        if (tempInfo.centerPt.z >hookInfo.centerPt.z)
        {
          hookInfo = tempInfo;
          hookmax_p = max_p;
          hookmin_p = min_p;
        }

        //viewer.addCube(min_p(0), max_p(0), min_p(1), max_p(1), min_p(2), max_p(2), 1, 0, 0, std::to_string(number));
        //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));
        continue;
      }
      if ((tempInfo.box3d.depth_z<0.2)
          || (tempInfo.box3d.width_x<0.2)
          || (tempInfo.box3d.height_y<0.2))
      {
        continue;//2020.11.21
      }

      if ((tempInfo.box3d.width_x <2) && (tempInfo.box3d.height_y <2) && (fabs(ground_height - tempInfo.centerPt.z - tempInfo.box3d.depth_z / 2) >3.5))//滤除悬空晃动的钩子和臂架节
      {
        continue;
      }

      //滤掉晃动的吊载
      if((m_nMode ==1) && (fabs(tempInfo.box3d.width_x - m_object_width) <0.8) && (fabs(tempInfo.box3d.height_y - m_object_len) <0.8)
         && (fabs(tempInfo.centerPt.z + tempInfo.box3d.depth_z/2 - m_laser_hook_or_lift_height) <1.0))
      {
        continue;
      }

      //去除贴近地面目标
      if (fabs(ground_height - (tempInfo.centerPt.z + tempInfo.box3d.depth_z / 2 + 0.5)) <1.0)
      {
        //continue;
      }

      //viewer.addCube(min_p(0), max_p(0), min_p(1), max_p(1), min_p(2), max_p(2), 1, 0, 0, std::to_string(number));
      //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));

      outObjectInfors.push_back(tempInfo);
      res = 1;
      allMin_p.push_back(min_p);
      allMax_p.push_back(max_p);
    }

    int ret;
    //ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/_cloudPE.pcd",*_cloudPE);
  }

  //钩子索引
  //viewer.addCube(hookmin_p(0), hookmax_p(0), hookmin_p(1), hookmax_p(1), hookmin_p(2), hookmax_p(2), 1.0, 0.0, 0.0, std::to_string(number), 0);
  //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(number));

  ////viewer.addPointCloud<pcl::PointXYZ>(cloud_hook_.makeShared());

  //    viewer.addPointCloud<pcl::PointXYZ>(_cloudPE);

  //    while (!viewer.wasStopped())
  //    {
  //        viewer.spinOnce();
  //    }
  return res;
}

//区域点云聚类
void ObstacleDetection::zoneCluster(std::vector<ObstacleInforStr> &outObinfors, double ground_height)
{
  //区域聚类提取障碍物信息
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
  float y_step;
  float z_step;
  y_step =3;
  z_step =3;
  Eigen::Vector4f min_p;//3DBox各顶点的x y z坐标对应的最小值
  Eigen::Vector4f max_p;//3DBox各顶点的x y z坐标对应的最大值
  Eigen::Vector3f centroid_p;//3DBOX的质心坐标
  //pcl::getMinMax3D(*cloud,min_p, max_p);
  pcl::getMinMax3D(*_cloudPE, min_p, max_p);

  for (float y = min_p[1]; y <max_p[1]; y += y_step)
  {
    for (float z = min_p[2]; z < max_p[2]; z += z_step)
    {
      Eigen::Vector4f min_point(0, y, z, 1.0);
      Eigen::Vector4f max_point(100, y + y_step, z + z_step, 1.0);
      clipper.setMin(min_point);
      clipper.setMax(max_point);
      clipper.setInputCloud(_cloudObs);
      clipper.filter(*cloud_filtered);
      clipper.setNegative(false);//默认为false

      //区域聚类
      if (cloud_filtered->points.size() == 0)
      {
        //判断滤波后是否存在点云
        continue;
      }

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);//设置输入点云
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(1.0);//紧邻点聚类精度容差，紧邻搜索半径，单位基于欧式空间
      ec.setMinClusterSize(_SegPara.ecMinCluster);
      ec.setMaxClusterSize(_SegPara.ecMaxCluster);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_filtered);
      ec.extract(_clusterS);

      //提取包围盒
      Obstacle3DBBoxZone(outObinfors, cloud_filtered, ground_height);
      _clusterS.clear();
      cloud_filtered->clear();
    }
  }
}

//把除吊钩区域外的点云加上滤波处理
void ObstacleDetection::filterZoneCluser(std::vector<ObstacleInforStr> &outObinfors, double ground_height)
{
  //除吊钩区域以外的区域做统计滤波去除离群点
  pcl::CropBox<pcl::PointXYZ> clipper1;//要包含的头文件，在官方文档里
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //Eigen::Vector4f min_point(0, -1.5, -1.5, 1.0);
  //Eigen::Vector4f max_point(100, 1.5, 1.5, 1.0);

  Eigen::Vector4f min_point(0, hookZoneX_min, hookZoneY_min, 1.0);
  Eigen::Vector4f max_point(100, hookZoneX_max, hookZoneY_max, 1.0);

  pcl::CropBox<pcl::PointXYZ> clipper2;//要包含的头文件，在官方文档里
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  clipper2.setMin(min_point);
  clipper2.setMax(max_point);
  clipper2.setInputCloud(_cloudPE);
  clipper2.setNegative(false);//默认为false
  clipper2.filter(*box_cloud2);//吊钩区域点云

  clipper1.setMin(min_point);
  clipper1.setMax(max_point);
  clipper1.setInputCloud(_cloudPE);
  clipper1.setNegative(true);//默认为false
  clipper1.filter(*box_cloud);//除吊钩区域以外的点云


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(box_cloud);
  sor.setMeanK(50); //K近邻搜索点个数
  sor.setStddevMulThresh(1.0); //标准差倍数
  sor.setNegative(false); //保留未滤波点（内点）
  sor.filter(*cloud_filter);  //吊钩区域以外的统计滤波以后的点云

  pcl::PointCloud<pcl::PointXYZ>::Ptr useCloud(new pcl::PointCloud<pcl::PointXYZ>);

  *useCloud = (*box_cloud2) + (*cloud_filter);//吊钩区域的点云以及统计滤波后的点云拼接

  //区域聚类提取障碍物信息
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
  float y_step = zoneCluster_y_step;
  float z_step = zoneCluster_z_step;
  Eigen::Vector4f min_p;//3DBox各顶点的x y z坐标对应的最小值
  Eigen::Vector4f max_p;//3DBox各顶点的x y z坐标对应的最大值
  Eigen::Vector3f centroid_p;//3DBOX的质心坐标
  //pcl::getMinMax3D(*cloud,min_p, max_p);
  pcl::getMinMax3D(*useCloud, min_p, max_p);

  for (float y = min_p[1]; y <max_p[1]; y += y_step)
  {
    for (float z = min_p[2]; z < max_p[2]; z += z_step)
    {
      Eigen::Vector4f min_point(0, y, z, 1.0);
      Eigen::Vector4f max_point(100, y + y_step, z + z_step, 1.0);
      clipper.setMin(min_point);
      clipper.setMax(max_point);
      clipper.setInputCloud(useCloud);
      clipper.filter(*cloud_filtered);
      clipper.setNegative(false);//默认为false

      //区域聚类
      if (cloud_filtered->points.size() == 0)
      {
        //判断滤波后是否存在点云
        continue;
      }

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);//设置输入点云
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(1.0);//紧邻点聚类精度容差，紧邻搜索半径，单位基于欧式空间
      ec.setMinClusterSize(_SegPara.ecMinCluster);
      ec.setMaxClusterSize(_SegPara.ecMaxCluster);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud_filtered);
      ec.extract(_clusterS);

      //提取包围盒
      Obstacle3DBBoxZone(outObinfors, cloud_filtered, ground_height);
      _clusterS.clear();
      cloud_filtered->clear();
    }
  }
}

//求吊钩区域雷达对地高
void ObstacleDetection::zoneGetPlaneHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &outPlaneHight)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropBox<pcl::PointXYZ> clipper;//要包含的头文件，在官方文档里
  Eigen::Vector4f min_point(0, hookZoneX_min, hookZoneY_min, 1.0);
  Eigen::Vector4f max_point(100, hookZoneX_max, hookZoneY_max, 1.0);
  clipper.setMin(min_point);
  clipper.setMax(max_point);
  clipper.setInputCloud(cloud);
  clipper.filter(*cloud_filtered);
  clipper.setNegative(false);//默认为false

  //计算区域地面高度
  Eigen::Vector4f min_p;
  Eigen::Vector4f max_p;
  printf("*************plane seg 3******************\n");

  pcl::getMinMax3D(*cloud_filtered, min_p, max_p);
  outPlaneHight = max_p(0);
}

int ObstacleDetection::ObstacleDetected(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CraneStateStr craneState,std::vector<ObstacleInforStr>& outObInfors,
                                        double &laser_hook_or_lift_height, double ground_height, double crane_hook_height)
{
  int res = 1;
  frame_num_++;
  //object_upsuface_pts.clear();
  plane_hight_ = 0;

  if (cloud->points.size() == 0)
  {
    res = -1;
    return res;
  }

  std::vector<Obstacle_state_Information> obstacles_in_livox_coordinate;
  std::vector<ObstacleInforStr> realOutObinfors;
  realOutObinfors.clear();
  outObInfors.clear();
  obstacles_in_livox_coordinate.clear();
  _cloudT->points.clear();
  _cloudF->points.clear();
  _clusterS.clear();

  float zoneGroundHeight =0;
  //zoneGetPlaneHeight(cloud,zoneGroundHeight);

  //体素化处理（滤波及下采样）
  CloudsPtFilter(cloud);

  //	//地面分割及祛除
  PlaneSeg(cloud);

  //    ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/treeRemovePlaneCloud.pcd",*_cloudPE);

  pcl::CropBox<pcl::PointXYZ> clipper1;//要包含的头文件，在官方文档里
  Eigen::Vector4f min_point(0, -1.5, -1.5, 1.0);
  Eigen::Vector4f max_point(100, 1.5, 1.5, 1.0);

  pcl::CropBox<pcl::PointXYZ> clipper2;//要包含的头文件，在官方文档里
  pcl::PointCloud<pcl::PointXYZ>::Ptr hookCloud(new pcl::PointCloud<pcl::PointXYZ>);
  clipper2.setMin(min_point);
  clipper2.setMax(max_point);
  clipper2.setInputCloud(_cloudPE);
  clipper2.setNegative(false);//默认为false
  clipper2.filter(*hookCloud);//吊钩区域点云

  clipper1.setMin(min_point);
  clipper1.setMax(max_point);
  clipper1.setInputCloud(_cloudPE);
  clipper1.setNegative(true);//默认为false
  clipper1.filter(*_cloudObs);//地面点云去掉吊钩区域的点云

  //    pcl::visualization::PCLVisualizer viewer("3DBbox");
  //    viewer.addPointCloud<pcl::PointXYZ>(_cloudObs);

  //    while (!viewer.wasStopped())
  //    {
  //        viewer.spinOnce();
  //    }

  getHookHeight(hookCloud, ground_height);

  if(zoneMode)
  {
    zoneCluster(outObInfors, ground_height);
    //filterZoneCluser(outObinfors, ground_height);
    //        pcl::visualization::PCLVisualizer viewer("3DBbox");

    //        viewer.addCube(hookmin_p(0), hookmax_p(0), hookmin_p(1), hookmax_p(1), hookmin_p(2), hookmax_p(2), 0.0, 1.0, 0.0, "hook", 0);
    //        viewer.addCube(0, 85, -3.5, 3.5, -3.5, 3.5, 0.0, 1.0, 0.0, "hook2", 0);

    //        for (int i = 0; i < allMax_p.size();i++)
    //        {
    //            viewer.addCube(allMin_p[i](0), allMax_p[i](0), allMin_p[i](1), allMax_p[i](1), allMin_p[i](2), allMax_p[i](2), 1.0, 0.0, 0.0, std::to_string(i),0);
    //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(i));
    //        }

    //viewer.addPointCloud<pcl::PointXYZ>(hookCloud);
    // viewer.addPointCloud<pcl::PointXYZ>(_cloudPE);//yt

    //        while (!viewer.wasStopped())
    //        {
    //            viewer.spinOnce();
    //        }
  }
  else
  {
    //	//聚类
    CloudsPtSeg(cloud);

    Obstacle3DBBox(outObInfors, laser_hook_or_lift_height, ground_height);
    //Obstacle3DBBox1(obstacles_in_livox_coordinate);
  }
  if (int(outObInfors.size()) == 0)
  {
    res = 0;
    return res;
  }

  float max_depth1 =0;


#if 1
  for(int num = 0; num < outObInfors.size(); num++)
  {
    //吊钩/吊载物/其他障碍物判断
    //中心点半径范围内的障碍物，默认为吊钩/吊载，半径范围2米
    //吊载底面高于地面一定距离，也可认为是吊载物
    if((outObInfors[num].centerPt.x>-2.5 && outObInfors[num].centerPt.x<2.5)
       && (outObInfors[num].centerPt.y>-2.5 && outObInfors[num].centerPt.y<2.5))
    {
      outObInfors[num].obstacleType = 0;
    }
    else
    {
      outObInfors[num].obstacleType = 1;
    }

    //2D图像投影映射/标注（需要判断雷达映射点是否在图像区域内）
    std::vector<cv::Point3f> objectPts;
    objectPts.clear();

    for(int i = 0; i<4; i++)
    {
      outObInfors[num].pts[i].x *= -1;//雷达的x轴与相机的x轴相反
      objectPts.push_back(outObInfors[num].pts[i]);
    }
    std::vector<cv::Point2f> projectedPoints;
    Convert3Dto2D(objectPts,craneState.focus,projectedPoints);

    outObInfors[num].rect2D.x = int(projectedPoints[0].x);
    outObInfors[num].rect2D.y = int(projectedPoints[0].y);
    outObInfors[num].rect2D.width = abs(int(projectedPoints[1].x-projectedPoints[0].x));
    outObInfors[num].rect2D.height = abs(int(projectedPoints[2].y-projectedPoints[1].y));
    //补充边界判断（超出图像显示区域）
    if(((outObInfors[num].rect2D.x+outObInfors[num].rect2D.width) < 0 )
       || ((outObInfors[num].rect2D.y+outObInfors[num].rect2D.height) < 0 )
       || (outObInfors[num].rect2D.x > 1080)
       || (outObInfors[num].rect2D.y > 720))
    {
      outObInfors[num].rect2D.x = 0;
      outObInfors[num].rect2D.y = 0;
      outObInfors[num].rect2D.width = 0;
      outObInfors[num].rect2D.height = 0;
    }
    if(outObInfors[num].rect2D.x<0 && (outObInfors[num].rect2D.x+outObInfors[num].rect2D.width)<1080)
    {
      outObInfors[num].rect2D.x = 0;
      outObInfors[num].rect2D.width = outObInfors[num].rect2D.width+outObInfors[num].rect2D.x;
    }
    if(outObInfors[num].rect2D.y<0 && (outObInfors[num].rect2D.y+outObInfors[num].rect2D.height)<720)
    {
      outObInfors[num].rect2D.y = 0;
      outObInfors[num].rect2D.height = outObInfors[num].rect2D.height+outObInfors[num].rect2D.y;
    }

    //雷达坐标系与臂尖坐标系转换
    PointType tempPt;
    PositionTranslate(outObInfors[num].centerPt,craneState.bcAngle * M_PI / 180,tempPt);

    //根据臂尖坐标系坐标过滤臂架
    if (tempPt.x >-3 && tempPt.x <3 && tempPt.y >-3 && tempPt.y <3)
    {
      continue;
    }

    outObInfors[num].centerPt.x = tempPt.x;
    outObInfors[num].centerPt.y = tempPt.y;
    outObInfors[num].centerPt.z = tempPt.z;

    realOutObinfors.push_back(outObInfors[num]);

    max_depth1 =outObInfors[num].box3d.depth_z > max_depth1 ? outObInfors[num].box3d.depth_z : max_depth1;
  }

#endif


  if(outObInfors.size() >=1)
  {
    h_mutex_.lock();
    obstacle_infors_.clear();
    //obstacle_infors_.assign(outObInfors.begin(),outObInfors.end());
    obstacle_infors_.assign(realOutObinfors.begin(), realOutObinfors.end());
    realOutObinfors.clear();
    h_mutex_.unlock();
  }

  //提取钩子区域的雷达对地高
  //    if(bThreadComputePosition ==false)
  //    {
  //        return res;
  //    }

  //std::cerr <<"g_dHookHeightCali = " << g_dHookHeightCali << std::endl;
  laser_hook_or_lift_height = ground_height - fabs(hookInfo.centerPt.z) + 0.5 + hookInfo.box3d.depth_z/2;//上表面
  double hook_Height_Bottom = ground_height - fabs(hookInfo.centerPt.z) + 0.5 - hookInfo.box3d.depth_z/2;//下表面
  //    g_dHookHeightMiddle = ground_height - fabs(hookInfo.centerPt.z) + 0.5;//中心点

  if(laser_hook_or_lift_height < 0 || hook_Height_Bottom <0)
  {
    std::cerr << "g_dHookHeight = "<< laser_hook_or_lift_height << std::endl;
    std::cerr <<" g_dHookHeightBottom = " << hook_Height_Bottom <<std::endl;
    std::cerr << "the hook height is error" << std::endl;
    return 0;
  }

  if(m_nMode ==0)//空钩
  {
    if((ground_height - hook_Height_Bottom) < 16)//此时空钩情况下，吊钩跟卷扬绳一体，且此时钩子全部被探测到
    {
      laser_hook_or_lift_height = hook_Height_Bottom - downHookThresh;//取下表面
    }
    else
    {
      laser_hook_or_lift_height = laser_hook_or_lift_height - upHookThresh;//取上表面
    }

    if(fabs(laser_hook_or_lift_height - crane_hook_height) > 5)
    {
      laser_hook_or_lift_height = crane_hook_height;
      m_dpreHookHeight = crane_hook_height;
      //std::cerr << "the empty hook height maybe is error" << std::endl;
    }
  }
  else//吊载 吊载可能跟索具 钩子连到一起 所以只能取下表面  但是也有可能 下表面雷达探测不到
  {
    laser_hook_or_lift_height = hook_Height_Bottom - downHookThresh;

    if(fabs(laser_hook_or_lift_height - (crane_hook_height - m_rope_height - m_object_height)) > 5)
    {
      laser_hook_or_lift_height = crane_hook_height  - m_rope_height - m_object_height;
      m_dpreHookHeight = crane_hook_height  - m_rope_height - m_object_height;

      std::cerr << "the none empty hook height maybe is error" << std::endl;
    }
  }
  m_dpreHookHeight = laser_hook_or_lift_height;
  m_laser_hook_or_lift_height = laser_hook_or_lift_height;

  //障碍物坐标臂尖转世界
  //obstacleTranstion();

  return res;
}

//获取障碍物检测结果
void ObstacleDetection::GetObstaclesValue(std::vector<ObstacleInforStr>& outObjectInfors)
{
  h_mutex_.lock();
  outObjectInfors.clear();
  //std::cerr << "the obstacle_infors_ size is = " << obstacle_infors_.size() << std::endl;
  outObjectInfors.assign(obstacle_infors_.begin(),obstacle_infors_.end());
  //obstacle_infors_.clear();
  h_mutex_.unlock();
}

//读取PCD文件并可视化
void read_pcd(std::string pcd_path,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file rabbit.pcd\n");
    //return(-1);
  }
  //std::cout << cloud->points.size() << std::endl;
  //qDebug() << cloud->points.size();
  //可视化
  //pcl::visualization::CloudViewer viewer("cloud viewer");

  // pcl::visualization::PCLVisualizer viewer("cloud viewer");
  // viewer.addPointCloud<pcl::PointXYZ>(cloud);
  // while (!viewer.wasStopped())
  // {
  // 	viewer.spinOnce();
  // }
  // system("pause");
}

void showColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<pcl::PointIndices> clusters)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters.empty())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

    srand(static_cast<unsigned int> (time(nullptr)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters.size(); i_segment++)
    {
      colors.push_back(static_cast<unsigned char> (rand() % 256));
      colors.push_back(static_cast<unsigned char> (rand() % 256));
      colors.push_back(static_cast<unsigned char> (rand() % 256));
    }

    colored_cloud->width = input->width;
    colored_cloud->height = input->height;
    colored_cloud->is_dense = input->is_dense;
    for (size_t i_point = 0; i_point < input->points.size(); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input->points[i_point].data);
      point.y = *(input->points[i_point].data + 1);
      point.z = *(input->points[i_point].data + 2);
      point.r = 255;
      point.g = 255;
      point.b = 255;
      colored_cloud->points.push_back(point);
    }

    std::vector< pcl::PointIndices >::const_iterator i_segment;
    auto next_color = 0;
    for (i_segment = clusters.begin(); i_segment != clusters.end(); i_segment++)
    {
      //std::vector<int>::iterator i_point;
      std::cout << "i_segment" << endl;
      int pt = 0;
      for (std::vector<int>::const_iterator i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
      {

        if(pt++ == 0)
        {
          std::cout << "i_segment_color" << endl;
        }
        int index;
        index = *i_point;
        colored_cloud->points[index].r = 255;//colors[3 * next_color];
        colored_cloud->points[index].g = 0;//colors[3 * next_color + 1];
        colored_cloud->points[index].b = 0;//colors[3 * next_color + 2];
      }
      next_color++;
    }
  }
  //return (colored_cloud);

  pcl::visualization::PCLVisualizer viewer("Seg Viewer");
  viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud);
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
}

void ObstacleDetection::PositionTranslate(const PointType radarPos,const float armAngle,PointType &end_pose)
{
  Eigen::Vector3f point_position(radarPos.x, radarPos.y, radarPos.z);//基准坐标系赋值？
  //雷达中心点到云台固定安装面中心点坐标转换
  double ptz_rotate_offset = -6.4; //degree
  Eigen::Isometry3f tf_1 = Eigen::Isometry3f(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
  tf_1.translation() = Eigen::Vector3f ( -0.0485,0.0252,0.436 );//平移向量

  Eigen::Vector3f end_position1 = tf_1 * point_position;

  //云台固定安装面中心点到臂尖坐标系转换 yt
  Eigen::Isometry3f tf_2 = Eigen::Isometry3f::Identity();
  //110h763
  tf_2.translation() = Eigen::Vector3f (0.08,-0.7,-0.95 );
  //220h8
  //tf_2.translation() = Eigen::Vector3f (1.12,-0.35,-0.05 );
  tf_2 = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI/2 + crane_parameter_->crane_luff_Angle_ * M_PI / 180,
                                             Eigen::Vector3f::UnitY())
                           *Eigen::AngleAxisf(-M_PI/2,
                                              Eigen::Vector3f::UnitZ()));

  //Eigen::Vector3f end_position = tf_2 * tf_1 * point_position; tzr
  Eigen::Vector3f end_position = tf_1 * tf_2 * point_position;
  end_pose.x = end_position[0];
  end_pose.y = end_position[1];
  end_pose.z = end_position[2];

  //    std::cerr << "hookarmhear_x ="<<end_pose.x<<std::endl;
  //    std::cerr << "hookarmhear_y ="<<end_pose.y<<std::endl;
  //    std::cerr << "hookarmhear_z ="<<end_pose.z<<std::endl;
}

void ObstacleDetection::obstacleTranstion()
{
  std::vector<ObstacleInforStr> collision_objects;
  //    for (unsigned int j = 0; j < last_added_collision_objects_.size(); j++)
  //    {
  //        auto_crane_manipulation_.remove_collision_object("temp_ob_" + std::to_string(j));
  //    }

  //added the collision objects detected from the scene monitor
  GetObstaclesValue(collision_objects);
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!collision_objects size: " << collision_objects.size() << std::endl;
  Eigen::Isometry3f main_link_end_to_world_tf;
  if (auto_crane_manipulation_1.get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_*M_PI/180,
                                                              crane_parameter_->crane_luff_Angle_*M_PI/180,
                                                              crane_parameter_->crane_arm_to_ground_height_,
                                                              main_link_end_to_world_tf))
  {
    //transform the collision objects to world frame
    for (unsigned int i = 0; i < collision_objects.size(); i++)
    {
      Eigen::Isometry3f object_pose = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
                                                        *Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
      object_pose.translation() = Eigen::Vector3f(collision_objects[i].centerPt.x,
                                                  collision_objects[i].centerPt.y,
                                                  collision_objects[i].centerPt.z);
      object_pose = main_link_end_to_world_tf * object_pose;
      collision_objects[i].centerPt.x = object_pose.translation()[0];
      collision_objects[i].centerPt.y = object_pose.translation()[1];
      collision_objects[i].centerPt.z = object_pose.translation()[2];
    }
  }
  printf("hello\n");
}

int ObstacleDetection::Obstacle3DBBox1(std::vector<Obstacle_state_Information>& obstacles_in_livox_coordinate)
{
  //visualization
  int res = 0;

  //    if(DEBUG_MODE)
  //    {
  //        pcl::visualization::PCLVisualizer viewer("3D-Bbox");
  //    }
  //    pcl::visualization::PCLVisualizer viewer("3DBbox");

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

  std::string cubuStrNum;
  int clusterNum = 0;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  if(!_clusterS.empty())
  {
    pcl::PointCloud<pcl::PointXYZ> temp_pointcloud;
    int number = 0;                                      //DEBUG时使用的显示不同窗体而已
    for (auto cluster : _clusterS) //pcl::PointIndices 写到这
    {
      number += 1;
      pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
      //            std::cout << sizeof(cluster);
      extract.setInputCloud(_cloudPE);//(_cloudPE);
      extract.setIndices(cluster_ptr);    //pcl::PointIndices::ptr 写到这
      extract.setNegative(false);
      extract.filter(*cloud_plane);            //拿到数据
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cloud_plane);
      feature_extractor.compute();

      std::vector<float> moment_of_inertia;
      std::vector<float> eccentricity;
      pcl::PointXYZ min_point_AABB;
      pcl::PointXYZ max_point_AABB;
      float major_value, middle_value, minor_value;

      /**********************************外接 长方体 算法一 ***********************************/
      /*            feature_extractor.getMomentOfInertia(moment_of_inertia);
            feature_extractor.getEccentricity(eccentricity);
            feature_extractor.getEigenValues(major_value, middle_value, minor_value);
            feature_extractor.getAABB(min_point_AABB, max_point_AABB);

            if(DEBUG_MODE)
            {
                viewer->setBackgroundColor(0, 0, 1);
                viewer->addCoordinateSystem(1.0);
                //viewer->initCameraParameters();

                viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z,
                                max_point_AABB.z, 1.0, 1.0, 0.0, "AABB"+std::to_string(number));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB"+std::to_string(number));
            }
            Obstacle_state_Information obstacleStateInformation; //障碍物最小点位信息

            obstacleStateInformation.coordinate_point[0] = Eigen::Vector3d (min_point_AABB.x,min_point_AABB.y,min_point_AABB.z);
            obstacleStateInformation.coordinate_point[1] = Eigen::Vector3d (min_point_AABB.x,max_point_AABB.y,min_point_AABB.z);
            obstacleStateInformation.coordinate_point[2] = Eigen::Vector3d (max_point_AABB.x,min_point_AABB.y,min_point_AABB.z);
            obstacleStateInformation.coordinate_point[4] = Eigen::Vector3d (max_point_AABB.x,max_point_AABB.y,min_point_AABB.z);
            obstacleStateInformation.coordinate_point[4] = Eigen::Vector3d (min_point_AABB.x,min_point_AABB.y,max_point_AABB.z);
            obstacleStateInformation.coordinate_point[5] = Eigen::Vector3d (min_point_AABB.x,max_point_AABB.y,max_point_AABB.z);
            obstacleStateInformation.coordinate_point[6] = Eigen::Vector3d (max_point_AABB.x,min_point_AABB.y,max_point_AABB.z);
            obstacleStateInformation.coordinate_point[7] = Eigen::Vector3d (max_point_AABB.x,max_point_AABB.y,max_point_AABB.z);*/
      /**********************************外接 长方体 算法二 ***********************************/

      pcl::PointXYZ min_point_OBB;
      pcl::PointXYZ max_point_OBB;
      pcl::PointXYZ position_OBB;

      Eigen::Matrix3f rotational_matrix_OBB;
      Eigen::Vector3f major_vector, middle_vector, minor_vector;
      Eigen::Vector3f mass_center;

      feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
      feature_extractor.getMassCenter(mass_center);
      feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

      Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat(rotational_matrix_OBB);

      Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
      Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
      Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
      Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
      Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
      Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
      Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
      Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

      p1 = rotational_matrix_OBB * p1 + position;     // OBB 算法的最小点向量和最大点向量，
      p2 = rotational_matrix_OBB * p2 + position;     // 是相对于包围长方体的中心向量而言。
      p3 = rotational_matrix_OBB * p3 + position;     // 旋转偏移四位素是从包围体的中心点转到原点
      p4 = rotational_matrix_OBB * p4 + position;
      p5 = rotational_matrix_OBB * p5 + position;
      p6 = rotational_matrix_OBB * p6 + position;
      p7 = rotational_matrix_OBB * p7 + position;
      p8 = rotational_matrix_OBB * p8 + position;

      Obstacle_state_Information obstacleStateInformation; //障碍物最小点位信息
      obstacleStateInformation.coordinate_point[0] = p1.cast<double>();   //强制转换为Vector3d
      obstacleStateInformation.coordinate_point[1] = p2.cast<double>();
      obstacleStateInformation.coordinate_point[2] = p3.cast<double>();
      obstacleStateInformation.coordinate_point[3] = p4.cast<double>();
      obstacleStateInformation.coordinate_point[4] = p5.cast<double>();
      obstacleStateInformation.coordinate_point[5] = p6.cast<double>();
      obstacleStateInformation.coordinate_point[6] = p7.cast<double>();
      obstacleStateInformation.coordinate_point[7] = p8.cast<double>();

      //            std::cerr<<"obstacle 3D BBox: p0:"<<p1<<"\np3:"<<p4<<"\np7:"<<p8<<"\np4:"<<p5<<std::endl;
      //            std::cerr<<"obstacle 3D BBox: coordinate_point0:"<<obstacleStateInformation.coordinate_point[0]<<"\ncoordinate_point3:"<<obstacleStateInformation.coordinate_point[3]<<"\ncoordinate_point7:"<<obstacleStateInformation.coordinate_point[7]<<"\ncoordinate_point4:"<<obstacleStateInformation.coordinate_point[4]<<std::endl;

      obstacles_in_livox_coordinate.push_back(obstacleStateInformation);

      if (1)
      {
        pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
        pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
        pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
        pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
        pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
        pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
        pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
        pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));


        //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));


        viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge"+std::to_string(number));
        viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge"+std::to_string(number));
        viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge"+std::to_string(number));
        viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge"+std::to_string(number));
        viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge"+std::to_string(number));
        viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge"+std::to_string(number));
        viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge"+std::to_string(number));
        viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge"+std::to_string(number));
        viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge"+std::to_string(number));
        viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge"+std::to_string(number));
        viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge"+std::to_string(number));
        viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge"+std::to_string(number));

        //                viewer->addPointCloud<pcl::PointXYZ>(_cloudPE, "sample cloud");
        //                while (!viewer->wasStopped())
        //                {
        //                    viewer->spinOnce();
        //                }
      }
    }

    viewer->addPointCloud<pcl::PointXYZ>(_cloudPE, "sample cloud");
    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
  }
  else{ return -1;}
  return res;
}

void ObstacleDetection::GetObjectsSize(double hook_height, double obj_width, double obj_length,double obj_height,double sling_height,double lift_Height,int lift_mode)
{
  m_hook_height = hook_height;
  m_object_height = obj_height;
  m_object_len = obj_length;
  m_object_width = obj_width;
  m_rope_height = sling_height;
  m_goods_height =lift_Height;
  m_nMode = lift_mode;
}

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include <iostream>
#include <QMutex>

#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "objects_msg/point_attr.h"
#include "objects_msg/object_attr.h"
#include "objects_msg/objects_list.h"
#include "std_msgs/String.h"

typedef struct _strObjResult
{
    float c_x;
    float c_y;
    float c_z;
    float qx;
    float qy;
    float qz;
    float qw;
    float width_x;
    float height_y;
    float depth_z;
}ObjResult;

class subscriberThread : public QObject
{
    Q_OBJECT
public:
  subscriberThread();
  ~subscriberThread(){}

  void Sub(const sensor_msgs::PointCloud2::ConstPtr& laser_data);
  void CalibResultSub(const objects_msg::objects_list::ConstPtr& _data);
  void getObjectsResult(std::vector<ObjResult>& objects);
  std::string receive;
  void ResetCalibState();

private:
  QMutex mutex;
  QMutex mutex_calib;

public Q_SLOTS:
  void thread_Spin();

Q_SIGNALS:
  void Signal();

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_;
  std::vector<ObjResult> objects_res_;
  bool bCalib3D_;

};


#endif // SUBSCRIBER_H

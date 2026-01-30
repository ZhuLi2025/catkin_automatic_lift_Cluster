#include "subscriberThread.hpp"

subscriberThread::subscriberThread()
{
    receive = "";
    bCalib3D_ = false;
    objects_res_.clear();
    laser_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void subscriberThread::Sub(const sensor_msgs::PointCloud2::ConstPtr& laser_data)
{
    mutex.lock();//上锁
    //ROS_INFO("laser data size: [%d]", laser_data->data.size());
    //点云转pcl格式
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*laser_data, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*laser_cloud_);
    //receive = msg->data.c_str();
    Q_EMIT Signal();
    mutex.unlock();//解锁
}

void subscriberThread::ResetCalibState()
{
    mutex_calib.lock();//上锁
    bCalib3D_ = false;
    mutex_calib.unlock();//解锁
}

void subscriberThread::CalibResultSub(const objects_msg::objects_list::ConstPtr& _data)
{
    mutex_calib.lock();//上锁
    objects_res_.clear();
    ObjResult temp;
    for (auto obj : _data->objects)
    {
        temp.c_x = obj.cx;
        temp.c_y = obj.cy;
        temp.c_z = obj.cz;
        temp.qx = obj.qx;
        temp.qy = obj.qy;
        temp.qz = obj.qz;
        temp.width_x = obj.width_x;
        temp.height_y = obj.height_y;
        temp.depth_z = obj.depth_z;

        objects_res_.push_back(temp);
    }

    std::cerr<<"objects_res_.size():"<<objects_res_.size()<<std::endl;
    if((objects_res_.size()!=0) && !bCalib3D_)
    {
        bCalib3D_ = true;
    }
    Q_EMIT Signal();
    mutex_calib.unlock();//解锁
}

void subscriberThread::getObjectsResult(std::vector<ObjResult>& objects)
{
    mutex_calib.lock();
    objects = objects_res_;
    mutex_calib.unlock();
}

void subscriberThread::thread_Spin()
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/livox/lidar",1000, &subscriberThread::Sub, this);
    ros::Subscriber objects_sub_ = n.subscribe<objects_msg::objects_list>("/objects/result", 100, &subscriberThread::CalibResultSub,this);//boost::bind(&MainWindow::ObjectsResultCallback, this, 1));
    ros::spin();
}


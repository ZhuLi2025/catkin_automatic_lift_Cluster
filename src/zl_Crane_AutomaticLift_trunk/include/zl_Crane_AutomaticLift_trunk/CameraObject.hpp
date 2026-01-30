#ifndef CAMERAOBJECT_H
#define CAMERAOBJECT_H

#include <QReadWriteLock>
#include<QString>
#include <opencv2/opencv.hpp>    //添加Opencv相关头文件
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/tracking.hpp>
#include <Eigen/Eigen>
#include "HCNetSDK.h"
#include "LinuxPlayM4.h"
#include "PlayM4.h"
#include "zl_common_define.hpp"

using namespace  cv;
using namespace std;

#define HPR_ERROR       1
#define HPR_OK              0

namespace zl_common {
namespace app {

class CameraObject
{
//  Q_OBJECT
public:
  CameraObject();

private:
      //定位相机登录、显示视频
      LONG m_lUserID;          //user ID
      LONG m_llRealHandle;       //视频图像加载
      static LONG m_nPort;
      int        m_focusIndex;
      QString m_strZoom;
      NET_DVR_PTZPOS m_ptzSetPos;

      bool m_bLoginCamera{false};


public:
      Mat m_dstImage;


public:
      int LoginCameraVideo();
      void OpenCamera();
      void CloseCamera();

      void setZoomValue(QString strZoomValue);
//      实时流回调
     static void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser);
//      解码回调 视频为YUV数据(YV12)，音频为PCM数据
      static void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2);

      list<Mat> getFrameList();

public:
      void get_transform_from_rotateCamera_to_initCamera(Eigen::Isometry2f& out_pose, float rotate_angle);
      void get_transform_from_slipeCamera_to_camera(Eigen::Isometry2f &out_pose);
      void get_transform_from_armHead_to_camera(Eigen::Isometry3f& out_pose, zl_common::app::HParameter *hcfg_);
      void get_transform_from_camera_to_armHead(Eigen::Isometry3f& out_pose, zl_common::app::HParameter *hcfg_, zl_common::app::CRANE_PARAM *crane_parameter_);
};
}
}
#endif // CAMERAOBJECT_H

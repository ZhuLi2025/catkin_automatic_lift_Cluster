#include "CameraObject.hpp"
#include<QMessageBox>
#include<iostream>


list<Mat> gframeList_;
QReadWriteLock g_ImageListMutex;
zl_common::app::CameraObject *pCameraObject = new zl_common::app::CameraObject();

namespace zl_common {
namespace app {
//*CameraObject::pCameraObject = new CameraObject();


LONG CameraObject::m_nPort = -1;
CameraObject::CameraObject()
{
  m_lUserID = 0;          //user ID
  m_llRealHandle = -1;       //视频图像加载
  m_nPort = -1;
  m_focusIndex = 0;
  m_strZoom = "10";
}

void CameraObject::setZoomValue(QString strZoomValue)
{
  QByteArray Q_strZoomValue = strZoomValue.toLocal8Bit();
  m_ptzSetPos.wAction = 1;
  m_ptzSetPos.wZoomPos = strtoll(Q_strZoomValue.data(), NULL, 16);		//一倍焦距
  NET_DVR_SetDVRConfig(m_lUserID, NET_DVR_SET_PTZPOS, 0, &m_ptzSetPos, sizeof(NET_DVR_PTZPOS));
}

// ////////////////////////////////////////////////////////////////////////
//解码回调 视频为YUV数据(YV12)，音频为PCM数据
void  CameraObject::DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2)
{
    long lFrameType = pFrameInfo->nType;
//    CameraObject *pCameraObject = new CameraObject();
//    std::cerr << "pBuf length is : " << strlen(pBuf) << std::endl;
    if (lFrameType == T_YV12)
    {
        g_ImageListMutex.lockForWrite();
        pCameraObject->m_dstImage.create(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
        Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (uchar *)pBuf);
        cvtColor(src, pCameraObject->m_dstImage, CV_YUV2BGRA_YV12);
        // brg to rgb
        cvtColor(pCameraObject->m_dstImage, pCameraObject->m_dstImage, CV_BGR2RGB );
        /////////////////////////////////
        // 选取当前视频某一帧ROI区域进行获取数据,待更改为雷达探测目标区域ROI图像
        //imwrite("/home/zoomlion/dstimg.jpg",dstImg);
//        pCameraObject->gframeList_.push_back(pCameraObject->m_dstImage);
        gframeList_.push_back(pCameraObject->m_dstImage);
        g_ImageListMutex.unlock();
    }
}

list<Mat> CameraObject::getFrameList()
{
  return gframeList_;
}
// /////////////////////////////////////////////////////////////////////////////////
//实时流回调
void CALLBACK CameraObject::g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser)
{
//   CameraObject *pCameraObject = new CameraObject();
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD:         //系统头
        if (!PlayM4_GetPort(&pCameraObject->m_nPort))  //获取播放库未使用的通道号
        {
            break;
        }
        if (dwBufSize > 0) {
            if (!PlayM4_SetStreamOpenMode(pCameraObject->m_nPort, STREAME_REALTIME)) {
                PlayM4_GetLastError(pCameraObject->m_nPort);
                break;
            }
            if (!PlayM4_OpenStream(pCameraObject->m_nPort, pBuffer, dwBufSize, 2408 * 2408)) {
                PlayM4_GetLastError(pCameraObject->m_nPort);
                break;
            }
            //设置解码回调函数 解码且显示
            if (!PlayM4_SetDecCallBackEx(pCameraObject->m_nPort, pCameraObject->DecCBFun, NULL, 0))
            {
                PlayM4_GetLastError(pCameraObject->m_nPort);
                break;
            }

            //打开视频解码
            if (!PlayM4_Play(pCameraObject->m_nPort, 0))
            {
                PlayM4_GetLastError(pCameraObject->m_nPort);
                break;
            }
        }
        break;
        //usleep(500);
    case NET_DVR_STREAMDATA:  //码流数据
        if (dwBufSize > 0 && pCameraObject->m_nPort != -1)
        {
            BOOL inData = PlayM4_InputData(pCameraObject->m_nPort, pBuffer, dwBufSize);
            while (!inData)
            {
                //                sleep(100);
                inData = PlayM4_InputData(pCameraObject->m_nPort, pBuffer, dwBufSize);
                //std::cerr << "PlayM4_InputData failed \n" << std::endl;
            }
        }
        break;
    }
}
void CameraObject::OpenCamera()
{
//      CameraObject *pCameraObject = new CameraObject();
      if (!m_bLoginCamera)
      {
          //****************************** 云台相机启动自检及初始化 ************************************//
          m_bLoginCamera = LoginCameraVideo();
          if (HPR_ERROR == m_bLoginCamera)
          {
              QMessageBox::information(NULL, "错误！", (" LoginCameraVideo is failure"));
              return;
          }
      }

      NET_DVR_CLIENTINFO ClientInfo = {0};
      ClientInfo.lChannel = 1; //Channel number 设备通道号
      ClientInfo.hPlayWnd = 0;  //窗口为空，设备SDK不解码只取流
      ClientInfo.lLinkMode = 0;    //Main Stream  码流 0   主码流
      ClientInfo.sMultiCastIP = NULL;

      //预览取流
      m_llRealHandle = NET_DVR_RealPlay_V30(m_lUserID, &ClientInfo, pCameraObject->g_RealDataCallBack_V30, NULL, TRUE);
      if (m_llRealHandle < 0)
      {
          QMessageBox::information(NULL, "错误！", "NET_DVR_RealPlay_V30 is failure.");
          NET_DVR_Cleanup();
          return;
      }
}

void CameraObject::CloseCamera()
{
  //关闭定位相机监控显示功能
  if (!NET_DVR_StopRealPlay(m_llRealHandle))			//停止预览
  {
      QMessageBox::information(NULL, "错误！", "NET_DVR_StopRealPlay is failed!");
  }
  m_llRealHandle = -1;

  if (m_nPort > -1)		//停止解码
  {
      if (!PlayM4_Stop(m_nPort))
      {
          PlayM4_GetLastError(m_nPort);
      }
      if (!PlayM4_CloseStream(m_nPort))
      {
          PlayM4_GetLastError(m_nPort);
      }
      PlayM4_FreePort(m_nPort);
      m_nPort = -1;
  }
}

//////////////////////////////////////////////////////////////////
/// \brief MainWindow::loadCameraVideo
/// \return 0-1
///相机登录及焦距初始化设置
/// ///////////////////////////////////////////////////////////////
int CameraObject::LoginCameraVideo()
{
NET_DVR_Init();
NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
struLoginInfo.bUseAsynLogin = false;

struLoginInfo.wPort = 8000;
   mempcpy(struLoginInfo.sDeviceAddress, "192.168.1.64",NET_DVR_DEV_ADDRESS_MAX_LEN);//add
    memcpy(struLoginInfo.sUserName, "admin", NAME_LEN);
   memcpy(struLoginInfo.sPassword, "Password", NAME_LEN);



m_lUserID = NET_DVR_Login_V30(struLoginInfo.sDeviceAddress, 8000, struLoginInfo.sUserName, struLoginInfo.sPassword, &struDeviceInfo);
if(m_lUserID < 0)
{
  int errorCode =  NET_DVR_GetLastError();
  const QString string_temp = "NET_DVR_Login_V30 is failure. The ErrorCode is " + QString::number(errorCode,10);
  QMessageBox::information(NULL, ("错误！"), string_temp);
  NET_DVR_Cleanup();
  return HPR_ERROR;
}

//相机焦距调节
// setZoomValue(m_strZoom);
return HPR_OK;
}

void CameraObject::get_transform_from_rotateCamera_to_initCamera(Eigen::Isometry2f& out_pose, float rotate_angle)
{
    //二维坐标转换 旋转角度逆时针方位为正方向
    //逆时针为正
    std::cerr <<"camera_rotate_angle_ ="<<rotate_angle<<std::endl;
    float theta = -rotate_angle * M_PI / 180;
    out_pose.setIdentity();
    out_pose.linear() << cos(theta),sin(theta),-sin(theta),cos(theta);
}


void CameraObject::get_transform_from_armHead_to_camera(Eigen::Isometry3f& out_pose, zl_common::app::HParameter *hcfg_)
{
    //云台固定安装面中心点到相机中心点坐标转换
    Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
//    tf.pretranslate(Eigen::Vector3f ( -0.052,0.0062,0.477 ));//平移向量
     tf.pretranslate(Eigen::Vector3f ( -hcfg_->camera_transform_ptz_x, -hcfg_->camera_transform_ptz_y, -hcfg_->camera_transform_ptz_z));//平移向量
    out_pose = tf;
}


void CameraObject::get_transform_from_camera_to_armHead(Eigen::Isometry3f& out_pose, zl_common::app::HParameter *hcfg_, zl_common::app::CRANE_PARAM *crane_parameter_)
{
    //云台固定安装面中心点到臂尖坐标系转换
    //yt
    Eigen::Isometry3f tf_1 = Eigen::Isometry3f::Identity();
    tf_1.translation() = Eigen::Vector3f (hcfg_->ptz_tansform_arm_header_x, hcfg_->ptz_tansform_arm_header_y, hcfg_->ptz_tansform_arm_header_z);
    tf_1 = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI/2 + crane_parameter_->crane_luff_Angle_ * M_PI / 180,
                                                                     Eigen::Vector3f::UnitY())
                                                   *Eigen::AngleAxisf(-M_PI/2,
                                                                      Eigen::Vector3f::UnitZ()));
    //相机中心到云台安装面中心点
    Eigen::Isometry3f tf_2 = Eigen::Isometry3f::Identity();
//    tf_2.pretranslate(Eigen::Vector3f ( 0.052,-0.0062,-0.477 ));//平移向量
    tf_2.pretranslate(Eigen::Vector3f ( hcfg_->camera_transform_ptz_x, hcfg_->camera_transform_ptz_y, hcfg_->camera_transform_ptz_z ));//平移向量
    out_pose = tf_2 * tf_1;
}
}
}

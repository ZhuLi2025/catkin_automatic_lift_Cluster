#ifndef ZL_COMMON_DEFINE_HPP
#define ZL_COMMON_DEFINE_HPP
#include <QString>
#include <vector>
#include <zl_manipulation/AutoCraneArmManipulation.h>

#define PTS_LENG      20

namespace zl_common{
namespace app {

typedef struct{
  float camera_xdire_angle = 0;
  float camera_ydire_angle = 0;
  float camera_to_armcenter_x = 0.0;
  float camera_to_armcenter_y = 0.0;

  float ptz_pitch_initial_angle = 0;
  float ptz_rotate_initial_angle = 0;
  QString ptz_str_pitch_initial_angle = "";
  QString ptz_str_rotate_initial_angle = "";

  float camera_transform_ptz_x = 0.0;
  float camera_transform_ptz_y = 0.0;
  float camera_transform_ptz_z = 0.0;

  float ptz_tansform_arm_header_x = 0.0;
  float ptz_tansform_arm_header_y = 0.0;
  float ptz_tansform_arm_header_z = 0.0;

  float left_rotate_and_luff_up_rotation_amend = 0.0;
  float left_rotate_and_luff_up_luffing_amend = 0.0;

  float left_rotate_and_luff_down_rotation_amend = 0.0;
  float left_rotate_and_luff_down_luffing_amend = 0.0;

  float right_rotate_and_luff_up_rotation_amend = 0.0;
  float right_rotate_and_luff_up_luffing_amend = 0.0;

  float right_rotate_and_luff_down_rotation_amend = 0.0;
  float right_rotate_and_luff_down_luffing_amend = 0.0;
}HParameter;

typedef struct
{
  double crane_rotation_Angle_;
  double crane_luff_length;
  double laser_camera_to_ground_height_;
  double crane_luff_Angle_;
  double crane_arm_to_ground_height_;
  double crane_mainarm_length_;
  double crane_hook_to_ground_height_;
  double laser_hook_to_ground_height{0};

  double crane_rope_ratio_;
  double crane_legState_;          //支腿方式-0全伸 1半伸
  double crane_Bob_Weight_;          //配重0-56t;1-48t;2-36t;3-24t;4-12t;5-0t
  QString crane_arm_stretch_State_;    //臂架状态
  double crane_arm_status_;
  std::vector<int> crane_vec_stretch_way_;

}CRANE_PARAM;

class MyPoint{
public:
  double m_pointX;
  double m_pointY;
public:
  MyPoint(){}
  MyPoint(double x, double y){m_pointX = x; m_pointY = y;}
};

class MyLine:public MyPoint
{
public:
  double a;
  double b;
  double c;
public:
  MyLine GetLine(MyPoint ptSource, MyPoint ptDestination);
  MyPoint GetCrossPoint(MyLine l1, MyLine l2);
};

//yt add for jiqunxietong
struct structPointXYZ
{
    double point_x;
    double point_y;
    double point_z;
};

struct pointPostrue
{
    double rotate;
    double amplitude;
    double roller;
};

struct avoidcommandtask
{
    int id;
    int type;
    structPointXYZ targetpositon;
    pointPostrue targetpostrue1;
    pointPostrue targetpostrue2;
    bool status1;
    bool status2;
    bool status;
    bool ifget;
};

struct buildinginformation//建筑构件信息（任务类型为吊装时）
{
    int id;	//整型，构建id
    int type;	//枚举整型，1 墙板，2 楼板，3 窗户，4 基槽，5 3D打印构件。
    int weight;	//构件重量
    structPointXYZ buildinglocation;	//构件位置
    structPointXYZ buildingtargetlocation;//装载构件目标位置
    double orientation;	//浮点型，构件朝向
};

struct dispatchtask
{
    int id;
    int type;//type:,	//枚举整型，任务类型1 行驶,2 吊装，3 回到安全位置
    buildinginformation taskinfor;
    structPointXYZ safelocation;
    structPointXYZ auto_crane_location;
    bool statusa;//auto crane location status
    bool statusb;//go building location status
    bool autodrvingb;

    bool recoverautodving;
    bool statust;//go target location status
    bool autodrvingt;

    bool statuss;//go safe location status
    bool ifget;
    bool completeall;

    int zhuyuxing;//0 chushi  1 -start 2 =end
    bool hook_down_up;
};
struct commandtask
{
    int id;
    int taskinfor;
    bool status;
    bool ifget;
};

//yt add for jiqunxietong

namespace zl
{
enum TransferState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};

enum ReactionThreshold
{
  REPLAN = 10, E_STOP = 8, SLOW_DOWN = 15, NORMAL = 25
  //    REPLAN = 2, E_STOP = 2, SLOW_DOWN = 4, NORMAL = 8
};

enum PickPlaceMode
{
  ONCE = 0, REPEAT = 1
};

struct RepeatPPTData
{
  std::queue<ompl::app::position_t> targets;
  ompl::app::position_t target_a;
  ompl::app::position_t target_b;

  RepeatPPTData()
  {
    target_a.x = 0;
    target_a.y = 0;
    target_a.z = 0;
    target_b.x = 0;
    target_b.y = 0;
    target_b.z = 0;
  }
};

struct PickPlaceTask
{
  PickPlaceMode mode;
  RepeatPPTData repeat_data;

  PickPlaceTask()
  {
    mode = zl::PickPlaceMode::ONCE;
  }
};
}//end namespace zl
}//end namespace app
}//end namespace zl_common
#endif // ZL_COMMON_DEFINE_HPP

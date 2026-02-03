#!/bin/bash
source devel/setup.bash

# 设置海康库路径
DIR="$( pwd )"
export LD_LIBRARY_PATH=$DIR/src/zl_Crane_AutomaticLift_trunk/3rd_partys/HCNetSDK/lib:$DIR/src/zl_Crane_AutomaticLift_trunk/3rd_partys/HCNetSDK/lib/HCNetSDKCom:$LD_LIBRARY_PATH

roslaunch zl_Crane_AutomaticLift_trunk zl_crane_automaticlift_trunk.launch

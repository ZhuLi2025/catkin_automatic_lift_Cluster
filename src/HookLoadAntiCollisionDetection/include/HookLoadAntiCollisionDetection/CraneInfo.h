#ifndef CRANE_INFO_H
#define CRANE_INFO_H

typedef struct CraneInfo {
    float mainArmLength;
    float subArmAngle;
    float mainArmAngle;
    float subArmLength;
    float mainArmToGroundHeight;
    float subArmToGroundHeight;
    float rotationAngle;
    float mainHookToGroundHeight;
    float subHookToGroundHeight;
    float fixedArmAngle;
    float fixedArmLength;
    float fixedArmToGroundHeight;
    float ropeRaio;
    int   legState;
    int   bobWeight;
    int   armStatus; // 0 主臂；1 塔臂；2 固定副臂
} _CraneInfo;

#endif
//
// Created by 李唐 on 2024/3/2.
//

#ifndef MOTIONKIT_MOTION_H
#define MOTIONKIT_MOTION_H
#include "Drv/NodeMotor.h"

#define LMOTOR_ID (0x01)
#define RMOTOR_ID (0x02)
#define DIRECTION (1)

typedef struct {
    double x;
    double y;
    double z;
}LinearType;
//Based on ROS Vector3/linear

typedef struct {
    double x;
    double y;
    double z;
}AngularType;
//Based on ROS Vector3/angular

typedef struct{
    LinearType linear;
    AngularType angular;
}TwistyType;
//Based on ROS Geometry/Twist

typedef struct{
    NodeMotorMode MotionMode;
    NodeMotorType LMotor;
    NodeMotorType RMotor;


    double LinearVelocity;
    double AngularVelocity;
}MotionType;

//extern MotionType PancakeMotion;
void MotionInit(MotionType *Motion,NodeMotorMode MotionMode);
void MotionEnable(MotionType *Motion);
void MotionUpdateVelocity(MotionType *Motion);
void MotionSetVolocity(MotionType *Motion,double Velocity);
void MotionSetRotate(MotionType *Motion, double Rotate);

#endif //MOTIONKIT_MOTION_H

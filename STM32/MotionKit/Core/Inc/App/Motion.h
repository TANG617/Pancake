//
// Created by 李唐 on 2024/3/2.
//

#ifndef MOTIONKIT_MOTION_H
#define MOTIONKIT_MOTION_H
#include "Drv/NodeMotor.h"

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
    NodeMotorType LMotor;
    NodeMotorType RMotor;

}MotionType;


#endif //MOTIONKIT_MOTION_H

//
// Created by 李唐 on 2024/3/2.
//

#ifndef MOTIONKIT_MOTION_H
#define MOTIONKIT_MOTION_H
#include "Drv/NodeMotor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "PancakeConfig.h"





typedef struct{
    NodeMotorMode MotionMode;
    NodeMotorType LMotor;
    NodeMotorType RMotor;

    uint8_t Enable;
    double LinearVelocity;
    double AngularVelocity;
}MotionType;


void MotionInit(MotionType *Motion,NodeMotorMode MotionMode);
void MotionEnable(MotionType *Motion);
void MotionDisable(MotionType *Motion);
void MotionUpdateVelocity(MotionType *Motion);
void MotionSetLinearVelocity(MotionType *Motion,double Velocity);
void MotionSetAngularVelocity(MotionType *Motion, double Rotate);

#endif //MOTIONKIT_MOTION_H

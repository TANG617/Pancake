//
// Created by 李唐 on 2024/3/2.
//

#include "App/Motion.h"
#define DELAY 1
void MotionInit(MotionType *Motion,NodeMotorMode MotionMode){
    Motion->MotionMode = MotionMode;

    Motion->LMotor.CanHandler = &hcan;
    Motion->LMotor.id = LMOTOR_ID;
    Motion->LMotor.Mode = MotionMode;
    Motion->LMotor.Velocity = 0;
    Motion->LMotor.Direction = DIRECTION;

    Motion->RMotor.CanHandler = &hcan;
    Motion->RMotor.id = RMOTOR_ID;
    Motion->RMotor.Mode = MotionMode;
    Motion->RMotor.Velocity = 0;
    Motion->LMotor.Direction = (-1)*(DIRECTION);
}

void MotionEnable(MotionType *Motion){
    NodeMotorEnable(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorEnable(&Motion->RMotor);
}

void MotionDisable(MotionType *Motion){
    NodeMotorDisable(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorDisable(&Motion->RMotor);
}
void MotionUpdateVelocity(MotionType *Motion){
    NodeMotorVelocityControl(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorVelocityControl(&Motion->RMotor);
}

void MotionSetVolocity(MotionType *Motion,double Velocity){
    Motion->LMotor.Velocity = Velocity * Motion->LMotor.Direction;
    osDelay(DELAY);
    Motion->RMotor.Velocity = Velocity ;
}

void MotionSetRotate(MotionType *Motion, double Rotate){
    Motion->LMotor.Velocity += Rotate * Motion->LMotor.Direction;
    osDelay(DELAY);
    Motion->RMotor.Velocity -= Rotate * Motion->RMotor.Direction;
}
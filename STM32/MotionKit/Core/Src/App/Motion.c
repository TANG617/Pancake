//
// Created by 李唐 on 2024/3/2.
//

#include "App/Motion.h"
#define DELAY 1
void MotionInit(MotionType *Motion,NodeMotorMode MotionMode){
    Motion->MotionMode = MotionMode;
    Motion->Enable = 0;

    Motion->LMotor.CanHandler = &hcan;
    Motion->LMotor.id = LMOTOR_ID;
    Motion->LMotor.Mode = MotionMode;
    Motion->LMotor.Velocity = 0;
    Motion->LMotor.Direction = DIRECTION;

    Motion->RMotor.CanHandler = &hcan;
    Motion->RMotor.id = RMOTOR_ID;
    Motion->RMotor.Mode = MotionMode;
    Motion->RMotor.Velocity = 0;
    Motion->RMotor.Direction = (-1)*(DIRECTION);
}

void MotionEnable(MotionType *Motion){
    Motion->Enable = 1;
    NodeMotorEnable(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorEnable(&Motion->RMotor);
}

void MotionDisable(MotionType *Motion){
    Motion->Enable = 0;
    NodeMotorDisable(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorDisable(&Motion->RMotor);
}
void MotionUpdateVelocity(MotionType *Motion){
    NodeMotorVelocityControl(&Motion->LMotor);
    osDelay(DELAY);
    NodeMotorVelocityControl(&Motion->RMotor);
}

void MotionSetLinearVelocity(MotionType *Motion,double Velocity){
    Motion->LinearVelocity = Velocity;
    Motion->LMotor.Velocity = Velocity * Motion->LMotor.Direction;
    osDelay(DELAY);
    Motion->RMotor.Velocity = Velocity * Motion->RMotor.Direction;
}

//Positive Rotate means turns right
void MotionSetAngularVelocity(MotionType *Motion, double Rotate){
    Motion->LMotor.Velocity += Rotate * Motion->LMotor.Direction;
    osDelay(DELAY);
    Motion->RMotor.Velocity -= Rotate * Motion->RMotor.Direction;
}
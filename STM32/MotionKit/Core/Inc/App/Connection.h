//
// Created by 李唐 on 2024/3/2.
//

#ifndef MOTIONKIT_CONNECTION_H
#define MOTIONKIT_CONNECTION_H
#include "stm32f1xx_hal.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//Serial.printf("%04d_%04d_%04d_%04d_%04d_%04d#",ps5.LStickX(),ps5.LStickY(),ps5.RStickX(),ps5.RStickY(),ps5.L2Value(),ps5.R2Value());

#define MAX_INT32  (128*256*256*256 -1 )
#define LINEAR_VEL_MAX  20
#define ANGULAR_VEL_MAX  20

//typedef struct {
//    double x;
//    double y;
//    double z;
//}LinearType;
////Based on ROS Vector3/linear
//
//typedef struct {
//    double x;
//    double y;
//    double z;
//}AngularType;
////Based on ROS Vector3/angular
//
//typedef struct{
//    LinearType linear;
//    AngularType angular;
//}TwistyType;
////Based on ROS Geometry/Twist


typedef enum{
    LinearVelocityMode = 0x01,
    LinearPositionMode = 0x02,
    AngularVelocityMode = 0x03,
    AngularPositionMode = 0x04,
}ControlFrameMode;

typedef struct{
    ControlFrameMode Mode;
    float LinearVelocity, AngularVelocity;
    float LVelocity, RVelocity;
    float LinearPosition,AngularPosition;
}ControlFrameType;


ControlFrameType PackageDecode(uint8_t* packageFrame);
ControlFrameType PackageFetch();

#endif //MOTIONKIT_CONNECTION_H
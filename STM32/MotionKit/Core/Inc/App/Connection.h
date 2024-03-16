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

#include "PancakeConfig.h"
#include "Motion.h"

#define MAX_INT16  (128*256 -1 )

typedef enum{
    Enable = 0x00,
    VelocityMode = 0x01,
    PositionMode = 0x02,
}ControlFrameMode;

typedef struct{
    ControlFrameMode Mode;
    float LinearVelocity, AngularVelocity;
    float LVelocity, RVelocity;
    float LinearPosition,AngularPosition;
}ControlFrameType;

extern uint8_t bufControlPack[12];
extern uint8_t ControlPack[5];

ControlFrameType DecodeControlFrame();
#endif //MOTIONKIT_CONNECTION_H

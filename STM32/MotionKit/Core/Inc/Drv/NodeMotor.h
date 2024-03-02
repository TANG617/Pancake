//
// Created by 李唐 on 2023/11/29.
//

#ifndef PIONEER_NODEMOTOR_H
#define PIONEER_NODEMOTOR_H
#include "can.h"
#include "stm32f1xx_hal.h"

#define P_MIN     (-12.5)
#define P_MAX      12.5
#define V_MIN     (-45)
#define V_MAX      45
#define KP_MIN     0
#define KP_MAX     500
#define KD_MIN     0
#define KD_MAX     5
#define T_MIN     (-18)
#define T_MAX      18

typedef enum {
    MIT              = 0x000,
    PositionVelocity = 0x100,
    Velocity         = 0x200,
} NodeMotorMode;

typedef struct{
    CAN_HandleTypeDef *CanHandler;
    CAN_TxHeaderTypeDef TxHeader;
    uint16_t id;
    float Position, Velocity, Kp, Kd, Torque;
    int Direction;
    NodeMotorMode Mode;
}NodeMotorType;

HAL_StatusTypeDef NodeMotorClearError(NodeMotorType *Motor);
HAL_StatusTypeDef NodeMotorEnable(NodeMotorType *Motor);
HAL_StatusTypeDef NodeMotorDisable(NodeMotorType *Motor);
HAL_StatusTypeDef NodeMotorSaveZero(NodeMotorType *Motor);

HAL_StatusTypeDef NodeMotorMITControl(NodeMotorType *Motor);
HAL_StatusTypeDef NodeMotorVelocityControl(NodeMotorType *Motor);
HAL_StatusTypeDef NodeMotorPositionVelocityControl(NodeMotorType *Motor);

#endif //PIONEER_NODEMOTOR_H

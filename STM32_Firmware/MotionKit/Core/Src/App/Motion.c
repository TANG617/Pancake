//
// Created by 李唐 on 2024/3/2.
//

#include "App/Motion.h"
extern MotionType PancakeMotion;
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
    osDelay(MOTION_DELAY);
    NodeMotorEnable(&Motion->RMotor);
}
void MotionDisable(MotionType *Motion){
    Motion->Enable = 0;
    NodeMotorDisable(&Motion->LMotor);
    osDelay(MOTION_DELAY);
    NodeMotorDisable(&Motion->RMotor);
}
void MotionUpdateVelocity(MotionType *Motion){
    NodeMotorVelocityControl(&Motion->LMotor);
    osDelay(MOTION_DELAY);
    NodeMotorVelocityControl(&Motion->RMotor);
}
void MotionSetLinearVelocity(MotionType *Motion,double Velocity){
    Motion->LinearVelocity = Velocity;
    Motion->LMotor.Velocity = Velocity * Motion->LMotor.Direction;
    osDelay(MOTION_DELAY);
    Motion->RMotor.Velocity = Velocity * Motion->RMotor.Direction;
}
//Positive Rotate means turns right
void MotionSetAngularVelocity(MotionType *Motion, double Rotate){
    Motion->LMotor.Velocity -= Rotate * Motion->LMotor.Direction;
    osDelay(MOTION_DELAY);
    Motion->RMotor.Velocity += Rotate * Motion->RMotor.Direction;
}

void StartMotionTask(void *argument)
{

    MotionInit(&PancakeMotion,Velocity);
    for(;;)
    {
        if(!HAL_GPIO_ReadPin(BTN_PUSH_GPIO_Port,BTN_PUSH_Pin))
        {
            MotionInit(&PancakeMotion,Velocity);
            MotionEnable(&PancakeMotion);
            osDelay(100);
            if(!HAL_GPIO_ReadPin(BTN_PUSH_GPIO_Port,BTN_PUSH_Pin))
            {

                osDelay(1000);
                if(!HAL_GPIO_ReadPin(BTN_PUSH_GPIO_Port,BTN_PUSH_Pin))
                {

                    MotionDisable(&PancakeMotion);
                    osDelay(1000);
                }
            }
        }
        
        
        MotionUpdateVelocity(&PancakeMotion);
        osDelay(30);
    }
}
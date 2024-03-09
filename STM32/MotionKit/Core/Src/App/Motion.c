//
// Created by 李唐 on 2024/3/2.
//

#include "App/Motion.h"
extern MotionType PancakeMotion;
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



void StartMotionTask(void *argument)
{
    /* USER CODE BEGIN StartMotionTask */
//    NodeMotorType NodeMotor1,NodeMotor2;
//    NodeMotor1.CanHandler = &hcan;
//    NodeMotor1.id = 0x01;
//    NodeMotor1.Mode = Velocity;
//    NodeMotor1.Velocity = 1;
//
//    NodeMotor2.CanHandler = &hcan;
//    NodeMotor2.id = 0x02;
//    NodeMotor2.Mode = Velocity;
//    NodeMotor2.Velocity = -1;

    MotionInit(&PancakeMotion,Velocity);
    /* Infinite loop */
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
//      NodeMotorVelocityControl(&NodeMotor1);
//      NodeMotorVelocityControl(&NodeMotor2);
//      MotionSetLinearVelocity(&PancakeMotion,(1.0*(status - 0)/100*LINEAR_VEL_MAX));
        MotionUpdateVelocity(&PancakeMotion);
        osDelay(30);
    }
    /* USER CODE END StartMotionTask */
}
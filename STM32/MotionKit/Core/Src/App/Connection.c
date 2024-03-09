//
// Created by 李唐 on 2024/3/2.
//

#include "App/Connection.h"

extern MotionType PancakeMotion;

uint8_t bufControlPack[12] = { };
uint8_t ControlPack[5] = { };

ControlFrameType DecodeControlFrame(){
    ControlFrameType controlFrame;
    uint16_t rawLinearData  = (ControlPack[1]<<8 | ControlPack[2]<<0 );
    uint16_t rawAngularData = (ControlPack[3]<<8 | ControlPack[4]<<0 );
    switch (ControlPack[0]) {
        case VelocityMode:
            controlFrame.Mode = VelocityMode;
            controlFrame.LinearVelocity  = (rawLinearData * 1.0 / MAX_INT16 - 1)  * 2 * LINEAR_VEL_MAX ;
            controlFrame.AngularVelocity = (rawAngularData * 1.0 / MAX_INT16 - 1) * ANGULAR_VEL_MAX ;
            break;
         case PositionMode:
            break;
        default:
            break;
    }
    return controlFrame;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance)
    {
            HAL_UART_Receive(&huart2, bufControlPack, 12,HAL_MAX_DELAY);
        }
}

void StartConnectionTask(void *argument)
{

    HAL_UART_Receive_IT(&huart2, bufControlPack, 1);

    for(;;)
    {

        for (int i = 0; i < 12; ++i) {
            if (bufControlPack[i] == '#') {
                ControlPack[0] = bufControlPack[i + 1];
                ControlPack[1] = bufControlPack[i + 2];
                ControlPack[2] = bufControlPack[i + 3];
                ControlPack[3] = bufControlPack[i + 4];
                ControlPack[4] = bufControlPack[i + 5];
                break;
            }
        }
        ControlFrameType controlFrame = DecodeControlFrame();
        switch(controlFrame.Mode){
            case VelocityMode:
                MotionSetLinearVelocity(&PancakeMotion,controlFrame.LinearVelocity);
                MotionSetAngularVelocity(&PancakeMotion,controlFrame.AngularVelocity);
                break;
            default:
                break;
        }

        osDelay(30);
        HAL_UART_Receive_IT(&huart2, bufControlPack, 1);

    }
}
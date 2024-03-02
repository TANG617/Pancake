//
// Created by 李唐 on 2024/3/2.
//

#include "App/Connection.h"
static float uint2float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

ControlFrameType PackageDecode(uint8_t* packageFrame){
    ControlFrameType controlFrame;
    int32_t rawData = (packageFrame[1]<<24)
                    + (packageFrame[2]<<16)
                    + (packageFrame[3]<<8)
                    + (packageFrame[4]<<0);
    switch (packageFrame[0]) {
        case LinearVelocityMode:
            controlFrame.Mode = LinearVelocityMode;
            controlFrame.LinearVelocity = rawData * 1.0 / MAX_INT32 * LINEAR_VEL_MAX ;
            break;
         case LinearPositionMode:
            break;
        case AngularVelocityMode:
            controlFrame.Mode = AngularVelocityMode;
            controlFrame.AngularVelocity = rawData * 1.0 / MAX_INT32 * ANGULAR_VEL_MAX ;
            break;
        case AngularPositionMode:
            break;
        default:
            break;
    }
    return controlFrame;
}

ControlFrameType PackageFetch(){
    uint8_t bufByte[] = "\0";
    uint8_t packageFrame[5];
    while(*bufByte != '#')
    {
        osDelay(10);
        HAL_UART_Receive(&huart2,bufByte,1,HAL_MAX_DELAY);
    }
    HAL_UART_Receive(&huart2,packageFrame ,5,HAL_MAX_DELAY);
    return PackageDecode(&packageFrame);
}
//uint8_t* DSC_GET(int16_t *data)
//{
//    uint8_t buf30[30];
//    uint8_t buf1[] = "\0";
//    while(*buf1 != '#')
//    {
//        HAL_UART_Receive(&huart2,buf1,1,HAL_MAX_DELAY);
//        osDelay(100);
//    }
//    for (uint8_t i = 0; i<30; i++)
//    {
//        HAL_UART_Receive(&huart2, buf30+i ,1,HAL_MAX_DELAY);
//    }
//    // return buf30;
//
//    uint8_t index = 0;
//    DSC_KEYS dsc_keys = 0;
//    int16_t value = 0;
//    int8_t sign = 1;
//    while(buf30[index] != '#')
//        // while(index<5)
//    {
//        if(buf30[index] != '_')
//        {
//            value *= 10;
//            if(buf30[index] == '-')
//            {
//                sign = -1;
//            }
//            else
//            {
//                value += buf30[index] - '0';
//            }
//        }
//        else
//        {
//            value *= sign;
//            // _DSC->Data[dsc_keys++] = value;
//            data[dsc_keys++] = value;
//            sign = 1;
//            value = 0;
//        }
//        index++;
//    }
//    value *= sign;
//    // _DSC->Data[dsc_keys++] = value;
//    data[dsc_keys++] = value;
//}
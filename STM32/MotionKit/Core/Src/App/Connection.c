//
// Created by 李唐 on 2024/3/2.
//

#include "App/Connection.h"
static float uint2float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

uint8_t RawControlFrame[5];
uint8_t bufByte = '\0';
ControlFrameType DecodeControlFrame(){
//    uint8_t packageFrame[5];
//    HAL_UART_Receive(&huart2,packageFrame ,5,HAL_MAX_DELAY);
//    ControlFrameType PackageDecode(uint8_t* packageFrame){
    ControlFrameType controlFrame;
    uint16_t rawLinearData  = (RawControlFrame[1]<<8 | RawControlFrame[2]<<0 );
    uint16_t rawAngularData = (RawControlFrame[3]<<8 | RawControlFrame[4]<<0 );
    switch (RawControlFrame[0]) {
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

//ControlFrameType PackageFetch(){
//    uint8_t bufByte[] = "\0";
//    uint8_t packageFrame[5];
//    while(*bufByte != '#')
//    {
//        osDelay(10);
////        HAL_UART_Receive(&huart2,bufByte,1,HAL_MAX_DELAY);
//        HAL_UART_Receive_IT(&huart2, bufByte, 1);
//    }
//    HAL_UART_Receive(&huart2,packageFrame ,5,HAL_MAX_DELAY);
//    return PackageDecode(&packageFrame);
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance)
    {
//        uint8_t bufByte[] = "\0";
//        HAL_UART_Receive_IT(&huart2, bufByte, 1);
        if(bufByte == '#'){
            HAL_UART_Receive(&huart2,RawControlFrame ,5,HAL_MAX_DELAY);
        }
//        HAL_UART_Receive_IT(&huart2, packageFrame, 1);
        bufByte = '\0';
//        HAL_UART_Receive_IT(&huart2, &bufByte, 1);
    }
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
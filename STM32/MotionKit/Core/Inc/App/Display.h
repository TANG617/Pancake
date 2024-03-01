//
// Created by 李唐 on 2023/11/13.
//

#ifndef DISPLAY_H
#define DISPLAY_H
#include "stm32f1xx_hal.h"
#include "Drv/LCD.h"
//#include "string.h"
//#include "stdlib.h"
//#include "stdarg.h"
//#include "stdio.h"
#include "lvgl/lvgl.h"

//extern lv_disp_draw_buf_t disp_buf;
//extern lv_disp_drv_t disp_drv;
//extern lv_disp_t * disp;
///*Static or global buffer(s). The second buffer is optional*/
//extern lv_color_t buf_1[LCD_Width * 4];

//HAL_StatusTypeDef ShowHelloWorld();
//HAL_StatusTypeDef DisplayNum(uint32_t num);
HAL_StatusTypeDef DisplayInit();
#endif //DISPLAY_H

//
// Created by 李唐 on 2023/11/13.
//

#include "App/Display.h"

extern MotionType PancakeMotion;

int status = 0;

void StartDisplayTask(void *argument)
{
    /* USER CODE BEGIN StartDisplayTask */
    DisplayInit();
    ui_init();

    /* Infinite loop */
    for(;;)
    {
        char MotionVelocity[6];
        lv_tick_inc(30);
        lv_timer_handler();
        float LMotor_Status = (PancakeMotion.LMotor.Velocity * PancakeMotion.LMotor.Direction);
        float RMotor_Status = (PancakeMotion.RMotor.Velocity * PancakeMotion.RMotor.Direction);
        lv_bar_set_value(ui_LMotor,LMotor_Status*100/LINEAR_VEL_MAX,LV_ANIM_ON);
        lv_bar_set_value(ui_RMotor,RMotor_Status*100/LINEAR_VEL_MAX,LV_ANIM_ON);
        lv_arc_set_value(ui_Direction,(LMotor_Status - RMotor_Status)*ARC_SCALE_RATIO);
        sprintf(MotionVelocity, "%2.1f", PancakeMotion.LinearVelocity);
        lv_label_set_text(ui_MainLabel1,MotionVelocity);
        if(PancakeMotion.Enable) lv_obj_add_state(ui_MotorEnable,LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_MotorEnable,LV_STATE_CHECKED);

        osDelay(30);


        if (HAL_GPIO_ReadPin(BTN_PUSH_GPIO_Port, BTN_PUSH_Pin) == 0) {
            status = 0;
        }
        if (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin) == 0) {
            status -= 5;
        }
        if (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin) == 0) {
            status += 5 ;
        }
        status = status % 100;

    }
    /* USER CODE END StartDisplayTask */
}
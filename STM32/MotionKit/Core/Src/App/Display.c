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
//    uint8_t status = 120;
//    static lv_style_t style_bg;
//    static lv_style_t style_indic;
//
//    lv_style_init(&style_bg);
//    lv_style_set_border_color(&style_bg, lv_palette_main(LV_PALETTE_BLUE));
//    lv_style_set_border_width(&style_bg, 2);
//    lv_style_set_pad_all(&style_bg, 6); /*To make the indicator smaller*/
//    lv_style_set_radius(&style_bg, 6);
//    lv_style_set_anim_time(&style_bg, 100);
//
//    lv_style_init(&style_indic);
//    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
//    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
//    lv_style_set_radius(&style_indic, 3);
//
//    lv_obj_t *bar = lv_bar_create(lv_scr_act());
//    lv_obj_remove_style_all(bar);  /*To have a clean start*/
//    lv_obj_add_style(bar, &style_bg, 0);
//    lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);
//
//    lv_obj_set_size(bar, 200, 20);
//    lv_obj_center(bar);
//    lv_bar_set_range(bar,0,255);

    /* Infinite loop */
    for(;;)
    {
//      OverviewScreen(&PancakeMotion);
//      lv_bar_set_value(bar, status%255, LV_ANIM_ON);
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
//      lv_obj_add_state(ui_MotorEnable,PancakeMotion.Enable ? LV_STATE_CHECKED : LV_STATE_DISABLED);
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
//void DisplayInit(){
//    ui_init();
//}
//
//void OverviewScreen(MotionType *PancakeMotion){
//    char MotionVelocity[6];
//    lv_tick_inc(30);
//    lv_timer_handler();
//    float LMotor_Status = (PancakeMotion->LMotor.Velocity * PancakeMotion.LMotor.Direction);
//    float RMotor_Status = (PancakeMotion->RMotor.Velocity * PancakeMotion.RMotor.Direction);
//    lv_bar_set_value(ui_LMotor,LMotor_Status*100/LINEAR_VEL_MAX,LV_ANIM_ON);
//    lv_bar_set_value(ui_RMotor,RMotor_Status*100/LINEAR_VEL_MAX,LV_ANIM_ON);
//    lv_arc_set_value(ui_Direction,(LMotor_Status - RMotor_Status));
//    sprintf(MotionVelocity, "%2.1f", PancakeMotion->RMotor.Velocity);
//    lv_label_set_text(ui_MainLabel1,MotionVelocity+1);
//    if(PancakeMotion->Enable) lv_obj_add_state(ui_MotorEnable,LV_STATE_CHECKED);
//    else lv_obj_clear_state(ui_MotorEnable,LV_STATE_CHECKED);
//}
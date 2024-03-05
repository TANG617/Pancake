//
// Created by 李唐 on 2023/11/13.
//

#include "App/Display.h"
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
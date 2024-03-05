// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.3.6
// Project name: Pancake

#include "../ui.h"
#include "PancakeConfig.h"
extern lv_font_t Montserrat_60_NUM;
void ui_OverviewScreen_screen_init(void)
{
    ui_OverviewScreen = lv_obj_create(NULL);
    lv_obj_clear_flag( ui_OverviewScreen, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
    lv_obj_set_style_bg_color(ui_OverviewScreen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
    lv_obj_set_style_bg_opa(ui_OverviewScreen, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

    ui_LMotor = lv_bar_create(ui_OverviewScreen);
    lv_obj_set_width( ui_LMotor, 15);
    lv_obj_set_height( ui_LMotor, 160);
    lv_obj_set_x( ui_LMotor, -100 );
    lv_obj_set_y( ui_LMotor, 10 );
    lv_obj_set_align( ui_LMotor, LV_ALIGN_CENTER );
    lv_obj_clear_flag( ui_LMotor, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
    lv_bar_set_value(ui_LMotor,0,LV_ANIM_OFF);
    lv_bar_set_range(ui_LMotor,(-1)*BAR_MAX_RANGE,BAR_MAX_RANGE);
    lv_bar_set_mode(ui_LMotor,LV_BAR_MODE_SYMMETRICAL);

    ui_RMotor = lv_bar_create(ui_OverviewScreen);
    lv_obj_set_width( ui_RMotor, 15);
    lv_obj_set_height( ui_RMotor, 160);
    lv_obj_set_x( ui_RMotor, 100 );
    lv_obj_set_y( ui_RMotor, 10 );
    lv_obj_set_align( ui_RMotor, LV_ALIGN_CENTER );
    lv_obj_clear_flag( ui_RMotor, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
    lv_bar_set_value(ui_RMotor,0,LV_ANIM_OFF);
    lv_bar_set_range(ui_RMotor,(-1)*BAR_MAX_RANGE,BAR_MAX_RANGE);
    lv_bar_set_mode(ui_RMotor,LV_BAR_MODE_SYMMETRICAL);

    ui_MotorEnable = lv_checkbox_create(ui_OverviewScreen);
    lv_checkbox_set_text(ui_MotorEnable,"MOTOR");
    lv_obj_set_width( ui_MotorEnable, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height( ui_MotorEnable, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x( ui_MotorEnable, -63 );
    lv_obj_set_y( ui_MotorEnable, -100 );
    lv_obj_set_align( ui_MotorEnable, LV_ALIGN_CENTER );
    lv_obj_clear_flag( ui_MotorEnable, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
    lv_obj_set_style_text_color(ui_MotorEnable, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
    lv_obj_set_style_text_opa(ui_MotorEnable, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_MotorEnable, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

    ui_RosEnable = lv_checkbox_create(ui_OverviewScreen);
    lv_checkbox_set_text(ui_RosEnable,"ROS");
    lv_obj_set_width( ui_RosEnable, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height( ui_RosEnable, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x( ui_RosEnable, 83 );
    lv_obj_set_y( ui_RosEnable, -100 );
    lv_obj_set_align( ui_RosEnable, LV_ALIGN_CENTER );
    lv_obj_clear_flag( ui_RosEnable, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE );    /// Flags
    lv_obj_set_style_text_color(ui_RosEnable, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
    lv_obj_set_style_text_opa(ui_RosEnable, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_RosEnable, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

    ui_Direction = lv_arc_create(ui_OverviewScreen);
    lv_obj_set_width( ui_Direction, 150);
    lv_obj_set_height( ui_Direction, 150);
    lv_obj_set_x( ui_Direction, 0 );
    lv_obj_set_y( ui_Direction, 140 );
    lv_obj_set_align( ui_Direction, LV_ALIGN_CENTER );
    lv_obj_clear_flag( ui_Direction, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
    lv_arc_set_range(ui_Direction, -50,50);
    lv_arc_set_bg_angles(ui_Direction,240,360);
    lv_arc_set_mode(ui_Direction, LV_ARC_MODE_SYMMETRICAL);
    lv_arc_set_rotation(ui_Direction,-30);


    ui_MainLabel1 = lv_label_create(ui_OverviewScreen);
    lv_obj_set_width( ui_MainLabel1, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height( ui_MainLabel1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x( ui_MainLabel1, lv_pct(0) );
    lv_obj_set_y( ui_MainLabel1, lv_pct(-10) );
    lv_obj_set_align( ui_MainLabel1, LV_ALIGN_CENTER );
    lv_label_set_text(ui_MainLabel1,"0");
    lv_obj_clear_flag( ui_MainLabel1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
    lv_obj_set_style_text_color(ui_MainLabel1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
    lv_obj_set_style_text_opa(ui_MainLabel1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_MainLabel1, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN| LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_MainLabel1, &Montserrat_60_NUM, LV_PART_MAIN| LV_STATE_DEFAULT);

}

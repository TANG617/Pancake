//
// Created by 李唐 on 2023/11/13.
//

#include "App/Display.h"
#include "Drv/LCD_Image.h"



/*A static or global variable to store the buffers*/
lv_disp_draw_buf_t disp_buf;
lv_disp_drv_t disp_drv;
lv_disp_t * disp;
/*Static or global buffer(s). The second buffer is optional*/
lv_color_t buf_1[LCD_Width ];
lv_color_t buf_2[LCD_Width ];


/*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */


void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one
     *`put_px` is just an example, it needs to implemented by you.*/
    LCD_Fill(area->x1,area->y1,area->x2,area->y2,(uint16_t *)color_p);
//    int32_t x, y;
//    for(y = area->y1; y <= area->y2; y++) {
//        for(x = area->x1; x <= area->x2; x++) {
////            put_px(x, y, *color_p);
//            LCD_Draw_ColorPoint(x,y,color_p->full);
//            color_p++;
//        }
//    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

HAL_StatusTypeDef DisplayInit(){
    lv_init();
    LCD_Init();
    lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2, LCD_Width);
              /*A variable to hold the drivers. Must be static or global.*/
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/

    disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
    disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = LCD_Width;                 /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = LCD_Height;                 /*Set the vertical resolution in pixels*/


    disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

//    static lv_style_t style;
//    lv_style_init(&style);
//    lv_style_set_flex_flow(&style, LV_FLEX_FLOW_ROW_WRAP);
//    lv_style_set_flex_main_place(&style, LV_FLEX_ALIGN_SPACE_EVENLY);
//    lv_style_set_layout(&style, LV_LAYOUT_FLEX);
//
//    lv_obj_t * cont = lv_obj_create(lv_scr_act());
//    lv_obj_set_size(cont, 240, 240);
//    lv_obj_center(cont);
//    lv_obj_add_style(cont, &style, 0);
//
//    uint32_t i;
//    for(i = 0; i < 8; i++) {
//        lv_obj_t * obj = lv_obj_create(cont);
//        lv_obj_set_size(obj, 40, LV_SIZE_CONTENT);
//        lv_obj_add_flag(obj, LV_OBJ_FLAG_CHECKABLE);
//
//        lv_obj_t * label = lv_label_create(obj);
//        lv_label_set_text_fmt(label, "%"LV_PRIu32, i);
//        lv_obj_center(label);
//    }
}

HAL_StatusTypeDef ShowHelloWorld()
{
    LCD_Show_Image(0,0,240,240,gImage_homer);
}

HAL_StatusTypeDef DisplayNum(uint32_t num)
{
    LCD_ShowNum(0, 0, num, 12, 32);
    return HAL_OK;
}
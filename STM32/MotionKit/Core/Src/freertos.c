/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "can.h"
#include "App/Display.h"
//#include <string.h>
#include <stdio.h>
#include "PancakeConfig.h"
#include "Drv/LCD.h"
#include "App/Motion.h"
#include "App/Connection.h"
#include "App/LVGL_UI/ui.h"
//#include "Drv/NodeMotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t status = 0;
MotionType PancakeMotion;
extern uint8_t bufByte;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 3200 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for ConnectivityTas */
osThreadId_t ConnectivityTasHandle;
const osThreadAttr_t ConnectivityTas_attributes = {
  .name = "ConnectivityTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartDisplayTask(void *argument);
void StartConnectivityTask(void *argument);
void StartMotionTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of ConnectivityTas */
  ConnectivityTasHandle = osThreadNew(StartConnectivityTask, NULL, &ConnectivityTas_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//      HAL_UART_Transmit(&huart2,"HELLO\n",6,HAL_MAX_DELAY);

      osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
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

/* USER CODE BEGIN Header_StartConnectivityTask */
//extern uint8_t bufByte;
/**
* @brief Function implementing the ConnectivityTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartConnectivityTask */
void StartConnectivityTask(void *argument)
{
  /* USER CODE BEGIN StartConnectivityTask */
//    uint8_t bufByte[] = "\0";
    uint8_t packageFrame[5];
    HAL_UART_Receive_IT(&huart2, &bufByte, 1);
  /* Infinite loop */
  for(;;)
  {
//      ControlFrameType controlFrame = PackageFetch();
      ControlFrameType controlFrame = DecodeControlFrame();
      switch(controlFrame.Mode){
          case VelocityMode:
              MotionSetLinearVelocity(&PancakeMotion,controlFrame.LinearVelocity);
              MotionSetAngularVelocity(&PancakeMotion,controlFrame.AngularVelocity);
              break;
          default:
              break;
      }
      HAL_UART_Receive_IT(&huart2, &bufByte, 1);
      osDelay(30);
  }
  /* USER CODE END StartConnectivityTask */
}

/* USER CODE BEGIN Header_StartMotionTask */
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotionTask */
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
      osDelay(100);
  }
  /* USER CODE END StartMotionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


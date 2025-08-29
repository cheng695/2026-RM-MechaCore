/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Variables */
/* Definitions for PIDControlH */
osThreadId_t PIDControlHHandle;
const osThreadAttr_t PIDControlH_attributes = {
  .name = "PIDControlH",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EventH */
osThreadId_t EventHHandle;
const osThreadAttr_t EventH_attributes = {
  .name = "EventH",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GimbalH */
osThreadId_t GimbalHHandle;
const osThreadAttr_t GimbalH_attributes = {
  .name = "GimbalH",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UartH */
osThreadId_t UartHHandle;
const osThreadAttr_t UartH_attributes = {
  .name = "UartH",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VisionSendH */
osThreadId_t VisionSendHHandle;
const osThreadAttr_t VisionSendH_attributes = {
  .name = "VisionSendH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TimerH */
osThreadId_t TimerHHandle;
const osThreadAttr_t TimerH_attributes = {
  .name = "TimerH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Queue_DT7ToGimbal */
osMessageQueueId_t Queue_DT7ToGimbalHandle;
const osMessageQueueAttr_t Queue_DT7ToGimbal_attributes = {
  .name = "Queue_DT7ToGimbal"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PIDControl(void *argument);
void EventReport(void *argument);
void Gimbal(void *argument);
void UartSend(void *argument);
void VisionSend(void *argument);
void TimerCallback(void *argument);

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

  /* Create the queue(s) */
  /* creation of Queue_DT7ToGimbal */
  Queue_DT7ToGimbalHandle = osMessageQueueNew (1, 18, &Queue_DT7ToGimbal_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PIDControlH */
  PIDControlHHandle = osThreadNew(PIDControl, NULL, &PIDControlH_attributes);

  /* creation of EventH */
  EventHHandle = osThreadNew(EventReport, NULL, &EventH_attributes);

  /* creation of GimbalH */
  GimbalHHandle = osThreadNew(Gimbal, NULL, &GimbalH_attributes);

  /* creation of UartH */
  UartHHandle = osThreadNew(UartSend, NULL, &UartH_attributes);

  /* creation of VisionSendH */
  VisionSendHHandle = osThreadNew(VisionSend, NULL, &VisionSendH_attributes);

  /* creation of TimerH */
  TimerHHandle = osThreadNew(TimerCallback, NULL, &TimerH_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_PIDControl */
/**
  * @brief  Function implementing the PIDControlH thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_PIDControl */
__weak void PIDControl(void *argument)
{
  /* USER CODE BEGIN PIDControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PIDControl */
}

/* USER CODE BEGIN Header_EventReport */
/**
* @brief Function implementing the EventH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EventReport */
__weak void EventReport(void *argument)
{
  /* USER CODE BEGIN EventReport */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EventReport */
}

/* USER CODE BEGIN Header_Gimbal */
/**
* @brief Function implementing the GimbalH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal */
__weak void Gimbal(void *argument)
{
  /* USER CODE BEGIN Gimbal */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal */
}

/* USER CODE BEGIN Header_UartSend */
/**
* @brief Function implementing the UartH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartSend */
__weak void UartSend(void *argument)
{
  /* USER CODE BEGIN UartSend */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UartSend */
}

/* USER CODE BEGIN Header_VisionSend */
/**
* @brief Function implementing the VisionSendH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisionSend */
__weak void VisionSend(void *argument)
{
  /* USER CODE BEGIN VisionSend */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VisionSend */
}

/* USER CODE BEGIN Header_TimerCallback */
/**
* @brief Function implementing the TimerH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TimerCallback */
__weak void TimerCallback(void *argument)
{
  /* USER CODE BEGIN TimerCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


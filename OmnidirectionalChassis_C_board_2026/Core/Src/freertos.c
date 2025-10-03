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
osThreadId defaultTaskHandle;
osThreadId chassis_ctrlHandle;
osThreadId chassis_monitorHandle;
osThreadId capacitorHandle;
osThreadId chassis_commHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Chassis_Ctrl(void const * argument);
void Chassis_Monitor(void const * argument);
void Capacitor(void const * argument);
void Chassis_Comm(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of chassis_ctrl */
  osThreadDef(chassis_ctrl, Chassis_Ctrl, osPriorityRealtime, 0, 1024);
  chassis_ctrlHandle = osThreadCreate(osThread(chassis_ctrl), NULL);

  /* definition and creation of chassis_monitor */
  osThreadDef(chassis_monitor, Chassis_Monitor, osPriorityHigh, 0, 512);
  chassis_monitorHandle = osThreadCreate(osThread(chassis_monitor), NULL);

  /* definition and creation of capacitor */
  osThreadDef(capacitor, Capacitor, osPriorityAboveNormal, 0, 512);
  capacitorHandle = osThreadCreate(osThread(capacitor), NULL);

  /* definition and creation of chassis_comm */
  osThreadDef(chassis_comm, Chassis_Comm, osPriorityNormal, 0, 512);
  chassis_commHandle = osThreadCreate(osThread(chassis_comm), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Chassis_Ctrl */
/**
* @brief Function implementing the chassis_ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Ctrl */
void Chassis_Ctrl(void const * argument)
{
  /* USER CODE BEGIN Chassis_Ctrl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Ctrl */
}

/* USER CODE BEGIN Header_Chassis_Monitor */
/**
* @brief Function implementing the chassis_monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Monitor */
void Chassis_Monitor(void const * argument)
{
  /* USER CODE BEGIN Chassis_Monitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Monitor */
}

/* USER CODE BEGIN Header_Capacitor */
/**
* @brief Function implementing the capacitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Capacitor */
void Capacitor(void const * argument)
{
  /* USER CODE BEGIN Capacitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Capacitor */
}

/* USER CODE BEGIN Header_Chassis_Comm */
/**
* @brief Function implementing the chassis_comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Comm */
void Chassis_Comm(void const * argument)
{
  /* USER CODE BEGIN Chassis_Comm */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Comm */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

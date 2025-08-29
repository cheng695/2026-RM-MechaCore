#ifndef __InH_H
#define __InH_H
#include "queue.h"
#ifdef __cplusplus
extern "C" {
#endif
/* C代码对C++的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
	
extern QueueHandle_t xQueue;
extern osMessageQueueId_t Queue_DT7ToGimbalHandle;

__weak void PIDControl(void *argument);
__weak void EventReport(void *argument);
__weak void Gimbal(void *argument);
__weak void UartSend(void *argument);
__weak void VisionSend(void *argument);
__weak void TimerCallback(void *argument);

/* USER CODE END Includes */

/* C代码对C++的声明  -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif


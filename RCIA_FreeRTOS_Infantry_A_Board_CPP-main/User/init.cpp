#include "init.hpp"
#include "can.hpp"
#include "uart.hpp"
#include "tim.h"
#include "vision.hpp"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include "dm4310_ctrl.h"

#ifdef __cplusplus
}
#endif  // __cplusplus

void init() {
    UartInit();
    CanInit();
    HAL_TIM_Base_Start_IT(&htim7);
	dm4310_motor_init();
	ctrl_enable();
	ctrl_enable();
	ctrl_enable();
	
	vision_flag = 1;
}

/**
 * @file init.cpp
 * @author XMX
 * @brief 初始化函数
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "init.hpp"
#include "can.hpp"
#include "uart.hpp"
#include "tim.h"
#include "power_limit.hpp"
#include "freertos.h"
/// @brief 初始化
void init() {
    UartInit();
    CanInit();
    HAL_TIM_Base_Start_IT(&htim7);		
	PowerControl.Wheel_PowerData.k1 = 6.9157e-07;
    PowerControl.Wheel_PowerData.k2 = 7.0767e-07;
			//power_care.k2=0.00000025;
		//power_care.a=0.00000025;
}

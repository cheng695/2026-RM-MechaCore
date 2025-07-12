/**
 * @file capacity_task.cpp
 * @author XMX
 * @brief 电容查询及发送任务
 * @version 1.0
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "capacity_task.hpp"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "variables.hpp"

uint16_t more_power;

/**
 * @brief 电容查询及功率设置
 * 
 * @param argument 
 */
void CapacityAskTask(void* argument) {
    for (;;) {
        //  ============================= 安合电容 =================================
        capacity.EnableOutput();
        osDelay(1);
        capacity.AskVoltage();
        osDelay(1);
        capacity.AskInputPower();
//        osDelay(1);
//        capacity.SetMaxChargePower(more_power);
        osDelay(1);
		capacity.SetMaxChargePower(referee.robot_status.chassis_power_limit + 4.9);
        //  ========================================================================

        // capacity.Ask_RCIA();  //自研电容数据查询
        // osDelay(250);
        // capacity.SetMaxChargePower_RCIA();  //自研电容组功率设置
        // osDelay(250);
    }
}

/**
 * @brief 电容充电功率发送任务（未使用 ——2024/8/7）
 * 
 * @param argument 
 */
void CapacityChargeTask(void* argument) {
    for (;;) {
        // //buffer_energy最大为60
        // auto normalize_buffer = (referee.power_heat_data.buffer_energy - 20) / 40.0f;
        // //2.0f指的是2W功率，超电协议需放大1000倍
        // uint16_t more_power = (normalize_buffer * 2.0f) * 1000.0f;
        // capacity.SetMaxChargePower(more_power);

        //绝对时间
        auto tick = osKernelGetTickCount();
        tick += pdMS_TO_TICKS(20);
        osDelayUntil(tick);
    }
}
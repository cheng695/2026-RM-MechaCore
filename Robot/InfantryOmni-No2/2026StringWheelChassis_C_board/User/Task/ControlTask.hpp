#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"

/*
 * 底盘控制任务对外只暴露任务入口和初始化兼容接口。
 * 底盘目标、输出、状态机等共享对象统一放在 RobotContext.hpp。
 */
void fsm_init();

extern "C" {
void Control(void const *argument);
}

#endif

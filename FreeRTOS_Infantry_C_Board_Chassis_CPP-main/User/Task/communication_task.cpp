/**
 * @file communication_task.cpp
 * @author XMX
 * @brief 板间通信发送任务
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "communication_task.hpp"
#include "cmsis_os2.h"
#include "variables.hpp"
#include "UI_Queue.hpp"
#include "darw_static.hpp"
#include "darw_dynamic.hpp"
/// @brief 板间通信发送任务
/// @param argument 

bool ui_brush;
void CommunicationTask(void* argument) {
	 UI::Static::UI_static.Init();

    for (;;) {
        comm.Send();
		UI::Dynamic::UI_dynamic.darw_UI();
		UI::UI_send_queue.send();
        osDelay(1);
    }
}
	
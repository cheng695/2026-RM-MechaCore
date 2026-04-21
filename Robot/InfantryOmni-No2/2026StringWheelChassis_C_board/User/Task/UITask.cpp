#include "UITask.hpp"

// 引入系统状态变量（根据你的实际控制框架可能需要 extern 或 include）
// extern uint8_t shoot_mode; 
// extern float gimbal_pitch;

ui_ns::darw_static UI_static;

extern "C" {
void UI(void const * argument)
{
    // 等待裁判系统上电稳定
    osDelay(3000);

    // 1. 调用一次静态 Init，把我们需要画的准星、圆环、刻度全部装入待发队列
    
    UI_static.Init();

    uint32_t last_rebuild_tick = osKernelSysTick();

    for(;;)
    {
        // 1. 每隔一段时间（例如 5000ms）触发一次“软重建”，防止裁判系统断电重启导致屏幕清空
        if (osKernelSysTick() - last_rebuild_tick > 5000)
        {
            UI_static.Init();
            last_rebuild_tick = osKernelSysTick();
        }

        // // 2. 队列会自动每隔 100ms（或者根据底层计数）向串口吐出一点点数据，防堵塞
        UI_static.send_wz(); 
        UI_static.send();

        // RTOS 任务延时
        osDelay(10);
    }
}
}

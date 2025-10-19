#include "User/Task/Inc/motorTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

uint32_t pos = 0;
int16_t vel = 0;

static uint32_t last_time;
static float now_time;
void MotorTask(void *argument)
{
    // 获取CAN设备实例
    auto &chassis_can = CAN_INSTANCE.get_device(CHASSIS_CAN);
    // 注册电机的Parse函数作为CAN接收回调
    Motor6020.registerCallback(&chassis_can);

    // 添加回调函数用来显示log
    chassis_can.register_rx_callback([](const HAL::CAN::Frame &frame) {
        pos = static_cast<uint32_t>(Motor6020.getAngleDeg(2));
        LOG.trace("Pos:%d\n", pos);
    });

    for (;;)
    {
        // 获取地址
        auto &can_motor = CAN_INSTANCE.get_device(CHASSIS_CAN);

        // 设置要转动的电机的数据与ID号
        Motor6020.setCAN(vel, 2);

        // 发送数据
        Motor6020.sendCAN(&can_motor);

        // 获取时间
        now_time = DWTimer.GetDeltaT(&last_time);

        osDelay(1);
    }
}

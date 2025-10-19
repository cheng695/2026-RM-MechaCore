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

    chassis_can.register_rx_callback([](const HAL::CAN::Frame &frame) {
        pos = static_cast<uint32_t>(Motor6020.getAngleDeg(2));
        LOG.trace("Pos:%d\n", now_time);
    });

    for (;;)
    {
        auto &can_motor = CAN_INSTANCE.get_device(CHASSIS_CAN);

        Motor6020.setCAN(vel, 2);
        Motor6020.sendCAN(&can_motor);

        now_time = DWT.GetDeltaT(&last_time);

        osDelay(1);
    }
}

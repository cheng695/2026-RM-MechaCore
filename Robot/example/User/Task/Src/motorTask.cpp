#include "User/Task/Inc/motorTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

uint32_t pos = 0;

void Init(void)
{
    // 获取CAN设备实例
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    // 注册电机的Parse函数作为CAN接收回调
    Motor6020.registerCallback(&can1);

    // 注册一个额外的回调用于打印位置信息
}

void MotorTask(void *argument)
{
    // 获取CAN设备实例
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    // 注册电机的Parse函数作为CAN接收回调
    Motor6020.registerCallback(&can1);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
        pos = static_cast<uint32_t>(Motor6020.getAngleDeg(2));
        LOG.trace("Pos:%d\n", pos);
    });

    for (;;)
    {
        auto &can_motor = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

        Motor6020.setCAN(5000, 2);
        Motor6020.sendCAN(&can_motor);

        // pos = static_cast<uint32_t>(Motor6020.getAngleDeg(2));
        // LOG.trace("Pos:%d\n", pos);

        osDelay(1);
    }
}

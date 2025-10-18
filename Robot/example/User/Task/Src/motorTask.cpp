#include "User/Task/Inc/motorTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

void MotorTask(void *argument)
{
    for (;;)
    {
        auto &can_motor = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

        Motor6020.setCAN(5000, 2);
        Motor6020.sendCAN(&can_motor);

        osDelay(1);
    }
}

uint8_t data[8] = {0};

HAL::CAN::Frame rx_frame;
uint32_t pos = 0;

// CAN接收回调函数需要用extern "C"声明
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        if (can1.receive(rx_frame))
        {
            Motor6020.Parse(rx_frame);

            pos = static_cast<uint32_t>(Motor6020.getAngleDeg(2));
            LOG.trace("Pos:%d\n", pos);
        }
    }
}

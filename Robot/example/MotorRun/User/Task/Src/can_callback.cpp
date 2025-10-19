#include "HAL/CAN/can_hal.hpp"

// CAN接收中断回调 - receive()会自动触发所有注册的回调
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can.get_handle())
    {
        can.receive(rx_frame); // receive()内部会自动触发回调
    }
}

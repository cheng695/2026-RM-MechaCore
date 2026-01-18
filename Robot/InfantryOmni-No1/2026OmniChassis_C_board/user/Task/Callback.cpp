#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "RemoteTask.hpp"

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};
        auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
            
        if(huart == uart3.get_handle())
        {
            uart3.receive_dma_idle(uart3_rx_buffer);
            uart3.trigger_rx_callbacks(uart3_rx_buffer);
            uart3.clear_ore_error(uart3_rx_buffer);
        }
    }
}


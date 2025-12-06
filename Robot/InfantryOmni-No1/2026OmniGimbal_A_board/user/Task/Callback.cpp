#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "RemoteTask.hpp"
#include "../user/core/HAL/LOGGER/logger.hpp"


extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame1;
    HAL::CAN::Frame rx_frame2;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    //auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame1);  
    }
    // else if (hcan == can2.get_handle())
    // {
    //     can2.receive(rx_frame2);  
    // }

}

extern "C" 
{
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        if (huart->Instance == USART6)
        {
            HAL::UART::Data uart6_rx_buffer{CommunicationData, 18};
            auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            
            if(huart == uart6.get_handle())
            {
                uart6.receive_dma_idle(uart6_rx_buffer);
                uart6.trigger_rx_callbacks(uart6_rx_buffer);
            }
        }
        else if(huart->Instance == UART8)
        {
            HAL::UART::Data uart8_rx_buffer{IMUData, sizeof(IMUData)};
            auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
            
            if(huart == uart8.get_handle())
            {
                uart8.receive_dma_idle(uart8_rx_buffer);
                uart8.trigger_rx_callbacks(uart8_rx_buffer);
            }
        }
    }
}

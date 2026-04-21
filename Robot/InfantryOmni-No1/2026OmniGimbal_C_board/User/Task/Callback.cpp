#include "MotorTask.hpp"

/* CAN回调 ---------------------------------------------------------------------------------------------*/
// fifo0
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can_bus = HAL::CAN::get_can_bus_instance();

    if (hcan->Instance == CAN1)
    {
        can_bus.get_can1().receive(rx_frame);
    }
    else if (hcan->Instance == CAN2)
    {
        can_bus.get_can2().receive(rx_frame);
    }
}
// fifo1
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can_bus = HAL::CAN::get_can_bus_instance();

    if (hcan->Instance == CAN1)
    {
        can_bus.get_can1().receive(rx_frame);
    }
    else if (hcan->Instance == CAN2)
    {
        can_bus.get_can2().receive(rx_frame);
    }
}

/* UART回调 ---------------------------------------------------------------------------------------------*/
extern "C" 
{
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        // 串口1
        if (huart->Instance == USART1)
        {
            HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
            auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
            
            if(huart == uart1.get_handle())
            {
                uart1.receive_dma_idle(uart1_rx_buffer);
                uart1.trigger_rx_callbacks(uart1_rx_buffer);
            }
        }
        // 串口3
        else if(huart->Instance == USART3)
        {
            HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
            auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
            
            if(huart == uart3.get_handle())
            {
                uart3.receive_dma_idle(uart3_rx_buffer);
                uart3.trigger_rx_callbacks(uart3_rx_buffer);
            }
        }
        //// 串口6
        // else if(huart->Instance == USART6)
        // {
        //     HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
        //     auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            
        //     if(huart == uart6.get_handle())
        //     {
        //         uart6.receive_dma_idle(uart6_rx_buffer);
        //         uart6.trigger_rx_callbacks(uart6_rx_buffer);
        //     }
        // }
    }
}

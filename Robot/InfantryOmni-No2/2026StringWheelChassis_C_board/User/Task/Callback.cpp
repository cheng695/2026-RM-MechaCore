#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp"
#include "../core/APP/Referee/RM_RefereeSystem.h"

/* CAN回调 ---------------------------------------------------------------------------------------------*/
// fifo0
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

// fifo1
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        can2.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

/* 串口回调 ---------------------------------------------------------------------------------------------*/
extern "C" 
{
    // 处理DMA接收中的空闲线路检测
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        // 板间通讯
        if(huart->Instance == USART6)
        {
            HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            
            if(huart == uart6.get_handle())
            {
                uart6.receive_dma_idle(uart6_rx_buffer);
                uart6.trigger_rx_callbacks(uart6_rx_buffer);
            }
        }
        // 裁判系统通讯
        else if (huart->Instance == USART1) 
        {
            // 使用系统传进来的 Size 表示这一包真正的长度，而不是写死 512
            HAL::UART::Data uart1_rx_buffer{referee_buffer, Size};
            auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
            
            if(huart == uart1.get_handle())
            {
                HAL::UART::Data next_rx_buffer{referee_buffer, 512};
                if (!uart1.receive_dma_idle(next_rx_buffer))
                {
                    uart1.receive_dma_idle(next_rx_buffer);
                }
                uart1.trigger_rx_callbacks(uart1_rx_buffer);
            }
        }

        // // 遥控器
        // else if(huart->Instance == USART3)
        // {
        //     HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
        //     auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
            
        //     if(huart == uart3.get_handle())
        //     {
        //         uart3.receive_dma_idle(uart3_rx_buffer);
        //         uart3.trigger_rx_callbacks(uart3_rx_buffer);
        //     }
        // }
    }

    // 定长数据接收
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
    }

/* 错误处理 ---------------------------------------------------------------------------------------------*/
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == USART1) // 裁判系统
        {
             // 清除错误标志并重新开启接收
            // Read SR/DR once to fully flush stale byte state in hardware.
            volatile uint32_t sr = huart->Instance->SR;
            volatile uint32_t dr = huart->Instance->DR;
            (void)sr;
            (void)dr;

            __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            
            // 强制解锁HAL库的状态机，这是解决HAL_BUSY导致DMA无法重启的关键
            huart->RxState = HAL_UART_STATE_READY;
            huart->Lock = HAL_UNLOCKED;

            // 错误后也需要重启接收
            HAL::UART::Data uart1_rx_buffer{referee_buffer, sizeof(referee_buffer)};
            auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
            uart1.receive_dma_idle(uart1_rx_buffer);
        }
        else if (huart->Instance == USART6) // 板间通讯
        {
            volatile uint32_t sr = huart->Instance->SR;
            volatile uint32_t dr = huart->Instance->DR;
            (void)sr;
            (void)dr;

            __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
            __HAL_UART_CLEAR_OREFLAG(huart);
            HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            uart6.receive_dma_idle(uart6_rx_buffer);
        }
        // else if (huart->Instance == USART3)  // 遥控器
        // {
        //     __HAL_UART_CLEAR_OREFLAG(huart);
        //     HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
        //     auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
        //     uart3.receive_dma_idle(uart3_rx_buffer);
        // }
    }
}

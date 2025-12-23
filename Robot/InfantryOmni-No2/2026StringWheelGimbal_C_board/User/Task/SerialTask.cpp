#include "SerialTask.hpp"

BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];
BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];

void ImuInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    
    HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
    HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};

    uart1.receive_dma_idle(uart1_rx_buffer);
    uart3.receive_dma_idle(uart3_rx_buffer);
    uart1.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 82 && data.buffer != nullptr)
        {
            HI12.DataUpdate(data.buffer);
        }
    });
    uart3.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 18 && data.buffer != nullptr)
        {
            DT7.parseData(data.buffer);
        }
    });
}

extern "C" {
void imu(void const * argument)
{
    ImuInit();
    for(;;)
    {
        osDelay(1);
    }
}

}

#include "ImuTask.hpp"

BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

void ImuInit()
{
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, 82};
    uart8.receive_dma_idle(uart8_rx_buffer);
    uart8.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 82 && data.buffer != nullptr)
        {
            HI12.DataUpdate(data.buffer);
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

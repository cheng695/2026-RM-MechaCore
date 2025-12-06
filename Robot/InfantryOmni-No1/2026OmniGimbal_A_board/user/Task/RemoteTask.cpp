#include "RemoteTask.hpp"

uint8_t IMUData[82];
BSP::IMU::HI12_float HI12;

void IMUInit()
{
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    HAL::UART::Data uart8_rx_buffer{IMUData, sizeof(IMUData)};
    uart8.receive_dma_idle(uart8_rx_buffer);
    uart8.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == sizeof(IMUData) && data.buffer != nullptr)
        {
            HI12.DataUpdate(data.buffer);
        }
    });
}

extern "C" {
void Remote(void const * argument)
{
    IMUInit();
    for(;;)
    {
        osDelay(5);
    }
}

}

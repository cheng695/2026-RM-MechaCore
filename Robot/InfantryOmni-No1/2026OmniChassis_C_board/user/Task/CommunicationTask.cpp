#include "CommunicationTask.hpp"

uint8_t CommunicationData[18];

void BoardCommunicationInit()
{
    // auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    // HAL::UART::Data uart1_tx_buffer{{DT7Rx_buffer, 18}}
    // uart3.receive_dma_idle(uart3_rx_buffer);
    // uart3.register_rx_callback([](const HAL::UART::Data &data) 
    // {
    //     if(data.size == 18 && data.buffer != nullptr)
    //     {
    //         DT7.parseData(data.buffer);
    //     }
    // });
}

void BoardCommunicationTX()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    memcpy(CommunicationData, DT7Rx_buffer, sizeof(DT7Rx_buffer));
    HAL::UART::Data uart1_tx_buffer{CommunicationData, sizeof(CommunicationData)};
    uart1.transmit_dma(uart1_tx_buffer);
}


extern "C" {
void Communication(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        BoardCommunicationTX();
        osDelay(500);
    }
}

}

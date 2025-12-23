#include "CommunicationTask.hpp"

BoardCommunication Cboard;
uint8_t CommunicationData[18];
uint8_t send_str2[sizeof(float) * 8]; 

void BoardCommunicationInit()
{
    // auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    // HAL::UART::Data uart6_rx_buffer{CommunicationData, sizeof(CommunicationData)};
    // uart6.receive_dma_idle(uart6_rx_buffer);
    // uart6.register_rx_callback([](const HAL::UART::Data &data) 
    // {
    //     if(data.size >= 18 && data.buffer != nullptr)
    //     {
    //         Cboard.updateTimestamp();
    //         DT7.parseData(data.buffer);
    //     }
    // });
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

}

void BoardCommunicationTX()
{

}


extern "C" {
void communication(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        //vofa_send(1, 2, 3, 4, 5, 6);
        BoardCommunicationTX();
        osDelay(5);
    }
}

}

#include "CommunicationTask.hpp"
#include "../User/Task/MotorTask.hpp"
#include "../User/Task/SerialTask.hpp"

/* 实例通讯设备 ---------------------------------------------------------------------------------------------*/
BoardCommunication Cboard;      // 板间通讯
Supercapacitor supercap(0x666); // 超级电容

/* 通讯数据缓存 ---------------------------------------------------------------------------------------------*/
uint8_t BoardTx[4];    // 发送上板
uint8_t BoardRx[23];    // 接收上板
uint8_t send_str2[sizeof(float) * 11];  // 发送vofa

/* 板间通讯 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 板件通讯串口接收回调与数据解析
 * 
 */
void BoardCommunicationInit()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
    
    uart6.receive_dma_idle(uart6_rx_buffer);
    uart6.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size >= 18 && data.buffer != nullptr)
        {
            Cboard.updateTimestamp();
            DT7.parseData(data.buffer);
            Cboard.SetYawAngle(data.buffer+18);
            Cboard.SetScroll(data.buffer+22);
        }
    });
}

/**
 * @brief vofa发送
 * 
 */

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9) 
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;
    *((float*)&send_str2[sendSize * 6]) = x7;
    *((float*)&send_str2[sendSize * 7]) = x8;
    *((float*)&send_str2[sendSize * 8]) = x9;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 9]) = 0x7F800000; // 小端存储为 00 00 80 7F
}

/**
 * @brief 板间通讯发送
 * 
 */
void BoardCommunicationTX()
{
    // vofa
    // auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    // HAL::UART::Data uart6_tx_buffer{send_str2, sizeof(send_str2)}; 
    // uart6.transmit_dma(uart6_tx_buffer);

    // 发给上板裁判系统数据

    memcpy(BoardTx, &ext_power_heat_data_0x0201.shooter_barrel_heat_limit, sizeof(ext_power_heat_data_0x0201.shooter_barrel_heat_limit));
    memcpy(BoardTx+2, &ext_power_heat_data_0x0201.shooter_barrel_cooling_value, sizeof(ext_power_heat_data_0x0201.shooter_barrel_cooling_value));
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_tx_buffer{BoardTx, sizeof(BoardTx)}; 
    uart6.transmit_dma(uart6_tx_buffer);
}


extern "C" {
void Communication(void const * argument) 
{
    BoardCommunicationInit();
    for(;;)
    {
        //vofa_send(supercap.GetPower_10times(), ext_power_heat_data_0x0201.chassis_power_limit, supercap.GetCurrentEnergy(), 1288.5f, 250.0f, energy_ring.GetPowerMax(), supercap.GetPower(), ext_power_heat_data_0x0202.chassis_power_buffer, 60.0f);
        BoardCommunicationTX();
        osDelay(5);
    }
}

}

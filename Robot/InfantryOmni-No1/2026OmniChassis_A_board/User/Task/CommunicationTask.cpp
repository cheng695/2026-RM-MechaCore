#include "CommunicationTask.hpp"
#include "../User/Task/MotorTask.hpp"
#include "../User/Task/SerialTask.hpp"

/* 瀹炰緥閫氳璁惧 ---------------------------------------------------------------------------------------------*/
BoardCommunication Cboard;      // 鏉块棿閫氳
Supercapacitor supercap(0x666); // 瓒呯骇鐢靛

/* 閫氳鏁版嵁缂撳瓨 ---------------------------------------------------------------------------------------------*/
uint8_t BoardTx[4];    // 鍙戦€佷笂鏉?
uint8_t BoardRx[64];    // 鎺ユ敹涓婃澘
uint8_t send_str2[sizeof(float) * 11];  // 鍙戦€乿ofa

/* 鏉块棿閫氳 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 鏉夸欢閫氳涓插彛鎺ユ敹鍥炶皟涓庢暟鎹В鏋?
 * 
 */
void BoardCommunicationInit()
{
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    HAL::UART::Data uart7_rx_buffer{BoardRx, sizeof(BoardRx)};
    
    uart7.receive_dma_idle(uart7_rx_buffer);
    uart7.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size >= 64 && data.buffer != nullptr)
        {
            Cboard.updateTimestamp();
            DT7.parseData(data.buffer);
            Cboard.SetYawAngle(data.buffer+18);
            Cboard.SetScroll(data.buffer+22);
            Cboard.SetLaunchFSMInput(data.buffer+23);
        }
    });
}

/**
 * @brief vofa鍙戦€?
 * 
 */

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9) 
{
    const uint8_t sendSize = sizeof(float); // 鍗曟诞鐐规暟鍗?瀛楄妭

    // 灏?涓诞鐐规暟鎹啓鍏ョ紦鍐插尯锛堝皬绔ā寮忥級
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;
    *((float*)&send_str2[sendSize * 6]) = x7;
    *((float*)&send_str2[sendSize * 7]) = x8;
    *((float*)&send_str2[sendSize * 8]) = x9;

    // 鍐欏叆甯у熬锛堝崗璁姹?0x00 0x00 0x80 0x7F锛?
    *((uint32_t*)&send_str2[sizeof(float) * 9]) = 0x7F800000; // 灏忕瀛樺偍涓?00 00 80 7F
}

/**
 * @brief 鏉块棿閫氳鍙戦€?
 * 
 */
void BoardCommunicationTX()
{
    // vofa
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    HAL::UART::Data uart8_tx_buffer{send_str2, sizeof(send_str2)}; 
    uart8.transmit_dma(uart8_tx_buffer);

    // 鍙戠粰涓婃澘瑁佸垽绯荤粺鏁版嵁
    // memcpy(BoardTx, &ext_power_heat_data_0x0201.shooter_barrel_heat_limit, sizeof(ext_power_heat_data_0x0201.shooter_barrel_heat_limit));
    // memcpy(BoardTx+2, &ext_power_heat_data_0x0201.shooter_barrel_cooling_value, sizeof(ext_power_heat_data_0x0201.shooter_barrel_cooling_value));
    // auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    // HAL::UART::Data uart7_tx_buffer{BoardTx, sizeof(BoardTx)}; 
    // uart7.transmit_dma(uart7_tx_buffer);
}


extern "C" {
void Communication(void const * argument) 
{
    BoardCommunicationInit();
    for(;;)
    {
        vofa_send(Cboard.GetLaunchState(), chassis_target.target_dial, chassis_output.out_dial, MotorLK4005.getAddAngleDeg(1), 250.0f, omni_ik.GetMotor(0), Motor3508.getVelocityRads(1), omni_ik.GetMotor(1), Motor3508.getVelocityRads(2));
        BoardCommunicationTX();
        osDelay(5);
    }
}

}




#include "CommunicationTask.hpp"
#include "MotorTask.hpp"
#include "usbd_cdc_if.h"
#include <cstdint>

/* 瀹炰緥鏉块棿閫氳璁惧 -----------------------------------------------------------------------------------------*/
BoardCommunication Cboard;
Vision vision;

/* 璁惧缂撳瓨 ------------------------------------------------------------------------------------------------*/
uint8_t BoardRx[sizeof(BoardCommunication::BoardRx)];    // 接收下板：热量数据 + 底盘小陀螺状态
uint8_t BoardTx[23];   // 鍙戦€佷笅鏉?
uint8_t send_str2[sizeof(float) * 8];
uint32_t send_time = 0;
uint32_t demo_time = 0;

/* 鏉块棿閫氳 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 鏉夸欢閫氳鍒濆鍖?(涓婁笅琛屽潎宸茶縼绉昏嚦 CAN2, 鐢?MotorTask.cpp 澶勭悊)
 */
void BoardCommunicationInit()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};

    uart6.receive_dma_idle(uart6_rx_buffer);
    uart6.register_rx_callback([](const HAL::UART::Data &data)
    {
            Cboard.updateTimestamp();
            Cboard.SetHeatCool(data.buffer);
            Cboard.SetHeatLimit(data.buffer+2);
            Cboard.SetChassisSpin(data.buffer+4);
    });
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    const uint8_t sendSize = sizeof(float); // 鍗曟诞鐐规暟鍗?瀛楄妭

    // 灏?涓诞鐐规暟鎹啓鍏ョ紦鍐插尯锛堝皬绔ā寮忥級
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 鍐欏叆甯у熬锛堝崗璁姹?0x00 0x00 0x80 0x7F锛?
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 灏忕瀛樺偍涓?00 00 80 7F

}

/* 瑙嗚閫氳 ------------------------------------------------------------------------------------------------*/
void Vision::Data_send()
{
    frame.head_one = 0x39;
    frame.head_two = 0x39;
    
    tx_gimbal.quat_w = HI12.GetQuaternion(0);
    tx_gimbal.quat_x = HI12.GetQuaternion(1);
    tx_gimbal.quat_y = HI12.GetQuaternion(2);
    tx_gimbal.quat_z = HI12.GetQuaternion(3);

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52;
    
    tx_other.tail = 0xFF;

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    memcpy(&Tx_pData[2],  &tx_gimbal.quat_w, sizeof(float));
    memcpy(&Tx_pData[6],  &tx_gimbal.quat_x, sizeof(float));
    memcpy(&Tx_pData[10], &tx_gimbal.quat_y, sizeof(float));
    memcpy(&Tx_pData[14], &tx_gimbal.quat_z, sizeof(float));

    Tx_pData[18] = tx_other.bullet_rate;
    Tx_pData[19] = tx_other.enemy_color;
    Tx_pData[20] = tx_other.vision_mode;
    Tx_pData[21] = tx_other.tail;

    send_time++;
    
    tx_gimbal.time = send_time;
    rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);
    demo_time = tx_gimbal.time - rx_target.time;
    
    Tx_pData[22] = (int32_t)tx_gimbal.time >> 24;
    Tx_pData[23] = (int32_t)tx_gimbal.time >> 16;
    Tx_pData[24] = (int32_t)tx_gimbal.time >> 8;
    Tx_pData[25] = (int32_t)tx_gimbal.time;

    CDC_Transmit_FS(Tx_pData, 26);
}

extern "C" void USB_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    if (Len == 0) return;

    if (Buf[0] == 0x39)
    {
        // 瑙嗚鏁版嵁 (0x39 0x39 澶? 19 瀛楄妭)
        if (Len > sizeof(vision.Rx_pData)) Len = sizeof(vision.Rx_pData);
        memcpy(vision.Rx_pData, Buf, Len);
        vision.dataReceive();
    }
}

void Vision::dataReceive()
{
    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0;
        rx_target.yaw_angle   = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0;
        rx_other.vision_ready = Rx_pData[10];
        
        if (rx_other.vision_ready == false || rx_other.vision_ready == 0)
        {
            vision_flag = false;
        }
        else
        {
            vision_flag = true;
        }
        
        yaw_angle_   = (rx_target.yaw_angle); //+ BSP::Motor::DM: :Motor4310.getAngleDeg(2);
        pitch_angle_ = (rx_target.pitch_angle); //+ BSP::Motor: :DM: :Motor4310.getAngleDeg(1);
        pitch_angle_*= -1.0;    //姣忓彴鏂瑰悜涓嶅悓yaw_angle*= -1.0;
        uint8_t new_fire = Rx_pData[11];

        if (!fire_value_initialized)
        {
            rx_other.fire = new_fire;
            fire_value_initialized = true;
        }
        else if (new_fire != rx_other.fire)
        {
            rx_other.fire = new_fire;
            ++fire_update_count;
        }
        
        rx_other.tail  = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];

        debug_rx_target = rx_target;
        debug_rx_other = rx_other;
    }
}

void UART_TX()
{
    // 鏉块棿閫氳
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_tx_buffer{BoardTx, sizeof(BoardTx)}; 
    uart6.transmit_dma(uart6_tx_buffer);
    
    // Vofa
    // auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    // HAL::UART::Data uart6_tx_buffer{send_str2, sizeof(send_str2)}; 
    // uart6.transmit_dma(uart6_tx_buffer);
}
 

extern "C" {
void Communication(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        UART_TX();
        vision.Data_send();

        osDelay(1);
    }
}

}





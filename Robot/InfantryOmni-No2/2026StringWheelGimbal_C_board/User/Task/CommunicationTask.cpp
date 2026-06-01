#include "CommunicationTask.hpp"
#include "MotorTask.hpp"
#include "usbd_cdc_if.h"

/* 实例板间通讯设备 -----------------------------------------------------------------------------------------*/
BoardCommunication Cboard;
Vision vision;
Navigation navigation;

/* 设备缓存 ------------------------------------------------------------------------------------------------*/
uint8_t send_str2[sizeof(float) * 8];
uint32_t send_time = 0;
uint32_t demo_time = 0;

/* 板间通讯 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 板件通讯初始化 (上下行均已迁移至 CAN2, 由 MotorTask.cpp 处理)
 */
void BoardCommunicationInit()
{
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

/* 视觉通讯 ------------------------------------------------------------------------------------------------*/
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
        // 视觉数据 (0x39 0x39 头, 19 字节)
        if (Len > sizeof(vision.Rx_pData)) Len = sizeof(vision.Rx_pData);
        memcpy(vision.Rx_pData, Buf, Len);
        vision.dataReceive();
    }
    else if(Buf[0] == 0xA5)
    {
        // 导航数据 (15 字节)
        if (Len > sizeof(navigation.Rx_pData)) Len = sizeof(navigation.Rx_pData);
        memcpy(navigation.Rx_pData, Buf, Len);
        navigation.dataReceive();
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
        pitch_angle_*= -1.0;    //每台方向不同yaw_angle*= -1.0;
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


/* 板间通讯发送 ------------------------------------------------------------------------------------------------
 * 已迁移至 CAN2, 由 MotorTask.cpp::send_board_downlink() 处理
 */

/* 导航通讯 ------------------------------------------------------------------------------------------------*/
void Navigation::Data_send()
{
    navigation.frame.head = 0xA5;

    Tx_pData[0] = frame.head;

    tx_odom.x = Cboard.GetChassisX();
    tx_odom.y = Cboard.GetChassisY();
    tx_odom.yaw = Cboard.GetChassisYaw();
    tx_vel.vx = Cboard.GetChassisVx();
    tx_vel.vy = Cboard.GetChassisVy();
    tx_vel.wz = Cboard.GetChassisWz();
    
    memcpy(&Tx_pData[1],  &tx_odom.x,   sizeof(float));
    memcpy(&Tx_pData[5],  &tx_odom.y,   sizeof(float));
    memcpy(&Tx_pData[9],  &tx_odom.yaw, sizeof(float));
    memcpy(&Tx_pData[13], &tx_vel.vx,   sizeof(float));
    memcpy(&Tx_pData[17], &tx_vel.vy,   sizeof(float));
    memcpy(&Tx_pData[21], &tx_vel.wz,   sizeof(float));

    tx_other.time = HAL_GetTick();
    memcpy(&Tx_pData[25], &tx_other.time, sizeof(uint32_t));

    // checksum: 前 29 字节累加和低 16 位
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 29; i++)
        sum += Tx_pData[i];
    tx_other.checksum = static_cast<uint16_t>(sum & 0xFFFF);
    memcpy(&Tx_pData[29], &tx_other.checksum, sizeof(uint16_t));

    CDC_Transmit_FS(Tx_pData, 31);
}

void Navigation::dataReceive()
{
    if (Rx_pData[0] == 0xA5)
    {
        memcpy(&rx_target.vx, &Rx_pData[1],  sizeof(float));
        memcpy(&rx_target.vy, &Rx_pData[5],  sizeof(float));
        memcpy(&rx_target.wz, &Rx_pData[9],  sizeof(float));
        memcpy(&rx_other.checksum, &Rx_pData[13], sizeof(uint16_t));

        // checksum 校验: 前 13 字节累加和低 16 位
        uint32_t sum = 0;
        for (uint8_t i = 0; i < 13; i++)
            sum += Rx_pData[i];
        if (static_cast<uint16_t>(sum & 0xFFFF) != rx_other.checksum)
        {
            // checksum mismatch, discard
        }
    }
}

void UART_TX()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_tx_buffer{send_str2, sizeof(send_str2)}; 
    uart6.transmit_dma(uart6_tx_buffer);
}
 

extern "C" {
void Communication(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        // vofa_send(gimbal_target.target_yaw, gimbal_target.target_pitch, HI12.GetAddYaw(), MotorJ4310.getAngleDeg(1), heat_control.GetBulletCount(), Motor3508.getCurrent(2));
        vision.Data_send();
        navigation.Data_send();
        
        osDelay(1);
    }
}

}





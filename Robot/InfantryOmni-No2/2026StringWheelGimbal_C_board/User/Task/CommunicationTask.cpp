#include "CommunicationTask.hpp"
#include "MotorTask.hpp"
#include "usbd_cdc_if.h"

/* 实例板间通讯设备 -----------------------------------------------------------------------------------------*/
BoardCommunication Cboard;
Vision vision;

/* 设备缓存 ------------------------------------------------------------------------------------------------*/
uint8_t BoardTx[23];
uint8_t BoardRx[4];
uint8_t send_str2[sizeof(float) * 8]; 
uint32_t send_time = 0;
uint32_t demo_time = 0;

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
        if(data.size >= 4 && data.buffer != nullptr)
        {
            Cboard.updateTimestamp();
            Cboard.SetHeatLimit(data.buffer);
            Cboard.SetHeatCool(data.buffer+2);
        }
    });
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

// 假设 q_imu 是从陀螺仪获取的四元数
// theta_p 是从 Pitch 电机编码器转换来的弧度 (需处理零位补偿)

typedef struct 
{
    float w, x, y, z;
} Quaternion;
Quaternion q_imu;

Quaternion get_gimbal_quaternion(Quaternion q_imu, float theta_p) 
{
    Quaternion q_p;
    Quaternion q_res;

    // 1. 构造 Pitch 的局部四元数 (绕 X 轴转动，因为 IMU X轴朝右)
    q_p.w = cosf(theta_p / 2.0f);
    q_p.x = sinf(theta_p / 2.0f);
    q_p.y = 0.0f;
    q_p.z = 0.0f;

    // 2. 四元数乘法: q_res = q_imu * q_p
    q_res.w = q_imu.w * q_p.w - q_imu.x * q_p.x - q_imu.y * q_p.y - q_imu.z * q_p.z;
    q_res.x = q_imu.w * q_p.x + q_imu.x * q_p.w + q_imu.y * q_p.z - q_imu.z * q_p.y;
    q_res.y = q_imu.w * q_p.y - q_imu.x * q_p.z + q_imu.y * q_p.w + q_imu.z * q_p.x;
    q_res.z = q_imu.w * q_p.z + q_imu.x * q_p.y - q_imu.y * q_p.x + q_imu.z * q_p.w;

    return q_res;
}

void Vision::Data_send()
{
    float imu_quaternion[4] = 
    {
        HI12.GetQuaternion(0),
        HI12.GetQuaternion(1),
        HI12.GetQuaternion(2),
        HI12.GetQuaternion(3)
    };
    memcpy(&q_imu, imu_quaternion, sizeof(Quaternion));
    Quaternion q_gimbal = get_gimbal_quaternion(q_imu, -MotorJ4310.getAngleRad(1));
    
    frame.head_one = 0x39;
    frame.head_two = 0x39;
    
    tx_gimbal.quat_w = q_gimbal.w;
    tx_gimbal.quat_x = q_gimbal.x;
    tx_gimbal.quat_y = q_gimbal.y;
    tx_gimbal.quat_z = q_gimbal.z;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52;
    
    tx_other.tail = 0xFF;

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    std::memcpy(&Tx_pData[2],  &tx_gimbal.quat_w, sizeof(float));
    std::memcpy(&Tx_pData[6],  &tx_gimbal.quat_x, sizeof(float));
    std::memcpy(&Tx_pData[10], &tx_gimbal.quat_y, sizeof(float));
    std::memcpy(&Tx_pData[14], &tx_gimbal.quat_z, sizeof(float));

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
    if(Len > sizeof(vision.Rx_pData)) Len = sizeof(vision.Rx_pData);
    memcpy(vision.Rx_pData, Buf, Len);
    vision.dataReceive();
}

void Vision::dataReceive()
{
    uint32_t rx_len = 19;

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


/* 通讯发送 ------------------------------------------------------------------------------------------------*/
void BoardCommunicationTX()
{
    float angle = Motor6020.getAngleRad(1);
    bool scroll = (gimbal_fsm.Get_Now_State() == MANUAL) && (launch_fsm.Get_Now_State() == LAUNCH_AUTO || launch_fsm.Get_Now_State() == LAUNCH_ONLY || launch_fsm.Get_Now_State() == LAUNCH_JAM);
    memcpy(BoardTx, DT7Rx_buffer, sizeof(DT7Rx_buffer));
    memcpy(BoardTx+18, &angle, sizeof(float));
    memcpy(BoardTx+22, &scroll, sizeof(bool));

    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_tx_buffer{BoardTx, sizeof(BoardTx)}; 
    uart6.transmit_dma(uart6_tx_buffer);

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
        // vofa_send(HI12.GetAddYaw(), gimbal_target.target_yaw, MotorJ4310.getAddAngleDeg(1), gimbal_target.target_pitch, HI12.GetGyroRPM(2), VisionPitchTarget);
        // vofa_send(gimbal_target.target_yaw, gimbal_target.target_pitch, HI12.GetAddYaw(), MotorJ4310.getAngleDeg(1), heat_control.GetBulletCount(), Motor3508.getCurrent(2));
        BoardCommunicationTX();
        vision.Data_send();
        
        osDelay(1);
    }
}

}





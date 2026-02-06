#include "CommunicationTask.hpp"
#include "../User/Task/MotorTask.hpp"
#include "../USB_DEVICE/App/usbd_cdc_if.h"

/* 实例板间通讯设备 -----------------------------------------------------------------------------------------*/
BoardCommunication Cboard;
Vision vision;

/* 设备缓存 ------------------------------------------------------------------------------------------------*/
uint8_t BoardTx[23];
uint8_t BoardRx[4];
uint8_t send_str2[sizeof(float) * 8]; 


uint32_t send_time;
// delay PID测试
float time_kp = 2.0, time_out, int_time = 5;
uint32_t demo_time; // 测试时间戳

/* 板间通讯 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 板件通讯串口接收回调与数据解析
 * 
 */
void BoardCommunicationInit()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_rx_buffer{BoardTx, sizeof(BoardTx)};
    uart6.receive_dma_idle(uart6_rx_buffer);
    uart6.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size >= 18 && data.buffer != nullptr)
        {
            Cboard.updateTimestamp();

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
void Vision::time_demo()
{
    if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = (uint32_t)(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

void Vision::Data_send()
{
    frame.head_one = 0x39;
    frame.head_two = 0x39;

    tx_gimbal.yaw_angle = HI12.GetAddYaw() * 100.0f;
    tx_gimbal.pitch_angle = MotorJ4310.getAngleDeg(1) * 100.0f;

    tx_other.bullet_rate = 26;
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝

    tx_other.tail = 0xFF; // 准备标志位

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    Tx_pData[2] = (int32_t)tx_gimbal.pitch_angle >> 24;
    Tx_pData[3] = (int32_t)tx_gimbal.pitch_angle >> 16;
    Tx_pData[4] = (int32_t)tx_gimbal.pitch_angle >> 8;
    Tx_pData[5] = (int32_t)tx_gimbal.pitch_angle;

    Tx_pData[6] = (int32_t)tx_gimbal.yaw_angle >> 24;
    Tx_pData[7] = (int32_t)tx_gimbal.yaw_angle >> 16;
    Tx_pData[8] = (int32_t)tx_gimbal.yaw_angle >> 8;
    Tx_pData[9] = (int32_t)tx_gimbal.yaw_angle;

    Tx_pData[10] = 26;
    Tx_pData[11] = 0x52; // 0x42红   0X52蓝色
    Tx_pData[12] = tx_other.vision_mode;
    Tx_pData[13] = tx_other.tail;

    send_time++;
    tx_gimbal.time = send_time;

    rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);
    
    demo_time = tx_gimbal.time - rx_target.time;

    Tx_pData[14] = (int32_t)tx_gimbal.time >> 24;
    Tx_pData[15] = (int32_t)tx_gimbal.time >> 16;
    Tx_pData[16] = (int32_t)tx_gimbal.time >> 8;
    Tx_pData[17] = (int32_t)tx_gimbal.time;

    CDC_Transmit_FS(Tx_pData, 18);
}

extern "C" void USB_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    if(Len > sizeof(vision.Rx_pData)) Len = sizeof(vision.Rx_pData);
    memcpy(vision.Rx_pData, Buf, Len);
    vision.dataReceive();
}

void Vision::dataReceive()
{

    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0f;
        rx_target.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0f;

        if ((fabs(rx_target.yaw_angle) > 25 && fabs(rx_target.pitch_angle) > 25) || rx_other.vision_ready == false)
        {
            vision_flag = false;
            rx_target.yaw_angle = 0;
            rx_target.pitch_angle = 0;
        }
        else
        {
            vision_flag = true;
        }

		//如果视觉时间戳差值大于100ms，判断为断连
		if(vision_flag == true && send_time - rx_target.time > 100)
		{
			vision_flag = false;
		}
		
        //		if((rx_other.vision_ready == false))
        //			vision_flag = false;
        //		else
        //			vision_flag = true;

        yaw_angle_ = rx_target.yaw_angle + HI12.GetAddYaw();
        pitch_angle_ = MotorJ4310.getAngleDeg(1) - rx_target.pitch_angle; 
        // pitch_angle_ *= -1.0; // 每台方向不同

        rx_other.vision_ready = Rx_pData[10];
        rx_other.fire = (Rx_pData[11]);
        rx_other.tail = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];
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
    //BoardCommunicationInit();
    for(;;)
    {
        // vofa_send(HI12.GetAddYaw(), gimbal_target.target_yaw, MotorJ4310.getAddAngleDeg(1), gimbal_target.target_pitch, HI12.GetGyroRPM(2), VisionPitchTarget);
        //vofa_send(heat_control.GetMaxHeat(), heat_control.GetNowHeat(), Motor3508.getCurrent(2), Motor3508.getCurrent(3), Motor3508.getVelocityRpm(3), 0);
        BoardCommunicationTX();
        vision.Data_send();
        
        osDelay(5);
    }
}

}





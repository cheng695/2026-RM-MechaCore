#include "CommunicationTask.hpp"
#include "../User/Task/MotorTask.hpp"
#include "../USB_DEVICE/App/usbd_cdc_if.h"

/* 瀹炰緥鏉块棿閫氳璁惧 -----------------------------------------------------------------------------------------*/
BoardCommunication Cboard;
Vision vision;
LaunchFSMInput launch_fsm_input;

namespace
{
    constexpr size_t kBoardTxDt7Offset = 0;
    constexpr size_t kBoardTxAngleOffset = 18;
    constexpr size_t kBoardTxScrollOffset = 22;
    constexpr size_t kBoardTxLaunchInputOffset = 23;
}

extern bool check_online();
extern bool is_jamming();

/* 璁惧缂撳瓨 ------------------------------------------------------------------------------------------------*/
uint8_t BoardTx[64];
uint8_t BoardRx[4];
uint8_t send_str2[sizeof(float) * 8]; 


uint32_t send_time;
// delay PID娴嬭瘯
float time_kp = 2.0, time_out, int_time = 5;
uint32_t demo_time; // 娴嬭瘯鏃堕棿鎴?

/* 鏉块棿閫氳 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 鏉夸欢閫氳涓插彛鎺ユ敹鍥炶皟涓庢暟鎹В鏋?
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
            Cboard.SetHeatLimit(data.buffer);
            Cboard.SetHeatCool(data.buffer+2);
        }
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
    tx_other.enemy_color = 0x52; // 0x42鎴戠孩   0X52鎴戣摑

    tx_other.tail = 0xFF; // 鍑嗗鏍囧織浣?

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
    Tx_pData[11] = 0x52; // 0x42绾?  0X52钃濊壊
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

		//濡傛灉瑙嗚鏃堕棿鎴冲樊鍊煎ぇ浜?00ms锛屽垽鏂负鏂繛
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
        // pitch_angle_ *= -1.0; // 姣忓彴鏂瑰悜涓嶅悓

        rx_other.vision_ready = Rx_pData[10];
        rx_other.fire = (Rx_pData[11]);
        rx_other.tail = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];
    }
}

/* 閫氳鍙戦€?------------------------------------------------------------------------------------------------*/
void BoardCommunicationTX()
{
    float angle = MotorJ4310.getAngleRad(2);
    uint8_t scroll = ((gimbal_fsm.Get_Now_State() == MANUAL) &&
                      (launch_fsm.Get_Now_State() == LAUNCH_AUTO ||
                       launch_fsm.Get_Now_State() == LAUNCH_ONLY ||
                       launch_fsm.Get_Now_State() == LAUNCH_JAM)) ? 1U : 0U;

    launch_fsm_input.equipment_online = check_online() ? 1U : 0U;
    launch_fsm_input.change = is_change ? 1U : 0U;
    launch_fsm_input.only_to_auto_time = 4000.0f;
    launch_fsm_input.is_vision = is_vision ? 1U : 0U;
    launch_fsm_input.is_shoot = heat_control.GetShot() ? 1U : 0U;
    launch_fsm_input.is_jamming = is_jamming() ? 1U : 0U;
    launch_fsm_input.launch_state = static_cast<int32_t>(launch_fsm.Get_Now_State());
    for (size_t i = 0; i < 28; ++i)
    {
        launch_fsm_input.alphabet[i] = alphabet[i] ? 1U : 0U;
    }

    memset(BoardTx, 0, sizeof(BoardTx));
    memcpy(BoardTx + kBoardTxDt7Offset, DT7Rx_buffer, sizeof(DT7Rx_buffer));
    memcpy(BoardTx + kBoardTxAngleOffset, &angle, sizeof(angle));
    memcpy(BoardTx + kBoardTxScrollOffset, &scroll, sizeof(scroll));

    size_t offset = kBoardTxLaunchInputOffset;
    memcpy(BoardTx + offset, &launch_fsm_input.equipment_online, sizeof(launch_fsm_input.equipment_online));
    offset += sizeof(launch_fsm_input.equipment_online);
    memcpy(BoardTx + offset, &launch_fsm_input.change, sizeof(launch_fsm_input.change));
    offset += sizeof(launch_fsm_input.change);
    memcpy(BoardTx + offset, &launch_fsm_input.only_to_auto_time, sizeof(launch_fsm_input.only_to_auto_time));
    offset += sizeof(launch_fsm_input.only_to_auto_time);
    memcpy(BoardTx + offset, &launch_fsm_input.is_vision, sizeof(launch_fsm_input.is_vision));
    offset += sizeof(launch_fsm_input.is_vision);
    memcpy(BoardTx + offset, &launch_fsm_input.is_shoot, sizeof(launch_fsm_input.is_shoot));
    offset += sizeof(launch_fsm_input.is_shoot);
    memcpy(BoardTx + offset, &launch_fsm_input.is_jamming, sizeof(launch_fsm_input.is_jamming));
    offset += sizeof(launch_fsm_input.is_jamming);
    memcpy(BoardTx + offset, launch_fsm_input.alphabet, sizeof(launch_fsm_input.alphabet));
    offset += sizeof(launch_fsm_input.alphabet);
    memcpy(BoardTx + offset, &launch_fsm_input.launch_state, sizeof(launch_fsm_input.launch_state));

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
        //vofa_send(HI12.GetGyroRad(0), gimbal_target.target_pitch2_vel, HI12.GetGyroRad(2), gimbal_target.target_yaw_vel, gimbal_yaw.getTorque(), yaw_adrc_try.GetZ1());
        BoardCommunicationTX();
        //vision.Data_send();
        
        osDelay(5);
    }
}

}







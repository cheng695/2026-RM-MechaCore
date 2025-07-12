/**
 * @file call_back.cpp
 * @author XMX
 * @brief 中断回调函数文件
 * @version 1.0
 * @date 2024-08-07
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "call_back.hpp"
#include "RM_RefereeSystem.h"
#include "capacity.hpp"
#include "chassis.hpp"
#include "clamp.hpp"
#include "error_handle.hpp"
#include "power_limit.hpp"
#include "variables.hpp"
constexpr auto vofa_cnt = 2;
double pm_voltage;
double pm_current;
float pm_power;
float target_speed = 0;
uint8_t send_str2[64];
float lowPassFilter(float input, float previousOutput, float alpha)
{
    return alpha * input + (1.0f - alpha) * previousOutput;
}
float lowPassFilteredValue = 0.0f; // 低通滤波后的电机速度值
const float alpha = 0.01f;         // 低通滤波系数，取值范围0.0 - 1.0，值越小滤波效果越强
// void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
//{
//     const uint8_t sendSize = 4;

//     	*((float *)&send_str2[0]) = x1;
//      *((float *)&send_str2[4]) =x2;
//      *((float *)&send_str2[8]) =x3;
//      *((float *)&send_str2[12]) =x4;
//            *((uint32_t *)&send_str2[sizeof(float) * 7]) = 0x7f800000;
//    	  HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * 8);
//}
/// @brief CAN FIFO0 接收中断函数
/// @param hcan 触发中断的can实例
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == kMotorCan->Instance)
    {

        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};

        HAL_CAN_GetRxMessage(kMotorCan, CAN_RX_FIFO0, &rx_header, buf); // 获取消息

        for (uint8_t i = 0; i < kMotorCount; i++)
        {
            if (rx_header.StdId == dji_motor_list[i]->recv_id_)
            {
                dji_motor_list[i]->DataUpdate(buf);  // 更新对应ID的电机数据
                dji_motor_list[i]->is_reply_ = true; // 置位应答位
                break;
            }
        }
        // if (rx_header.StdId == 0x233) {
        // capacity.Receive(rx_header, buf);
        //}
        if (rx_header.StdId == 0x212)
        {

            pm_voltage = (double)((int32_t)(buf[1] << 8) | (int32_t)(buf[0])) / 100.0;
            pm_current = (double)((int32_t)(buf[3] << 8) | (int32_t)(buf[2])) / 100.0;
            pm_power = pm_voltage * pm_current;
        }
    }
    else if (hcan->Instance == kCapacityCan->Instance)
    {

        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};
        HAL_CAN_GetRxMessage(kCapacityCan, CAN_RX_FIFO0, &rx_header, buf); // 获取消息

        //  ============================= 安合电容 =================================
        if (rx_header.StdId == 0x611)
        {
            short tempqweqwe = buf[0] << 8 | buf[1];
            capacity.target_power = tempqweqwe / 100.0f;
        }
        else if (rx_header.StdId == 0x612)
        {
            capacity.voltage = ((float)(buf[2] << 8 | buf[3]) / 100.0f);
        }
        //  =======================================================================

        //  ============================= 自研电容 =================================
        //		if(rx_header.StdId == 0x100)
        //		{
        //			capacity.Send(kCapacityCan,referee.robot_status.robot_id,referee.robot_status.chassis_power_limit);
        //		}
        //  =======================================================================

        capacity.is_reply_ = true; // 置位应答位
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == kCommUart->Instance)
    {
        if (Size == kCommRecvSize)
        {
            comm.RecvUpdate(comm_rx_buf); // 更新板间通信数据
            comm.is_reply_ = true;        // 置位应答位
        }
        // 重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    }
    else if (huart->Instance == kRefereeUart->Instance)
    {
        referee.recv_size = Size;                // 获取数据长度
        referee.FrameProcessing(referee_rx_buf); // 裁判系统数据更新
        referee.is_reply = true;                 // 置位应答位
        //        //重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(kRefereeUart, referee_rx_buf, kRefereeRecvSize);
        // RM_RefereeSystem::RM_RefereeSystemParse(huart);
    }
}

void MainCallBack()
{
    /*  =========================== PID运算 ===========================  */
    // lowPassFilteredValue = lowPassFilter(PowerControl.GetEstWheelPow() , lowPassFilteredValue , alpha);
    //    vofaSend(capacity.Out_Power,PowerControl.getMAXPower(),capacity.In_Power,PowerControl.Wheel_PowerData.Cur_EstimatedPower,0,0);
    // 201转速PID

    auto measure_rpm_201 = td_201.Compute(motor_201.actual_rpm_); // 电机速度数据滤波
    auto dji_motor_201_input = pid_vel_201.Compute(target_rpm_201, measure_rpm_201);
    motor_201.input_ = dji_motor_201_input; // 速度PID运算
    // motor_201.input_=power_get[0];
    // 202转速PID
    auto measure_rpm_202 = td_202.Compute(motor_202.actual_rpm_); // 电机速度数据滤波
    auto dji_motor_202_input = pid_vel_202.Compute(target_rpm_202, measure_rpm_202);
    motor_202.input_ = dji_motor_202_input; // 速度PID运算
                                            // motor_202.input_ =power_get[1];
    // 203转速PID
    auto measure_rpm_203 = td_203.Compute(motor_203.actual_rpm_); // 电机速度数据滤波
    auto dji_motor_203_input = pid_vel_203.Compute(target_rpm_203, measure_rpm_203);
    motor_203.input_ = dji_motor_203_input; // 速度PID运算
    // motor_203.input_=power_get[2];
    // 204转速PID
    auto measure_rpm_204 = td_204.Compute(motor_204.actual_rpm_); // 电机速度数据滤波
    auto dji_motor_204_input = pid_vel_204.Compute(target_rpm_204, measure_rpm_204);
    motor_204.input_ = dji_motor_204_input; // 速度PID运1算
    // motor_204.input_=power_get[3];
    //   ======================= 测试自研电容组 add in 2024/7/29 =======================
    //  PowerCare();  //功率限值
    //  	if(referee.power_heat_data.buffer_energy < testvalue){
    //          testvalue = referee.power_heat_data.buffer_energy;
    //      }
    //   ==============================================================================
    //

    DjiMotorSend(); // 电机数据发送
    //		    *((float*)&send_str2[0]) = PowerControl.Wheel_PowerData.Cur_EstimatedPower;
    //	*((float*)&send_str2[4]) = PowerControl.Wheel_PowerData.EstimatedPower;
    //	*((float*)&send_str2[8]) = capacity.target_power;
    //	*((float*)&send_str2[12]) = PowerControl.Wheel_PowerData.Cmd_MaxT[1];
    //	*((float*)&send_str2[16]) =  power_get[1];
    //	*((float*)&send_str2[20]) = pid_vel_201.output_;
    ////	*((float*)&send_str2[24]) =  chGy_chassis.gy.ASz;
    //	*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
    //	  HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * (7 + 1));
}

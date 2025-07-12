/**
 * @file capacity.cpp
 * @author XMX
 * @brief 安合超级电容类方法
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "capacity.hpp"
#include "can.hpp"
#include "uart.hpp"
#include "call_back.hpp"
#include "referee_detect_task.hpp"
#include "variables.hpp"
#include "can.h"
/// @brief 使能输出
void Capacity::EnableOutput() {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x600;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_buf[1] = 2;

    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX0);
}

/// @brief 设置最大充电功率
/// @param more_power 
void Capacity::SetMaxChargePower(uint16_t more_power) {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x601;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

	more_power *= 100;
    tx_buf[0] = (more_power) >> 8;
    tx_buf[1] = (more_power);
	
    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX1);
}

/// @brief 查询电容组电压
void Capacity::AskVoltage() {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x612;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_REMOTE;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;


    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX0);
}

/// @brief 查询输入功率
void Capacity::AskInputPower() {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x611;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_REMOTE;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
	

	
    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX0);
}

/// @brief 自研电容组
void Capacity::Ask_RCIA() {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x715;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX0);
}

/// @brief 自研电容组
void Capacity::SetMaxChargePower_RCIA() {
    CAN_TxHeaderTypeDef tx_header{0};
    uint8_t tx_buf[8]{0};

    tx_header.StdId = 0x615;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_buf[0] = charge_power >> 8;
    tx_buf[1] = charge_power;

    HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, tx_buf, (uint32_t*)CAN_TX_MAILBOX0);
}
void Capacity::SetDischargePower(uint16_t power) {
    uint8_t data[8] = {0xAA, 0x55, 0x04, 0x00, 
                      static_cast<uint8_t>(power >> 8), 
                      static_cast<uint8_t>(power & 0xFF), 
                      0x00, 0x00};
    HAL_UART_Transmit(&huart6, data, 8, 10);
}
CAN_RxHeaderTypeDef rx_header{0};
        
//void Capacity::Receive(CAN_RxHeaderTypeDef& rx_header,uint8_t* buf){

//	if (rx_header.StdId == 0x233) {
//		In_Power = ((buf[0] << 8) | buf[1]) / 100.0f;    //底盘功率
//        voltage = ((buf[2] << 8) | buf[3]) / 100.0f;     //超电电压
//        Out_Power = ((buf[4] << 8) | buf[5]) / 100.0f;   //底盘+超电   
//        State = buf[6];
//        Cmd = buf[7];
//	}
//}


//void Capacity::Send(CAN_HandleTypeDef* hcan, uint8_t robot_id, uint16_t power_limit)	
//{    
//	CAN_TxHeaderTypeDef tx_header;
//	uint8_t capacity_tx_buf[4]{0};
//	tx_header.StdId = 0x100;
//	tx_header.IDE   = CAN_ID_STD;
//	tx_header.RTR   = CAN_RTR_DATA;
//	tx_header.DLC   = 8;
//	 
//	capacity_tx_buf[0] = (robot_id >> 8) & 0xFF;        // 机器人ID          
//    capacity_tx_buf[1] = robot_id & 0xFF;  				
//    capacity_tx_buf[2] = (power_limit >> 8) & 0xFF;		// 功率高8位
//	capacity_tx_buf[3] = power_limit & 0xFF;           // 功率低8位
//	capacity_tx_buf[4] = 0;
//	capacity_tx_buf[5] = 0;
//	capacity_tx_buf[6] = 0;
//	capacity_tx_buf[7] = 0;
//	
//	HAL_CAN_AddTxMessage(kCapacityCan, &tx_header, capacity_tx_buf,(uint32_t*)CAN_TX_MAILBOX0);
//	
//}

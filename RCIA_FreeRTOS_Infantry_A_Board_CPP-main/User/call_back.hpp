#pragma once
#include "can.hpp"
#include "uart.hpp"

extern uint16_t first_flag;
void CanCallBack(CAN_HandleTypeDef* _hcan);
void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);
extern uint8_t gete;
#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void MainCallBack();
	
//void dm4310_fbdata(motor_t *motor, uint8_t *rx_data);

#ifdef __cplusplus
}
#endif  // __cplusplus

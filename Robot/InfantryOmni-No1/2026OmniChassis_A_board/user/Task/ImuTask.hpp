#ifndef RemoteTask_h
#define RemoteTask_h 

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/IMU/HI12_imu.hpp"

extern BSP::IMU::HI12_float HI12;
extern uint8_t HI12RX_buffer[82];


#endif

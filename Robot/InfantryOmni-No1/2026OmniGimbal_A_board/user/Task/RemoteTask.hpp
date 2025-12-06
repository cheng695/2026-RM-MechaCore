#ifndef RemoteTask_h
#define RemoteTask_h 

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/IMU/HI12_imu.hpp"

extern uint8_t IMUData[82];
extern BSP::IMU::HI12_float HI12;

#endif

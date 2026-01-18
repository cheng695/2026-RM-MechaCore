#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
// #include "../user/core/BSP/RemoteControl/DT7.hpp"
// #include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/Task/ControlTask.hpp"

extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern uint8_t DT7Rx_buffer[18];
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern float YawTarget;

#endif

#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern uint8_t CommunicationData[18];

#endif

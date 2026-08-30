#ifndef SERIALTASK_H
#define SERIALTASK_H

#include "FreeRTOS.h"
#include "HI12_imu.hpp"
#include "OperatorInput.hpp"
#include "SimpleKey.hpp"
#include "cmsis_os.h"
#include "uart_hal.hpp"

#include <cstring>

extern BSP::IMU::HI12_float HI12;
extern uint8_t HI12RX_buffer[82];
extern uint8_t DT7Rx_buffer[18];
extern uint8_t power_buffer[12];
extern uint8_t referee_buffer[512];

OperatorInput BuildOperatorInput();

/*
 * 外部功率计数据对象。
 */
class Power
{
public:
    void DataUpdate(uint8_t* buffer)
    {
        memcpy(&V, buffer, sizeof(float));
        memcpy(&I, buffer + 4, sizeof(float));
        memcpy(&power, buffer + 8, sizeof(float));
    }

    float Getpower()
    {
        return power;
    }

private:
    float V = 0.0f;
    float I = 0.0f;
    float power = 0.0f;
};

extern Power power;

extern "C" {
void Serival(void const* argument);
}

#endif

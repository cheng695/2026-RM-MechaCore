#include "chassis.hpp"

void Chassis_Monitor(void const * argument)
{
    while(1)
    {
        MOTOR_MONITOR.MotorMonitor();
        osDelay(10);
    }
}

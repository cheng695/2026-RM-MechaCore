#include "chassis.hpp"

#ifdef __cplusplus
extern "C" {
#endif
	
void Chassis_Monitor(void const * argument)
{
    while(1)
    {
        MOTOR_MONITOR.MotorMonitor();
        REMOTE_MONITOR.RemotecontrolMonitor();
        osDelay(10);
    }
}

void Chassis_Ctrl(void const * argument)
{
    while(1)
    {
        wheel_control.wheelControl();
        remote_control.remoteControl();
        osDelay(1);
    }
}

void Capacitor(void const * argument)
{
    while(1)
    {
        osDelay(1);
    }
}


void Chassis_Comm(void const * argument)
{
    while(1)
    {
        osDelay(1);
    }
}

#ifdef __cplusplus
}
#endif
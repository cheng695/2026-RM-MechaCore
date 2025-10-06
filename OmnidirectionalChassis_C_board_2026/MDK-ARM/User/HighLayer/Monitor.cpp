#include "Monitor.hpp"

Monitor::motor_monitor MOTOR_MONITOR;
Monitor::remote_monitor REMOTE_MONITOR;

void Monitor::motor_monitor::MotorMonitor()
{
    MotorState();
}

void Monitor::remote_monitor::RemotecontrolMonitor()
{
    RemoteState();
}

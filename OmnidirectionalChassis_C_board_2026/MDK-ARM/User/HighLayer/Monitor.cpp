#include "Monitor.hpp"

Monitor::motor_monitor MOTOR_MONITOR;
Monitor::remote_monitor REMOTE_MONITOR;
Monitor::board_monitor BOARD_MONITOR;

void Monitor::motor_monitor::MotorMonitor()
{
    MotorState();
}

void Monitor::remote_monitor::RemotecontrolMonitor()
{
    RemoteState();
}

void Monitor::board_monitor::BoardMonitor()
{
    void CreceiveState();
}

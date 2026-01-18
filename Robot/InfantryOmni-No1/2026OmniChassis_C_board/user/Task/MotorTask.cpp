#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::Dji::GM6020<1> Motor6020(0x204, {2}, 0x1FF);

void MotorInit(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
    });
}

static void motor_control_logic()
{
    // for (int i = 0; i <= 4; i++) 
    // {
    //     if (Motor3508.isConnected(i+1)) 
    //     {
    //         Motor3508.setCAN(static_cast<int16_t>(wheel_pid[i].getOutput()), i+1);
    //     } 
    //     else 
    //     {
    //         Motor3508.setCAN(0, i+1);
    //     }
    // }
    // Motor3508.sendCAN();

    Motor6020.setCAN(static_cast<int16_t>(YawVelocity_pid.getOutput()), 2);
    Motor6020.sendCAN();

}

extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    for(;;)
    {
        motor_control_logic();
        osDelay(1);
    } 
}

}
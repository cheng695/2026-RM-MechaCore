#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::Dji::GM6020<4> Motor6020(0x204, {1, 2, 3, 4}, 0x1FE);

void MotorInit(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        
    });
}

static void motor_control_logic()
{
    for(int i = 0; i < 4; i++)
    {
        Motor6020.setCAN(static_cast<int16_t>(chassis_output.out_string[i]), i + 1);
        Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
    }
    Motor6020.sendCAN();
    Motor3508.sendCAN();
}                               


extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    for(;;)
    {
        motor_control_logic();
        osDelay(2);
    } 
}

}
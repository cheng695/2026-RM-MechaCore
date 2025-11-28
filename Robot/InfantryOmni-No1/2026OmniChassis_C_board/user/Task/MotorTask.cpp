#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);

void Init(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
    });
}

extern "C"{
void Control(void const * argument)
{
    Init();
    for(;;)
    {
        osDelay(1);
    } 
}

}
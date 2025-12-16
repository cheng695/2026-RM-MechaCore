#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 4}, 0x200);
BSP::Motor::Dji::GM6020<1> Motor6020(0x204, {2}, 0x1FF);
BSP::Motor::Dji::GM2006<1> Motor2006(0x200, {5}, 0x200);
BSP::Motor::DM::J4310<1> MotorJ4310(0x00, {2}, {0x01});
BSP::Motor::LK::LK4005<4> Motor4005(0, {1, 2, 3, 4}, {1, 2, 3, 4});
void MotorInit(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
        Motor2006.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        MotorJ4310.Parse(frame);
    });
}

static void motor_control_logic()
{
    Motor6020.setCAN(static_cast<int16_t>(gimbal_output.out_yaw), 2);
    Motor6020.sendCAN();
    
    MotorJ4310.ctrl_Mit(0x01, 0.0f, 0.0f, 0.0f, 0.0f, gimbal_output.out_pitch);

    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[0]), 1);
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[1]), 4);
    Motor3508.sendCAN();

    Motor2006.setCAN(static_cast<int16_t>(launch_output.out_dial), 5);
    Motor2006.sendCAN();
}                               


extern "C"{
void motor(void const * argument)
{
    MotorInit();
    for(;;)
    {
        motor_control_logic();
        osDelay(1);
    } 
}

}
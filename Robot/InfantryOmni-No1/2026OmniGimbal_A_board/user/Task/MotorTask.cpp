#include "MotorTask.hpp"

BSP::Motor::Dji::GM6020<1> Motor6020(0x204, {2}, 0x1FF);
BSP::Motor::Dji::GM3508<2> Motor3508(0x200, {1,4}, 0x200);
BSP::Motor::Dji::GM2006<1> Motor2006(0x200, {3}, 0x200);
BSP::Motor::DM::J4310<1> MotorJ4310(0x00, {2}, 0x01);

void MotorInit(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    //static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor6020.Parse(frame);
        // Motor2006.Parse(frame);
        // Motor3508.Parse(frame);
    });
    // can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    // {
    //     MotorJ4310.Parse(frame);
    // });
}

static void motor_control_logic()
{
    for (int i = 0; i <= 4; i++) 
    {
        // if (Motor3508.isConnected(i+1)) 
        // {
            // Motor3508.setCAN(static_cast<int16_t>(surgewheel_pid[i].getOutput()), i+1);
        // } 
        // else 
        // {
        //     Motor3508.setCAN(0, i+1);
        // }
    }
    // Motor3508.sendCAN();
    // if(Motor6020.isConnected(2))
    // {
    //     Motor6020.setCAN(static_cast<int16_t>(yaw_pid.getOutput()), 2);

    // // }
    // Motor6020.sendCAN();


}
int a = 0;
extern "C"{
void Motor(void const * argument)
{
    
    MotorInit();
    for(;;)
    {
        Motor6020.setCAN(static_cast<int16_t>(yaw_pid.getOutput()), 2);
        Motor6020.sendCAN();
//        if(a == 1)
//        {
//			MotorJ4310.on(2, 0);
//        }
//		else if(a == 0)
//		{
//			MotorJ4310.off(2, 0);
//		}
    auto& log = HAL::LOGGER::Logger::getInstance();
    log.trace("%d", HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
        
        motor_control_logic();
        osDelay(5);
    } 
}

}
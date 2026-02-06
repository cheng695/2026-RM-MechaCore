#include "MotorTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::Dji::GM6020<4> Motor6020(0x204, {1, 2, 3, 4}, 0x1FE);

CAN接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN初始化函数
 * 
 * 初始化CAN总线并注册电机反馈数据解析回调函数
 */
void MotorInit(void)
{
    // 实例CAN
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    // 注册CAN接收回调函数
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        supercap.Parse(frame);
    });
}


static void motor_control_logic(uint32_t tick)
{
    // 200Hz for Motors (Every 5ms)
    if (tick % 5 == 0)
    {
        for(int i = 0; i < 4; i++)
        {
            Motor6020.setCAN(static_cast<int16_t>(chassis_output.out_string[i]), i + 1);
            Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
        }
        Motor6020.sendCAN();
        Motor3508.sendCAN();
    }

    // 500Hz for Supercap (Every 2ms)
    if (tick % 2 == 0)
    {
        supercap.sendCAN();
    }
}                               

extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    static uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        osDelay(1);
    } 
}

}


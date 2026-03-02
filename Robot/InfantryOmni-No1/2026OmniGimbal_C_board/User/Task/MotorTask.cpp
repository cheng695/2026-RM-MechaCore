#include "MotorTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1, 2, 3}, 0x200); // 1号ID是2006，避免ID冲突合并到3508中
BSP::Motor::Dji::GM6020<1> Motor6020(0x204, {1}, 0x1FE);
BSP::Motor::DM::J4310<1> MotorJ4310(0x00, {2}, {0x01});
BSP::Motor::DM::J4340<1> MotorJ4340(0x00, {4}, {0x03});
BSP::Motor::LK::LK4005<1> MotorLK4005(0x140, {1}, {0x01});

/* CAN接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN初始化函数
 * 
 * 初始化CAN总线并注册电机反馈数据解析回调函数
 */
void MotorInit(void)
{
    // 实例CAN
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    // 注册CAN接收回调函数
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        MotorJ4310.Parse(frame);
        MotorJ4340.Parse(frame);
        MotorLK4005.Parse(frame);
    });
}

/* 电机发送控制帧 ----------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制函数
 * 
 * 根据控制系统的输出结果向各电机发送控制指令
 */
static void motor_control()
{
    static uint8_t send_seq = 0;
    static uint32_t last_recovery_4310 = 0;
    static uint32_t last_recovery_4340 = 0;
    static uint32_t last_recovery_lk4005 = 0;
    send_seq++;

    static uint32_t last_enable_ensure_4310 = 0;
    static uint32_t last_enable_ensure_4340 = 0;
    static uint32_t last_enable_ensure_lk4005 = 0;

   
    if(MotorLK4005.getIsenable())
    {
        if(HAL_GetTick() - last_recovery_lk4005 > 2000)
        {
            MotorLK4005.On(1, 1);
            last_enable_ensure_lk4005 = HAL_GetTick();
        }
                
        MotorLK4005.ctrl_Torque(1, 1, 0.0f);
    }
    else
    {
        MotorLK4005.SetReply(1, 1);
    }

    // if (send_seq % 2 == 0) 
    // {
    //     if(MotorJ4310.getIsenable())
    //     {
    //             if(HAL_GetTick() - last_enable_ensure_4310 > 2000)
    //             {
    //                MotorJ4310.On(1, BSP::Motor::DM::MIT);
    //                last_enable_ensure_4310 = HAL_GetTick();
    //             }
                
    //             MotorJ4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, gimbal_output.out_pitch2);
    //     }
    //     else
    //     {
    //         MotorJ4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //     }
    // }
    // else 
    // {
    //     if(MotorJ4340.getIsenable())
    //     {
    //             if(HAL_GetTick() - last_enable_ensure_4340 > 2000)
    //             {
    //                MotorJ4340.On(1, BSP::Motor::DM::MIT);
    //                last_enable_ensure_4340 = HAL_GetTick();
    //             }

    //             MotorJ4340.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, gimbal_output.out_pitch1);
    //     }
    //     else
    //     {
    //         MotorJ4340.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //     }
    // }
}                               

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制任务函数
 * 
 * 任务主循环，负责初始化电机并持续执行电机控制
 * 
 * @param argument 任务参数指针
 */
extern "C"{
void Motor(void const * argument)
{
    MotorInit();    // 初始化电机和CAN总线
    for(;;)
    {
        motor_control();  // 执行电机控制
        osDelay(1);       // 延时1ms，控制频率1000Hz
    } 
}

}
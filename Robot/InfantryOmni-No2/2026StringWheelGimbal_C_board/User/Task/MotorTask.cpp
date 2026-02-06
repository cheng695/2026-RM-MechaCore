#include "MotorTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1, 2, 3}, 0x200); // 1号ID是2006，避免ID冲突合并到3508中
BSP::Motor::Dji::GM6020<1> Motor6020(0x204, {1}, 0x1FE);
BSP::Motor::DM::J4310<1> MotorJ4310(0x00, {2}, {0x01});

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
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    // 注册CAN接收回调函数
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        MotorJ4310.Parse(frame);
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
    // 云台
    Motor6020.setCAN(static_cast<int16_t>(gimbal_output.out_yaw), 1);
    Motor6020.sendCAN();

    if(gimbal_fsm.Get_Now_State() == MANUAL)
    {
        MotorJ4310.ctrl_Mit(0x01, 
                           gimbal_target.target_pitch*3.1415926f/180.0f, 
                           gimbal_target.target_pitch_vel, 
                           pitch_manual_pid.getK(0), 
                           pitch_manual_pid.getK(2), 
                           gimbal_output.out_pitch);
    }
    else if(gimbal_fsm.Get_Now_State() == VISION)
    {
        MotorJ4310.ctrl_Mit(0x01, 
                           gimbal_target.target_pitch*3.1415926f/180.0f, 
                           0.0f, 
                           pitch_vision_pid.getK(0), 
                           pitch_vision_pid.getK(2), 
                           gimbal_output.out_pitch);
    }
    else
    {
        MotorJ4310.ctrl_Mit(0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    
    // 发射机构
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_dial), 1); 
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[0]), 2);
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[1]), 3);
    Motor3508.sendCAN();
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
        osDelay(5);       // 延时1ms，控制频率1000Hz
    } 
}

}
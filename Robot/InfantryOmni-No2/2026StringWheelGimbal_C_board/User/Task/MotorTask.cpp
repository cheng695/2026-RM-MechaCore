#include "MotorTask.hpp"
#include "ControlTask.hpp"
#include "Module/RobotContext.hpp"


/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<2> Motor3508(0x200, {2, 3}, 0x200);
BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {2, 6}, {0x01, 0x05}); // 1是pitch，2是yaw
BSP::Motor::DM::J4340<1> MotorJ4340(0x00, {4}, {0x03});
BSP::Motor::LK::LK4005<1> MotorLK4005(0x140, {1}, {0x01});

float Motor4310_angle[2] = {0.0f, 0.0f};
float Motor4310_velocity[2] = {0.0f, 0.0f};

namespace
{
enum class DmEnablePhase
{
    ClearError,
    Enable,
    WaitFeedback
};

struct DmEnableState
{
    DmEnablePhase phase = DmEnablePhase::ClearError;
    uint8_t wait_ticks = 0;
};

constexpr uint8_t DM_ENABLE_RETRY_TICKS = 5;

/* 达妙电机上电握手：清错误、发使能、等待 err=1，失败后循环重试。 */
template <typename MotorType>
bool UpdateDmEnable(MotorType& motor, uint8_t motor_id, DmEnableState& state)
{
    if (motor.GetErr(motor_id) == 1)
    {
        state.phase = DmEnablePhase::WaitFeedback;
        state.wait_ticks = 0;
        return true;
    }

    switch (state.phase)
    {
    case DmEnablePhase::ClearError:
        motor.ClearErr(motor_id, BSP::Motor::DM::MIT);
        state.phase = DmEnablePhase::Enable;
        state.wait_ticks = 0;
        break;
    case DmEnablePhase::Enable:
        motor.On(motor_id, BSP::Motor::DM::MIT);
        state.phase = DmEnablePhase::WaitFeedback;
        state.wait_ticks = 0;
        break;
    case DmEnablePhase::WaitFeedback:
        ++state.wait_ticks;
        if (state.wait_ticks >= DM_ENABLE_RETRY_TICKS)
        {
            state.phase = DmEnablePhase::ClearError;
            state.wait_ticks = 0;
        }
        break;
    }

    return false;
}

/*
 * 达妙采用应答式通信。握手期间和 STOP 状态下仍需持续发送 MIT 帧，
 * 否则电机会认为通信中断，主控也无法持续获得反馈数据。
 */
static void SendDmZeroMit()
{
    MotorJ4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    MotorJ4310.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    MotorJ4340.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}
} // namespace

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
        MotorLK4005.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        MotorJ4310.Parse(frame);
        MotorJ4340.Parse(frame);
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
    static DmEnableState pitch2_enable_state;
    static DmEnableState yaw_enable_state;
    static DmEnableState pitch1_enable_state;

    /* 三个达妙电机全部完成握手前，不进入正常 MIT 控制。 */
    const bool pitch2_ready = UpdateDmEnable(MotorJ4310, 1, pitch2_enable_state);
    const bool yaw_ready = UpdateDmEnable(MotorJ4310, 2, yaw_enable_state);
    const bool pitch1_ready = UpdateDmEnable(MotorJ4340, 1, pitch1_enable_state);

    if (!(pitch2_ready && yaw_ready && pitch1_ready))
    {
        /* 握手未完成时也发送零 MIT，维持达妙反馈通信。 */
        SendDmZeroMit();
        /* 达妙尚未准备好时，发射机构保持停止。 */
        Motor3508.setCAN(0, 1);
        Motor3508.setCAN(0, 2);
        MotorLK4005.ctrl_Torque(1, 1, 0);
        Motor3508.sendCAN();
        return;
    }

    if (gimbal_fsm.Get_Now_State() == STOP)
    {
        /* STOP 只保持通信，不执行任何角度、速度或力矩控制。 */
        SendDmZeroMit();
        Motor3508.setCAN(0, 1);
        Motor3508.setCAN(0, 2);
        MotorLK4005.ctrl_Torque(1, 1, 0);
        Motor3508.sendCAN();
        return;
    }

    /*
     * 达妙使能已经由 UpdateDmEnable() 统一完成。
     * 这里仅负责发送正常 MIT 控制帧，不再重复判断 err、On 或 Off。
     */
    MotorJ4340.ctrl_Mit(1,
                        gimbal_output.out_J1_angle,
                        gimbal_output.out_J1_velocity,
                        gimbal_k.J1_Kp,
                        gimbal_k.J1_Kd,
                        gimbal_output.out_J1_torque);

    MotorJ4310.ctrl_Mit(1,
                        gimbal_output.out_J2_angle,
                        gimbal_output.out_J2_velocity,
                        gimbal_k.J2_Kp,
                        gimbal_k.J2_Kd,
                        gimbal_output.out_J2_torque);

    MotorJ4310.ctrl_Mit(2,
                        gimbal_output.out_yaw_angle,
                        gimbal_output.out_yaw_velocity,
                        gimbal_k.yaw_Kp,
                        gimbal_k.yaw_Kd,
                        gimbal_output.out_yaw_torque);
    
    // 发射机构
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[0]), 1);
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[1]), 2);
    MotorLK4005.ctrl_Torque(1, 1, static_cast<int16_t>(launch_output.out_dial));
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
        motor_control();            // 执行电机控制和电机 CAN 发送
        
        /* CAN 邮箱释放后，补发各总线缓存的最新控制帧。 */
        HAL::CAN::get_can_bus_instance().get_can1().process_tx_queue();
        HAL::CAN::get_can_bus_instance().get_can2().process_tx_queue();

        for(int i = 0; i < 2; ++i)
        {
            Motor4310_angle[0] = HI12.GetAngle(1) * 3.14159f / 180.0f; // 弧度rad
            Motor4310_angle[1] = MotorJ4310.getAngleRad(i + 1);
            Motor4310_velocity[i] = MotorJ4310.getVelocityRads(i + 1);
        }
        osDelay(1);                 // 延时1ms，控制频率1000Hz
    }
}

}

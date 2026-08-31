#pragma once

#include <cstdint>

#define STATUS_MAX (10)

/* 云台最终工作状态。 */
enum Enum_Gimbal_States
{
    STOP = 0,       /* 停止状态。 */
    DOWN,           /* 已完成折叠的状态。 */
    MANUAL,         /* 手动控制状态。 */
    VISION,         /* 视觉控制状态。 */
    STATUS_COUNT
};

/* 云台状态机事件。事件只表达目标模式，不描述具体电机动作。 */
enum class GimbalEvent
{
    None = 0,           /* 无状态切换请求。 */
    StopRequested,      /* 请求进入停止状态。 */
    DownRequested,      /* 请求进入折叠完成状态。 */
    ManualRequested,    /* 请求进入手动状态。 */
    VisionRequested     /* 请求进入视觉状态。 */
};

/* 云台模式切换时需要执行的半自动动作流程。 */
enum class GimbalTransition
{
    None = 0,
    StopToManual,       /* STOP -> MANUAL：Yaw 保持，Pitch 回正。 */
    StopToDown,         /* STOP -> DOWN：Yaw 对齐，Pitch 直接折叠。 */
    DownToManual,       /* DOWN -> MANUAL：Yaw 保持，Pitch 回正。 */
    ManualToDown,       /* MANUAL -> DOWN：Yaw 对齐，Pitch 折叠。 */
    VisionToDown        /* VISION -> DOWN：Yaw 对齐，Pitch 折叠。 */
};

/* 状态统计信息。 */
struct Struct_Status
{
    const char* Name;
    uint32_t Enter_Count;
    uint32_t Total_Run_Time;
    void* User_Data;
};

class Gimbal_FSM
{
public:
    Struct_Status Status[STATUS_MAX];
    Enum_Gimbal_States State_gimbal;

    /* 初始化状态、转换阶段和统计信息。 */
    void Init();

    inline Enum_Gimbal_States Get_Now_State();
    inline const char* Get_Now_State_Name();

    /* 消费一个明确的模式请求事件。 */
    void Dispatch(GimbalEvent event);

    /*
     * 转换控制器完成当前动作后调用。
     * 例如 Pitch 回正完成、或 Pitch 折叠完成后，才允许提交最终状态。
     */
    void CompleteTransition();

    /* 设备离线等安全场景下立即取消转换并进入 STOP。 */
    void ForceStop();

    inline GimbalTransition Get_Transition() const;
    inline bool Is_Transitioning() const;

    /*
     * 旧控制循环的临时兼容接口。
     * 新的事件检测器接入后应删除此接口及其调用。
     */
    void StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool vision_flag);

    void TIM_Update();
    uint32_t Get_State_Run_Time(Enum_Gimbal_States state);
    uint32_t Get_State_Enter_Count(Enum_Gimbal_States state);
    void Reset_State_Statistics(Enum_Gimbal_States state);

private:
    uint32_t State_Run_Time[STATUS_MAX] = {0};
    GimbalTransition transition_ = GimbalTransition::None;

    void TransitTo(Enum_Gimbal_States next_state);
    void BeginTransition(GimbalTransition transition);
};

inline Enum_Gimbal_States Gimbal_FSM::Get_Now_State()
{
    return State_gimbal;
}

inline const char* Gimbal_FSM::Get_Now_State_Name()
{
    return Status[State_gimbal].Name;
}

inline GimbalTransition Gimbal_FSM::Get_Transition() const
{
    return transition_;
}

inline bool Gimbal_FSM::Is_Transitioning() const
{
    return transition_ != GimbalTransition::None;
}

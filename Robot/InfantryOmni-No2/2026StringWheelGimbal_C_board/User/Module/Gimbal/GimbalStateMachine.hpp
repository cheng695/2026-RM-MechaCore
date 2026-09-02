#pragma once

#include <cstdint>

#define STATUS_MAX (10)

/*
 * 云台最终工作状态。
 *
 * STOP、DOWN、MANUAL、VISION 是对外可见的稳定状态。
 * 折叠和起立过程不是稳定工作模式，由 GimbalTransition 表示。
 */
enum Enum_Gimbal_States
{
    STOP = 0,       /* 停止状态。 */
    DOWN,           /* 折叠完成状态。 */
    MANUAL,         /* 普通手动状态。 */
    VISION,         /* 视觉控制状态。 */
    STATUS_COUNT
};

/*
 * 云台状态机事件。
 *
 * 事件只表达“请求进入哪个模式”，不关心遥控器开关、鼠标、视觉标志等输入细节。
 * 这些输入由 GimbalEventDetector 解释，FSM 只负责状态切换和统计。
 */
enum class GimbalEvent
{
    None = 0,           /* 无状态切换请求。 */
    StopRequested,      /* 请求进入 STOP。 */
    DownRequested,      /* 请求进入 DOWN，可能先触发折叠过渡动作。 */
    ManualRequested,    /* 请求进入 MANUAL，可能先触发起立过渡动作。 */
    VisionRequested     /* 请求进入 VISION，只允许从 MANUAL 进入。 */
};

/*
 * 云台半自动过渡流程。
 *
 * 过渡流程用于描述“还没到最终状态，但正在执行折叠/起立动作”的阶段。
 * Controller 和 Planner 根据该值生成中间目标；动作到位后调用 CompleteTransition()。
 */
enum class GimbalTransition
{
    None = 0,
    StopToManual,       /* STOP -> MANUAL：Yaw 保持当前位置，Pitch 起立。 */
    StopToDown,         /* STOP -> DOWN：Yaw 先对齐固定角，Pitch 直接折叠。 */
    DownToManual,       /* DOWN -> MANUAL：Yaw 保持当前位置，Pitch 起立。 */
    ManualToDown,       /* MANUAL -> DOWN：Yaw 先对齐固定角，Pitch 折叠。 */
    VisionToDown        /* VISION -> DOWN：Yaw 先对齐固定角，Pitch 折叠。 */
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

    /* 初始化状态、过渡阶段和统计信息。 */
    void Init();

    inline Enum_Gimbal_States Get_Now_State();
    inline const char* Get_Now_State_Name();

    /* 消费一个明确的模式请求事件。 */
    void Dispatch(GimbalEvent event);

    /*
     * Controller 判断当前过渡动作完成后调用。
     * 例如 Pitch 起立到位、或 Pitch 折叠到位后，FSM 才提交到最终稳定状态。
     */
    void CompleteTransition();

    /* 设备离线等安全场景下立即取消过渡并进入 STOP。 */
    void ForceStop();

    inline GimbalTransition Get_Transition() const;
    inline bool Is_Transitioning() const;

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

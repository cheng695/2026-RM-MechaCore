#pragma once

#include <cstdint>

#define STATUS_MAX (10)

/* 发射机构的稳定工作状态。 */
enum Enum_Launch_States
{
    LAUNCH_STOP = 0,      /* 完全停止。 */
    LAUNCH_CEASEFIRE,     /* 摩擦轮预旋，拨盘不送弹。 */
    LAUNCH_ONLY,          /* 单发模式。每次触发只执行一发动作。 */
    LAUNCH_AUTO,          /* 连发模式。拨盘持续送弹。 */
    LAUNCH_JAM,           /* 卡弹处理，拨盘反转退弹。 */
    LAUNCH_STATUS_COUNT
};

/* 发射机构状态机事件。事件直接表达目标工作模式或安全动作。 */
enum class LaunchEvent
{
    None = 0,             /* 无事件。 */
    StopRequested,        /* 请求完全停止。 */
    CeasefireRequested,   /* 请求进入停火待机。 */
    OnlyRequested,        /* 请求进入单发模式。 */
    AutoRequested,        /* 请求进入连发模式。 */
    JamDetected           /* 检测到卡弹，请求进入退弹状态。 */
};

/* 发射机构状态统计信息。 */
struct Struct_Status_launch
{
    const char* Name;
    uint32_t Enter_Count;
    uint32_t Total_Run_Time;
    void* User_Data;
};

class Launch_FSM
{
public:
    Struct_Status_launch Status[STATUS_MAX];
    Enum_Launch_States State_launch;

    /* 初始化状态、统计信息和卡弹恢复状态。 */
    void Init();

    inline Enum_Launch_States Get_Now_State();
    inline const char* Get_Now_State_Name();

    /* 消费一个发射机构事件。重复请求当前状态不会重复进入状态。 */
    void Dispatch(LaunchEvent event);

    /* 设备离线或其他最高优先级安全条件下立即停止。 */
    void ForceStop();

    /* 周期更新时间统计，并处理 JAM 的 150 ms 自动恢复。 */
    void TIM_Update();

    uint32_t Get_State_Run_Time(Enum_Launch_States state);
    uint32_t Get_State_Enter_Count(Enum_Launch_States state);
    void Reset_State_Statistics(Enum_Launch_States state);

private:
    static constexpr uint32_t JAM_DURATION_TICKS = 30;
    static constexpr uint32_t ONLY_TIMEOUT_TICKS = 4000;

    uint32_t State_Run_Time[STATUS_MAX] = {0};

    /* JAM 结束后恢复到进入卡弹处理前的稳定状态。 */
    Enum_Launch_States resume_state_ = LAUNCH_CEASEFIRE;

    void TransitTo(Enum_Launch_States next_state);
    void BeginJam();
};

inline Enum_Launch_States Launch_FSM::Get_Now_State()
{
    return State_launch;
}

inline const char* Launch_FSM::Get_Now_State_Name()
{
    return Status[State_launch].Name;
}

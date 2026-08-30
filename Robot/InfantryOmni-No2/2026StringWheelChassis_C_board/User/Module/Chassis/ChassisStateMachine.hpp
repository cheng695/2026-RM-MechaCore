#pragma once

#include <stdint.h>

#define STATUS_MAX (10)

/*
 * 底盘工作状态。
 *
 * STOP      : 停止，电机输出清零。
 * FOLLOW    : 底盘跟随云台。
 * NOTFOLLOW : 底盘不跟随云台，旋转由操作员输入决定。
 */
enum Enum_Chassis_States
{
    STOP = 0,
    FOLLOW,
    NOTFOLLOW,
    STATUS_COUNT
};

/*
 * 底盘状态机事件。
 *
 * 事件直接表达“请求进入哪个状态”。遥控器开关、Q 键、设备在线状态等输入细节
 * 由 ChassisEventDetector 解释，FSM 只负责切换状态和维护统计信息。
 */
enum class ChassisEvent
{
    None = 0,             /* 无状态切换请求。 */
    StopRequested,        /* 请求进入停止/急停状态。 */
    FollowRequested,      /* 请求进入底盘跟随状态。 */
    NotFollowRequested,   /* 请求进入底盘不跟随状态。 */
};

/*
 * 状态统计信息。
 */
struct Struct_Status
{
    const char* Name;
    uint32_t Enter_Count;
    uint32_t Total_Run_Time;
    void* User_Data;
};

/*
 * 底盘有限状态机。
 */
class Chassis_FSM
{
public:
    Struct_Status Status[STATUS_MAX];
    Enum_Chassis_States State_chassis;

    void Init();

    inline Enum_Chassis_States Get_Now_State();
    inline const char* Get_Now_State_Name();

    /*
     * 事件驱动入口。传入事件应当已经是明确的状态切换请求。
     */
    void Dispatch(ChassisEvent event);

    /*
     * 安全停机入口。设备离线时可直接强制刷新到 STOP。
     */
    void ForceStop();

    void TIM_Update();
    uint32_t Get_State_Run_Time(Enum_Chassis_States state);
    uint32_t Get_State_Enter_Count(Enum_Chassis_States state);
    void Reset_State_Statistics(Enum_Chassis_States state);

private:
    uint32_t State_Run_Time[STATUS_MAX] = {0};

    void TransitTo(Enum_Chassis_States next_state);
};

inline Enum_Chassis_States Chassis_FSM::Get_Now_State()
{
    return State_chassis;
}

inline const char* Chassis_FSM::Get_Now_State_Name()
{
    return Status[State_chassis].Name;
}

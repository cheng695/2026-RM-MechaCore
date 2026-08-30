#include "ChassisStateMachine.hpp"

namespace
{
const char* State_Names[STATUS_COUNT] = {
    "STOP",
    "FOLLOW",
    "NOTFOLLOW",
};
} /* namespace */

/*
 * 初始化状态机和状态统计信息。
 */
void Chassis_FSM::Init()
{
    for (int i = 0; i < STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_chassis = STOP;
    Status[STOP].Enter_Count = 1;
}

/*
 * 执行状态转换，并在状态变化时更新运行时间和进入次数统计。
 */
void Chassis_FSM::TransitTo(Enum_Chassis_States next_state)
{
    if (next_state < STOP || next_state >= STATUS_COUNT)
    {
        next_state = STOP;
    }

    if (State_chassis == next_state)
    {
        return;
    }

    Status[State_chassis].Total_Run_Time += State_Run_Time[State_chassis];
    State_Run_Time[State_chassis] = 0;

    State_chassis = next_state;
    Status[State_chassis].Enter_Count++;
}

/*
 * 事件驱动状态更新入口。
 *
 * FSM 不再理解左右开关、Q 键或设备在线状态，只接收
 * StopRequested / FollowRequested / NotFollowRequested 这类明确事件。
 */
void Chassis_FSM::Dispatch(ChassisEvent event)
{
    switch (event)
    {
        case ChassisEvent::StopRequested:
            TransitTo(STOP);
            break;

        case ChassisEvent::FollowRequested:
            TransitTo(FOLLOW);
            break;

        case ChassisEvent::NotFollowRequested:
            TransitTo(NOTFOLLOW);
            break;

        case ChassisEvent::None:
        default:
            break;
    }
}

/*
 * 强制进入 STOP。用于设备离线等安全保护场景。
 */
void Chassis_FSM::ForceStop()
{
    TransitTo(STOP);
}

/*
 * 周期性统计当前状态运行时间。
 */
void Chassis_FSM::TIM_Update()
{
    State_Run_Time[State_chassis]++;
}

/*
 * 获取指定状态的累计运行时间。
 *
 * 如果查询的是当前状态，需要把已经结算的历史时间和当前仍在运行的时间相加；
 * 如果查询的是非当前状态，直接返回已经结算的历史累计时间。
 */
uint32_t Chassis_FSM::Get_State_Run_Time(Enum_Chassis_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_chassis)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }

    return Status[state].Total_Run_Time;
}

/*
 * 获取指定状态的进入次数。
 *
 * 每次 TransitTo() 成功切换到目标状态时，该状态的 Enter_Count 会加一。
 */
uint32_t Chassis_FSM::Get_State_Enter_Count(Enum_Chassis_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    return Status[state].Enter_Count;
}

/*
 * 重置指定状态的统计信息。
 *
 * 如果重置的是当前正在运行的状态，Enter_Count 重新置为 1，
 * 表示当前这次进入仍然有效；否则进入次数和累计时间都清零。
 */
void Chassis_FSM::Reset_State_Statistics(Enum_Chassis_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_chassis)
    {
        Status[state].Enter_Count = 1;
    }
}

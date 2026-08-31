#include "LaunchStateMachine.hpp"

namespace
{
const char* STATE_NAMES[LAUNCH_STATUS_COUNT] = {
    "LAUNCH_STOP",
    "LAUNCH_CEASEFIRE",
    "LAUNCH_ONLY",
    "LAUNCH_AUTO",
    "LAUNCH_JAM"
};
}

void Launch_FSM::Init()
{
    for (int i = 0; i < LAUNCH_STATUS_COUNT; ++i)
    {
        Status[i].Name = STATE_NAMES[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_launch = LAUNCH_STOP;
    resume_state_ = LAUNCH_CEASEFIRE;
    Status[LAUNCH_STOP].Enter_Count = 1;
}

void Launch_FSM::TransitTo(Enum_Launch_States next_state)
{
    if (next_state < LAUNCH_STOP || next_state >= LAUNCH_STATUS_COUNT)
    {
        next_state = LAUNCH_STOP;
    }

    if (State_launch == next_state)
    {
        return;
    }

    /* 只有真正切换状态时才结算原状态运行时间和进入次数。 */
    Status[State_launch].Total_Run_Time += State_Run_Time[State_launch];
    State_Run_Time[State_launch] = 0;
    State_launch = next_state;
    Status[State_launch].Enter_Count++;
}

void Launch_FSM::BeginJam()
{
    /* 只允许从稳定发射状态进入 JAM，避免恢复到 STOP 或 JAM。 */
    if (State_launch == LAUNCH_ONLY || State_launch == LAUNCH_AUTO)
    {
        resume_state_ = State_launch;
        TransitTo(LAUNCH_JAM);
    }
}

void Launch_FSM::Dispatch(LaunchEvent event)
{
    if (event == LaunchEvent::None)
    {
        return;
    }

    /* 停止事件优先级最高，可以打断 JAM 和其他发射状态。 */
    if (event == LaunchEvent::StopRequested)
    {
        ForceStop();
        return;
    }

    /* JAM 期间只接受停止事件，避免退弹动作被普通模式请求打断。 */
    if (State_launch == LAUNCH_JAM)
    {
        return;
    }

    switch (event)
    {
        case LaunchEvent::CeasefireRequested:
            TransitTo(LAUNCH_CEASEFIRE);
            break;

        case LaunchEvent::OnlyRequested:
            TransitTo(LAUNCH_ONLY);
            break;

        case LaunchEvent::AutoRequested:
            TransitTo(LAUNCH_AUTO);
            break;

        case LaunchEvent::JamDetected:
            BeginJam();
            break;

        case LaunchEvent::StopRequested:
        case LaunchEvent::None:
        default:
            break;
    }
}

void Launch_FSM::ForceStop()
{
    resume_state_ = LAUNCH_CEASEFIRE;
    TransitTo(LAUNCH_STOP);
}

void Launch_FSM::TIM_Update()
{
    if (State_launch < LAUNCH_STOP || State_launch >= LAUNCH_STATUS_COUNT)
    {
        State_launch = LAUNCH_STOP;
        return;
    }

    State_Run_Time[State_launch]++;

    /* JAM 持续 30 个 5 ms 控制周期，即约 150 ms，然后恢复原模式。 */
    if (State_launch == LAUNCH_JAM && State_Run_Time[LAUNCH_JAM] >= JAM_DURATION_TICKS)
    {
        const Enum_Launch_States next_state =
            (resume_state_ == LAUNCH_ONLY || resume_state_ == LAUNCH_AUTO)
                ? resume_state_
                : LAUNCH_CEASEFIRE;
        TransitTo(next_state);
    }

    /* 单发模式持续超时后自动转为连发，保留旧逻辑的保护行为。 */
    if (State_launch == LAUNCH_ONLY && State_Run_Time[LAUNCH_ONLY] >= ONLY_TIMEOUT_TICKS)
    {
        TransitTo(LAUNCH_AUTO);
    }
}

uint32_t Launch_FSM::Get_State_Run_Time(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_launch)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }

    return Status[state].Total_Run_Time;
}

uint32_t Launch_FSM::Get_State_Enter_Count(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return 0;
    }

    return Status[state].Enter_Count;
}

void Launch_FSM::Reset_State_Statistics(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_launch)
    {
        Status[state].Enter_Count = 1;
    }
}

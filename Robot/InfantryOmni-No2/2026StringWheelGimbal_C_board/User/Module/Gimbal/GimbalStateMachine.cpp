#include "GimbalStateMachine.hpp"

namespace
{
const char* State_Names[STATUS_COUNT] = {
    "STOP",
    "DOWN",
    "MANUAL",
    "VISION"
};
} /* namespace */

void Gimbal_FSM::Init()
{
    for (int i = 0; i < STATUS_COUNT; ++i)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_gimbal = STOP;
    Status[STOP].Enter_Count = 1;
    transition_ = GimbalTransition::None;
}

/** 
 * 将云台状态机切换到指定状态。
 * @param next_state 目标状态
 */
void Gimbal_FSM::TransitTo(Enum_Gimbal_States next_state)
{
    if (next_state < STOP || next_state >= STATUS_COUNT)
    {
        next_state = STOP;
    }

    if (State_gimbal == next_state)
    {
        return;
    }

    /* 状态真正提交时才更新统计，转换过程不计入最终工作状态。 */
    Status[State_gimbal].Total_Run_Time += State_Run_Time[State_gimbal];
    State_Run_Time[State_gimbal] = 0;
    State_gimbal = next_state;
    Status[State_gimbal].Enter_Count++;
}

/**
 * @brief 开始状态转换
 * 
 * @param transition 转换类型
 */
void Gimbal_FSM::BeginTransition(GimbalTransition transition)
{
    /* 转换进行中不重复启动新的姿态动作。STOP 请求除外，由 Dispatch 单独处理。 */
    if (transition_ == GimbalTransition::None)
    {
        transition_ = transition;
    }
}

/**
 * @brief 分发事件
 * 
 * @param event 事件类型
 */
void Gimbal_FSM::Dispatch(GimbalEvent event)
{
    if (event == GimbalEvent::None)
    {
        return;
    }

    /* 急停具有最高优先级，可以打断所有折叠和回正动作。 */
    if (event == GimbalEvent::StopRequested)
    {
        ForceStop();
        return;
    }

    /* 半自动动作未完成前，不接受新的普通模式请求。 */
    if (Is_Transitioning())
    {
        return;
    }

    switch (event)
    {
        case GimbalEvent::DownRequested:
            if (State_gimbal == STOP)
            {
                BeginTransition(GimbalTransition::StopToDown);
            }
            else if (State_gimbal == MANUAL)
            {
                BeginTransition(GimbalTransition::ManualToDown);
            }
            else if (State_gimbal == VISION)
            {
                BeginTransition(GimbalTransition::VisionToDown);
            }
            /* 已经处于 DOWN 时不重复执行折叠动作。 */
            break;

        case GimbalEvent::ManualRequested:
            if (State_gimbal == STOP)
            {
                BeginTransition(GimbalTransition::StopToManual);
            }
            else if (State_gimbal == DOWN)
            {
                BeginTransition(GimbalTransition::DownToManual);
            }
            else if (State_gimbal == VISION)
            {
                TransitTo(MANUAL);
            }
            break;

        case GimbalEvent::VisionRequested:
            /* VISION 只能从 MANUAL 进入，STOP 和 DOWN 请求会被拒绝。 */
            if (State_gimbal == MANUAL)
            {
                TransitTo(VISION);
            }
            break;

        case GimbalEvent::StopRequested:
        case GimbalEvent::None:
        default:
            break;
    }
}

/**
 * @brief 状态转换完成
 * 
 */
void Gimbal_FSM::CompleteTransition()
{
    Enum_Gimbal_States next_state = State_gimbal;

    switch (transition_)
    {
        case GimbalTransition::StopToManual:
        case GimbalTransition::DownToManual:
            next_state = MANUAL;
            break;

        case GimbalTransition::StopToDown:
        case GimbalTransition::ManualToDown:
        case GimbalTransition::VisionToDown:
            next_state = DOWN;
            break;

        case GimbalTransition::None:
        default:
            return;
    }

    transition_ = GimbalTransition::None;
    TransitTo(next_state);
}

void Gimbal_FSM::ForceStop()
{
    transition_ = GimbalTransition::None;
    TransitTo(STOP);
}


void Gimbal_FSM::TIM_Update()
{
    if (State_gimbal >= STOP && State_gimbal < STATUS_COUNT)
    {
        State_Run_Time[State_gimbal]++;
    }
}

uint32_t Gimbal_FSM::Get_State_Run_Time(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_gimbal)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }

    return Status[state].Total_Run_Time;
}

uint32_t Gimbal_FSM::Get_State_Enter_Count(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    return Status[state].Enter_Count;
}

void Gimbal_FSM::Reset_State_Statistics(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_gimbal)
    {
        Status[state].Enter_Count = 1;
    }
}

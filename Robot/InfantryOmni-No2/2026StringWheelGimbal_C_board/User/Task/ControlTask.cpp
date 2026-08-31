#include "ControlTask.hpp"
#include "Module/Gimbal/GimbalController.hpp"
#include "Module/Launch/LaunchController.hpp"
#include "Module/Gimbal/GimbalPlanner.hpp"
#include "Module/Launch/LaunchPlanner.hpp"
#include "Module/Launch/LaunchInputBuilder.hpp"
#include "Module/Launch/LaunchEventDetector.hpp"

bool check_online()
{
    bool isconnected = true;
    // 上场要注释掉。不然发射机构掉电云台会失能，过不了检录。
    // if(!Motor6020.isConnected(1, 6) || !MotorJ4310.isConnected(1, 4) || !Motor3508.isConnected(1, 2) || !Motor3508.isConnected(1, 3) || !Motor3508.isConnected(1, 1))
    // {
    //     isconnected = false;
    // }

    if(!DT7.isConnected() || !HI12.isConnected())
    {
        isconnected = false;
    }
    
    if(!isconnected)
    {
        return false;
    }

    return true;
}

void fsm_init()
{
    InitGimbalPlanner();
    InitLaunchPlanner();
    gimbal_fsm.Init();
    launch_fsm.Init();
}

extern "C"
{
void Control(void const * argument)
{
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
    static uint8_t control_tick = 0;
    for (;;)
    {
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        float current[2] = {Motor3508.getCurrent(2), Motor3508.getCurrent(3)};
        float velocity[2] = {Motor3508.getVelocityRpm(2), Motor3508.getVelocityRpm(3)};
        heat_control.HeatControl(gimbal_target.target_surgewheel[0], current, velocity,
                                 Cboard.GetHeatLimit(), Cboard.GetHeatCool(), 0.001f, 15.0f);
        ++control_tick;
        if (control_tick >= 5)
        {
            control_tick = 0;
            RunGimbalController();
            launch_fsm.TIM_Update();
            const LaunchInput launch_input = BuildLaunchInput(operator_input, check_online(), is_jamming());
            launch_fsm.Dispatch(DetectLaunchEvent(launch_input, launch_fsm.Get_Now_State()));
            RunLaunchController();
        }
        osDelay(1);
    }
}
}

#include "ControlTask.hpp"
#include "Module/Gimbal/GimbalController.hpp"
#include "Module/Launch/LaunchController.hpp"
#include "Module/Launch/LaunchPlanner.hpp"
#include "Module/Launch/LaunchInputBuilder.hpp"
#include "Module/Launch/LaunchEventDetector.hpp"
#include "Module/RobotContext.hpp"

bool check_online()
{
    bool isconnected = true;
    // 涓婂満瑕佹敞閲婃帀銆備笉鐒跺彂灏勬満鏋勬帀鐢典簯鍙颁細澶辫兘锛岃繃涓嶄簡妫€褰曘€?    // if(!Motor6020.isConnected(1, 6) || !MotorJ4310.isConnected(1, 4) || !Motor3508.isConnected(1, 2) || !Motor3508.isConnected(1, 3) || !Motor3508.isConnected(1, 1))
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
    InitGimbalControl();
    InitLaunchPlanner();
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
        heat_control.HeatControl(launch_planner_target.target_surgewheel[0], current, velocity,
                                 Cboard.GetHeatLimit(), Cboard.GetHeatCool(), 0.001f, 15.0f);
        ++control_tick;
        if (control_tick >= 5)
        {
            control_tick = 0;
            RunGimbalController();
            launch_fsm.TIM_Update();
            const LaunchInput launch_input = BuildLaunchInput(operator_input, check_online(), is_jamming());
            launch_fsm.Dispatch(DetectLaunchEvent(launch_input, launch_fsm.Get_Now_State()));
            RunLaunchController(launch_input);
        }
        osDelay(1);
    }
}
}

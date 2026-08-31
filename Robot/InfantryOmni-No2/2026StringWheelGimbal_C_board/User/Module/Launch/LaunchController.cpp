#include "LaunchController.hpp"
#include "Task/ControlTask.hpp"
#include "Module/Launch/LaunchEventDetector.hpp"
#include <algorithm>
#include <cmath>

/* 发射机构部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 发射机构停止模式控制函数
 * 
 * 重置所有控制器并设置输出为0
 */
void launch_velocity()
{
    // 速度环期望值置零，快速停下
    dial_pid.UpDate(gimbal_target.target_dial, Motor3508.getVelocityRpm(1));
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    launch_output.out_dial = dial_pid.getOutput();
}

/**
 * @brief 发射机构单发模式控制函数
 * 
 * 拨盘按角度控制，摩擦轮保持设定转速
 */
void launch_angle()
{
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();

    // 拨盘角度环
    dial_angle_pid.UpDate(gimbal_target.target_dial, Motor3508.getAddAngleDeg(1));
    dial_velocity_pid.UpDate(dial_angle_pid.getOutput(), Motor3508.getVelocityRpm(1));
    launch_output.out_dial = dial_velocity_pid.getOutput();
}

/**
 * @brief 发射机构卡弹控制函数
 * 
 * 卡弹的时候输出反向力矩
 */
void launch_jam()
{
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(0.0f, Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    launch_output.out_dial = gimbal_target.target_dial;
}

/**
 * @brief 发射机构主控制循环
 * 
 */
void main_loop_launch()
{
    RunLaunchController();
}

/**
 * @brief 卡弹检测
 * 
 * @return true 
 * @return false 
 */
bool is_jamming()
{
    static uint32_t last_check_time = 0;
    static float last_angle = 0;
    
    float current_angle = Motor3508.getAddAngleDeg(1);
    float err = fabs(gimbal_target.target_dial - current_angle);
    
    // 1. 位置误差超过 120 度
    if (err > 120.0f * 36.0f)
    {
        uint32_t now = HAL_GetTick();
        
        // 初始化
        if (last_check_time == 0)
        {
            last_check_time = now;
            last_angle = current_angle;
            return false;
        }
        
        // 2. 每 300ms 检查一次
        if (now - last_check_time > 300)
        {
            // 3. 如果这 300ms 内转动角度小于 120 度 -> 判定卡弹
            if (fabs(current_angle - last_angle) < 120.0f * 36.0f)
            {
                // 重置检测状态（因为即将进入 Jam 模式，Jam模式下 error 会变小）
                last_check_time = 0; 
                return true;
            }
            
            // 更新基准
            last_check_time = now;
            last_angle = current_angle;
        }
    }
    else
    {
        // 误差不大的时候，重置计时器
        last_check_time = 0;
    }
    
    return false;
}



void RunLaunchController()
{
    main_loop_launch();
}


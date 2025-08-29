#include "InHpp.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
AutoLaunch_t AutoLaunch;
int Set_Yaw_Angle[4]   = {3830, 3830, 3830, 3830}; // 预设的yaw角度
int Set_Motor_Speed[4] = {4000, 4000, 4000, 4000}; // 预设的电机转速

/**
 * @brief 飞镖自动发射函数
 * @details 该函数根据不同的模式启动自动发射流程
 * @param mode
 */
void AutoLaunch_c::StartAutoLaunch(int mode)
{
    // 处理启动条件
    HandleLaunchTrigger(mode);

    // 状态机处理
    switch (LaunchState) {
        case INIT_LAUNCH:
            HandleInitLaunch();
            break;
        case WAIT_LIMIT_TRIGGER:
            HandleWaitLimit();
            break;
        case FIRST_DART_LAUNCH:
            HandleFirstDartLaunch();
            break;
        case YAW_ADJUSTMENT:
            HandleYawAdjustment();
            break;
        case SECOND_DART_LAUNCH:
            HandleSecondDartLaunch();
            break;
        case STOP_LAUNCH:
            HandleStopLaunch();
            break;
    }

    // 检查是否需要重置状态
    CheckStateReset(mode);
}

// 处理发射触发条件
void AutoLaunch_c::HandleLaunchTrigger(int mode)
{
    if (mode == DT7CtrlMode && LaunchedCount <= 2 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 2 && LaunchedCount <= 2 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 0 && DT7UartCom.rc.ch3 < 370) {
        VisionControl();
    }
}

// 开始新的发射
void AutoLaunch_c::StartNewLaunch()
{
    LaunchedCount++;
    if (LaunchedCount < 3)
        LaunchState = INIT_LAUNCH;
}

/*****************************************************************************************************************/
// 处理初始化发射状态
void AutoLaunch_c::HandleInitLaunch()
{
    Update_Yaw_And_FrictionSpeed();

    int target = (LaunchedCount == 1) ? 10000 : -10000;
    Set_Motor_Target(&SpeedPID_SlidePlatformM2006.Target, target, -10000, 10000);

    LaunchState = WAIT_LIMIT_TRIGGER;
}

// 处理等待限位触发状态
void AutoLaunch_c::HandleWaitLimit()
{
    if ((Dart.LeftLimit == 1 && LaunchedCount == 1) || (Dart.RightLimit == 1 && LaunchedCount == 2)) {
        Set_Motor_Target(&SpeedPID_SlidePlatformM2006.Target, 0, 0, 0); // 如果到达限位，停止左右丝杆
        // 如果角度到达
        if (Angle_Reached == true) {

            Set_Motor_Target(&SpeedPID_VerticalLiftM2006.Target, 6000, -6000, 6000); // 设置上下丝杆目标转速
            LaunchTime = SystemTick;                                                 // 重置发射时间计数

            LaunchState = FIRST_DART_LAUNCH; // 更新自动发射状态
        }
    }
}

// 处理第一次发射状态
void AutoLaunch_c::HandleFirstDartLaunch()
{
    // 发射第一次
    if (SystemTick - LaunchTime > 3700) { // 如果发射时间超过3700ms

        Set_Motor_Target(&SpeedPID_VerticalLiftM2006.Target, 0, 0, 0); // 停止上下丝杆
        MotorConfigID++;                                               // 切换到下一个电机配置ID
        Update_Yaw_And_FrictionSpeed();                                // 更新Yaw轴角度和摩擦轮速度
        LaunchTime = SystemTick;                                       // 重置发射时间计数

        LaunchState = YAW_ADJUSTMENT; // 更新自动发射状态
    }
}

// 处理Yaw角度调整状态
void AutoLaunch_c::HandleYawAdjustment()
{
    if (Angle_Reached == true) {

        LaunchTime = SystemTick;                                                 // 重置发射时间计数
        Set_Motor_Target(&SpeedPID_VerticalLiftM2006.Target, 6000, -6000, 6000); // 设置上下丝杆目标转速

        LaunchState = SECOND_DART_LAUNCH; // 更新自动发射状态
    }
}

// 处理第二次发射状态
void AutoLaunch_c::HandleSecondDartLaunch()
{
    // 发射第二次
    if (SystemTick - LaunchTime > 3700) {                                         // 如果发射时间超过3700ms
        Set_Motor_Target(&SpeedPID_VerticalLiftM2006.Target, -6000, -6000, 6000); // 停止上下丝杆，并回退
        MotorConfigID++;                                                          // 切换到下一个电机配置ID
        LaunchTime = SystemTick;                                                  // 重置发射时间计数

        LaunchState = STOP_LAUNCH; // 更新自动发射状态
    }
}

// 处理结束发射状态
void AutoLaunch_c::HandleStopLaunch()
{
    if (SystemTick - LaunchTime > 7500) {
        Set_Motor_Target(&SpeedPID_VerticalLiftM2006.Target, 0, 0, 0); // 停止上下丝杆
        Set_Motor_Target(&SpeedPID_RightDownFriction.Target, 0, 0, 0); // 停止摩擦轮

        LaunchState = FINISH_LAUNCH; // 更新自动发射状态
    }
}

/*****************************************************************************************************************/

// 检查是否需要重置状态
void AutoLaunch_c::CheckStateReset(int mode)
{
    bool shouldReset = ((DT7UartCom.rc.s2 == MID && mode == DT7CtrlMode) ||
                        (Dart.RefereeSystemState == 0 && mode == RfSysMode)) &&
                       LaunchState == FINISH_LAUNCH;

    if (shouldReset) {
        LaunchState = IDLE;
    }
}

void AutoLaunch_c::VisionControl()
{
    // 视觉控制函数，暂时未实现
    // 这里可以添加视觉相关的控制逻辑
    // Set_Motor_Target(&SpeedPID_AngleSensorM3508.Target, VisionUartReceive.Target_Yaw);
    Dart.Yaw_Angle = VisionUartReceive.Target_Yaw;
}

/**
 * @brief 更新Yaw轴角度和摩擦轮速度
 * @details 该函数用于更新Yaw轴角度和摩擦轮速度，并调用限制函数确保角度在有效范围内
 */
void AutoLaunch_c::Update_Yaw_And_FrictionSpeed()
{
    // 更新Yaw轴角度和摩擦轮速度
    Set_Motor_Target(&Dart.Yaw_Angle, Set_Yaw_Angle[MotorConfigID]);
    Set_Motor_Target(&SpeedPID_RightDownFriction.Target, Set_Motor_Speed[MotorConfigID]); // 设置摩擦轮目标转速
}

/**
 * @brief 同步Yaw轴目标角度到PID控制器
 * @details 该函数将Yaw轴的目标角度同步到PID控制器中，以便进行控制
 */
void AutoLaunch_c::SyncYawAngle()
{
    if (std::abs(Set_Yaw_Angle[MotorConfigID] - SpeedPID_AngleSensorM3508.Current) >= 2)
        Angle_Reached = false;
    else
        Angle_Reached = true;

    SpeedPID_AngleSensorM3508.Target = Dart.Yaw_Angle;
}

/**
 * @brief 设置电机目标值
 * @param value 电机目标值
 * @details 该函数用于设置电机的目标值，并调用限制函数确保角度在有效范围内
 */
void AutoLaunch_c::Set_Motor_Target(int *target_motor, int value, int min, int max)
{
    *target_motor = Dart.Limit_Value(value, min, max);
}

#ifndef __AutoLaunch_Hpp
#define __AutoLaunch_Hpp


typedef class AutoLaunch_c
{
public:
    int LaunchState;    // 发射状态
    int LaunchedCount;      // 发射计数
    int MotorConfigID;      // 电机配置ID
    int LaunchTime;         // 发射时间定时器
    bool Angle_Reached;     // 角度是否到达

    void StartAutoLaunch(int mode); // 启动自动发射
    void HandleLaunchTrigger(int mode); // 处理发射触发条件
    void StartNewLaunch(); // 开始新的发射

    void HandleInitLaunch(); // 处理初始化发射状态
    void HandleWaitLimit(); // 处理等待限位触发状态
    void HandleFirstDartLaunch(); // 处理第一次发射状态
    void HandleYawAdjustment(); // 处理Yaw角度调整状态
    void HandleSecondDartLaunch(); // 处理第二次发射状态
    void HandleStopLaunch(); // 处理结束发射状态

    void CheckStateReset(int mode); // 检查是否需要重置状态

    void VisionControl();           // 视觉控制
    void Update_Yaw_And_FrictionSpeed(); // 更新Yaw轴角度和摩擦轮速度
    void SyncYawAngle(); // 同步Yaw轴目标角度到PID控制器
    void Set_Motor_Target(int *target_motor, int value, int min = 3580, int max = 4020);
    
    
} AutoLaunch_t;


enum LaunchState {
    IDLE = 0,
    INIT_LAUNCH = 1,        // 初始化发射准备
    WAIT_LIMIT_TRIGGER = 2, // 等待限位触发
    FIRST_DART_LAUNCH = 3,  // 第一个飞镖发射
    YAW_ADJUSTMENT = 4,     // Yaw轴调整
    SECOND_DART_LAUNCH = 5, // 第二个飞镖发射
    STOP_LAUNCH = 6,       // 停止发射
    FINISH_LAUNCH = 7       // 发射完成
};

extern AutoLaunch_t AutoLaunch;

#endif

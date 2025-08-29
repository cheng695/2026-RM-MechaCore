#ifndef __Gimbal_Hpp
#define __Gimbal_Hpp

// extern char FrFireFlag;

// enum ChassisRunTypdef
// {
//     ChassisEBA = 0
//     ,FollowMode = 1
//     ,AdaptiveMode = 2
//     ,GyroMode = 3
//     ,FixMode = 4
// };

// enum OperateMode_t
// {
//     RemoteMode = 0
//     ,KeyBoardMode = 1
// };

// enum GyroSpeed_t
// {
//     MiniGyroSpeed = 20
//     ,MiddleGyroSpeed = 30
//     ,HighGyroSpeed = 40
//     ,MaxGyroSpeed = 50
// };

// enum FrSpeed_t
// {
//     NoFrSpeed = 1200
//     ,FirstFrSpeed = 5180
//     ,MiniFrSpeed = 4400
//     ,MiddleFrSpeed = 4460
//     ,MaxFrSpeed = 4500
// };

// #define FireIntervalDelay 333
// #define VisionFireIntervalDelay 800

// typedef struct
// {
//     struct
//     {
//         float YawIMu;
//         float YawMotor;
//         float PitchIMu;
//         float PitchMotor;
//     }Remote;
//     struct
//     {
//         float YawIMu;
//         float YawMotor;
//         float PitchIMu;
//         float PitchMotor;
//     }Keyboard;
// }Sensitivity_t;
// extern Sensitivity_t Sensitivity;

// typedef struct
// {
//     int Up;
//     int Down;
// }PitchLimit_t;
// extern PitchLimit_t PitchLimit;

// typedef struct
// {
//     short Up;
//     short LeftDown;
//     short RightDown;
//     short Left;
//     short Right;
// }FrExpandRPM_t;
// extern FrExpandRPM_t FrExpandRPM;

// typedef struct
// {
//     FrSpeed_t First;
//     FrSpeed_t Second;
// }FrTargetRPM_t;
// extern FrTargetRPM_t FrTargetRPM;

// typedef struct
// {
//     signed char Vx;
//     signed char Vy;
//     signed char Vw;
//     char RunState1;
//     char GyroSpeed;
//     signed char ExtraControlPower;
//     short YawM6020Angle;
//     short PitchM3508Angle;
//     ChassisRunTypdef ChassisRunMode;
// }GimbalToChassisUartData_t;
// extern GimbalToChassisUartData_t GimbalToChassisUartData;

#define Dart_Launch_Opening_Status ext_dart_client_cmd_0x020A.dart_launch_opening_status
#define Dart_Remaining_Time        ext_dart_remaining_time_0x0105.dart_remaining_time
#define DT7ConnectState            DT7UartCom.Connect_State
#define Dart_Yaw_Angle_Medium      3830

typedef class Dart_c
{
public:
    int RefereeSystemState; // 裁判系统状态
    int LeftLimit;          // 左限位
    int RightLimit;         // 右限位
    int Yaw_Angle;          // yaw角度
    bool Rpm_Change_Lock;   // 转速锁定状态
    bool isDT7Misaligned;   // 表示DT7遥控器的数据帧错位
    static const int deadZone     = 10; // 摇杆死区

    void DartInit();     // 初始化
    void RegularEvent(); // 定期事件函数

    void CheckControllerConnection(); // 检查遥控器连接状态
    void FriSpeedControl();           // 速度控制
    void ManualMotorControl();        // 手动电机控制
    void AutoLaunchMode();            // 自动发射模式

    void EmergencyStop(); // 紧急停止
    void Add_Motor_Target(int *target_motor, int add_value, int min, int max);
    int Limit_Value(int value, int min, int max);

} Dart_t;

enum {
    // 遥控器拨杆状态
    UP   = 1,
    DOWN = 2,
    MID  = 3,

    MAX_YAW   = 4020,
    LEAST_YAW = 3580,

    // 遥控器控制模式
    DT7CtrlMode = 0,
    // 裁判系统控制模式
    RfSysMode = 1,
};

extern Dart_t Dart;
extern TickType_t SystemTick;

#endif

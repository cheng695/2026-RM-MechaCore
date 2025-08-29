#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */
// GimbalToChassisUartData_t GimbalToChassisUartData = {GimbalToChassisUartData.RunState1 = 0x20};
// PitchLimit_t PitchLimit                           = {0};
// char FrFireFlag                                   = 0;
Dart_t Dart;

/*  =========================== 进程的变量 ===========================  */
TickType_t SystemTick; // 系统滴答计数

// unsigned char QueueRxIT_Remote[18]  = {0};
/*  =========================== 函数的声明 ===========================  */

/* Private application code --------------------------------------------------*/
/**
 * @brief 机甲大师云台任务函数
 * @param argument 任务参数
 * @details 该函数是云台任务的主循环，负责处理云台的各种控制逻辑
 */
void Gimbal(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    Dart.DartInit(); // 初始化飞镖相关参数和通信
    /* Infinite loop */
    for (;;) {
        // vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        Dart.RegularEvent(); // 定期事件函数

        SystemTick = xTaskGetTickCount(); // 获取当前系统滴答计数

        // unsigned char QueueRxIT_Remote[DT7UartReceiveLength] = {0};
        // if (osMessageQueueGet(Queue_DT7ToGimbalHandle, QueueRxIT_Remote, 0, 0) == osOK) {
        //     std::copy(QueueRxIT_Remote, QueueRxIT_Remote + DT7UartReceiveLength, DT7UartCom.UnpackingArr);
        // }
        // DT7UartCom.Unpacking();

        // osDelay(1); // 延时1毫秒，避免CPU占用过高
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

void Dart_c::DartInit()
{
    // 初始化飞镖参数
    Dart.Yaw_Angle           = 3830; // 初始化Yaw轴角度
    Rpm_Change_Lock          = true; // 初始化转速锁定状态
    AutoLaunch.LaunchState   = 0;    // 初始化自动发射状态
    AutoLaunch.LaunchedCount = 0;    // 初始化发射计数

    can_filter_init(); // CAN过滤器初始化

    // 初始化遥控器、裁判系统、视觉通信
    HAL_UART_Receive_DMA(&DT7UartHandle, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
    HAL_UART_Receive_DMA(&VisionUartHandle, VisionUartReceive.ReceiveArr, VisionUartReceiveLength);
    HAL_UART_Receive_DMA(&VofaUartHandle, VofaCallBack.ReceiveArr, VofaReceiveLength);
    __HAL_DMA_DISABLE_IT(VofaUartHandle.hdmarx, DMA_IT_HT);
}

/**
 * @brief 定期检查函数
 * @details 该函数定期执行系统检查，整合了多个状态检查功能
 */
void Dart_c::RegularEvent()
{
    // 系统状态检查（裁判系统状态和限位开关检查）
    if (Dart_Launch_Opening_Status == 1 || Dart_Remaining_Time == 0)
        RefereeSystemState = 0; // 仓门关闭或者倒计时结束
    else if (Dart_Launch_Opening_Status == 2)
        RefereeSystemState = 1; // 开启中或者关闭中
    else if (Dart_Launch_Opening_Status == 0 && Dart_Remaining_Time > 0)
        RefereeSystemState = 2; // 仓门开启并且倒计时>0的时候

    // 更新限位状态
    LeftLimit  = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    RightLimit = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);

    // 控制系统检查（整合了遥控器状态检查和模式切换）
    CheckControllerConnection();

    // 遥控器未连接时执行紧急停止
    if (DT7UartCom.isConnected == false || all_motors_connected == false) {
        EmergencyStop();
        return;
    }

    // 根据遥控器状态切换控制模式
    if (DT7UartCom.rc.s1 == UP && DT7UartCom.isConnected)
        FriSpeedControl();
    else if (DT7UartCom.rc.s1 == MID && DT7UartCom.isConnected)
        ManualMotorControl();
    else if (DT7UartCom.rc.s1 == DOWN && DT7UartCom.isConnected)
        AutoLaunchMode();

    // 电机同步控制（统一为右下摩擦轮）
    SpeedPID_RightUpFriction.Target  = SpeedPID_RightDownFriction.Target;
    SpeedPID_LeftUpFriction.Target   = SpeedPID_RightDownFriction.Target * -1;
    SpeedPID_LeftDownFriction.Target = SpeedPID_RightDownFriction.Target * -1;

    AutoLaunch.SyncYawAngle(); // 同步Yaw轴角度到PID控制器
}

/**
 * @brief 检查遥控器连接状态
 * @details 该函数用于检查遥控器是否连接，通过判断摇杆和开关的状态来确定连接状态
 */
void Dart_c::CheckControllerConnection()
{
    bool stickNeutral      = (std::abs(DT7UartCom.Coord.ch0) + std::abs(DT7UartCom.Coord.ch1) + std::abs(DT7UartCom.Coord.ch2) + std::abs(DT7UartCom.Coord.ch3)) < deadZone * 4;
    bool switchesDown      = (DT7UartCom.rc.s1 == DOWN) && (DT7UartCom.rc.s2 == DOWN);
    DT7UartCom.isConnected = stickNeutral && switchesDown;
}

/**
 * @brief 根据遥控器输入调整电机目标转速
 *
 * 控制逻辑：
 * - 当s2拨杆居中且摇杆接近零时，解除转速锁定
 * - 未锁定状态下，根据遥控器输入调整四个电机的目标转速
 * - 目标值在有效范围内
 *
 * @note 依赖全局变量 DT7UartCom、Rpm_Change_Lock 和 SpeedPID_RightDownFriction
 */
void Dart_c::FriSpeedControl()
{
    // 解除转速锁定条件
    if (DT7UartCom.rc.s2 == MID &&
        std::abs(DT7UartCom.Coord.ch1) < deadZone &&
        std::abs(DT7UartCom.Coord.ch3) < deadZone) {
        Rpm_Change_Lock = false; // 解除转速锁定
    }

    // 遥控器增加转速
    if (Rpm_Change_Lock == 0) {
        if (DT7UartCom.rc.s2 == UP) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, 500, 0, 16000);
        } else if (DT7UartCom.rc.s2 == MID && DT7UartCom.Coord.ch1 >= 500) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, 100, 0, 16000);
        } else if (DT7UartCom.rc.s2 == MID && DT7UartCom.Coord.ch3 >= 500) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, 10, 0, 16000);
        } else if (DT7UartCom.rc.s2 == DOWN) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, -500, 0, 16000);
        } else if (DT7UartCom.rc.s2 == MID && DT7UartCom.Coord.ch1 <= -500) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, -100, 0, 16000);
        } else if (DT7UartCom.rc.s2 == MID && DT7UartCom.Coord.ch3 <= -500) {
            Add_Motor_Target(&SpeedPID_RightDownFriction.Target, -10, 0, 16000);
        } else {
            return; // 如果没有匹配的条件，直接返回，不设置Rpm_Change_Lock = 1
        }
        Rpm_Change_Lock = true; // 合并rpm_change_lock = true的设置
    }
}

/**
 * @brief 手动控制电机
 * @details 该函数通过遥控器输入控制Yaw轴角度、上下丝杆和左右丝杆的运动
 */
void Dart_c::ManualMotorControl()
{
    // 遥控设置yaw轴角度
    SpeedPID_YawM6020Data.Target = Yaw_Angle; // ch0 * -14;

    // 遥控上下丝杆
    if (DT7UartCom.rc.s2 == UP && Rpm_Change_Lock == false) {
        SpeedPID_VerticalLiftM2006.Target = 6000;
    } else if (DT7UartCom.rc.s2 == MID) {
        SpeedPID_VerticalLiftM2006.Target = 0;
        Rpm_Change_Lock                   = false; // 解除锁定状态
    } else if (DT7UartCom.rc.s2 == DOWN && Rpm_Change_Lock == false) {
        SpeedPID_VerticalLiftM2006.Target = -6000;
    }

    // 遥控左右丝杆
    if ((Dart.LeftLimit != 1 && DT7UartCom.Coord.ch2 >= 0) || (Dart.RightLimit != 1 && DT7UartCom.Coord.ch2 <= 0))
        SpeedPID_SlidePlatformM2006.Target = DT7UartCom.Coord.ch2 * 14;
    else
        SpeedPID_SlidePlatformM2006.Target = 0;
}

void Dart_c::AutoLaunchMode()
{
    if (DT7UartCom.rc.s2 == UP)
        AutoLaunch.StartAutoLaunch(DT7CtrlMode); // 启动自动发射模式(15s)
    else if (DT7UartCom.rc.s2 == MID)
        AutoLaunch.StartAutoLaunch(RfSysMode); // 启动自动发射模式(裁判系统模式)
    else if (DT7UartCom.rc.s2 == DOWN)         // 紧急停止模式
        EmergencyStop();
}

/**
 * @brief 紧急停止函数
 * @details 该函数在紧急情况下调用，重置所有发射相关参数和电机目标
 */
void Dart_c::EmergencyStop()
{
    AutoLaunch.LaunchedCount = 0;    // 重置发射计数
    AutoLaunch.LaunchState   = 0;    // 重置自动发射状态
    Rpm_Change_Lock          = true; // 重置锁定状态
    AutoLaunch.MotorConfigID = 0;    // 重置电机配置ID

    // 紧急停止函数，设置所有电机目标为0
    SpeedPID_RightDownFriction.Target  = 0;
    SpeedPID_RightUpFriction.Target    = 0;
    SpeedPID_LeftDownFriction.Target   = 0;
    SpeedPID_LeftUpFriction.Target     = 0;
    SpeedPID_VerticalLiftM2006.Target  = 0;
    SpeedPID_SlidePlatformM2006.Target = 0;
    SpeedPID_YawM6020Data.Target       = 0;

    // 将Yaw角度设置为当前角度
    Yaw_Angle                        = 3830; // 重置Yaw角度
    SpeedPID_AngleSensorM3508.Target = SpeedPID_AngleSensorM3508.Current;
}

/**
 * @brief 添加电机目标值
 * @param add_value 要添加的目标值
 * @details 该函数用于增加电机的目标值，并调用限制函数确保角度在有效范围内
 */
void Dart_c::Add_Motor_Target(int *target_motor, int add_value, int min, int max)
{
    *target_motor = Limit_Value(*target_motor + add_value, min, max);
}

/**
 * @brief 限制函数
 * @param value 当前值
 * @param min 最小值
 * @param max 最大值
 * @return 限制后的值
 * @details 该函数用于限制值在指定范围内
 */
int Dart_c::Limit_Value(int value, int min, int max)
{
    if (value <= min) return min;
    if (value >= max) return max;
    return value;
}

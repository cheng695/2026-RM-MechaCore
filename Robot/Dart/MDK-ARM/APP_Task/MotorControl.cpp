#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */
FinalOut_Can_t FinalOut_Can = {0};

/**
**********************************************************************
* @brief:      	YawM6020Control: 控制Yaw轴电机的函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于对从其他任务传来的Yaw轴电机目标值进行PID处理
***********************************************************************
**/
void YawM6020Control()
{
    // if(JointMotorRunMode.YawM6020 == MotorMode)
    // {
    //     PosPID_MotorYaw.PosObservation.Measure = YawM6020Data.Angle;
    //     PosPID_MotorYaw.SpeedObservation.Measure = YawM6020Data.RPM;
    //     PosPID_MotorYaw.Compute();
    //     FinalOut_Can.YawM6020 = PosPID_MotorYaw.SpeedObservation.Output;
    // }
    // else if (JointMotorRunMode.YawM6020 == ImuMode)
    // {
    //     PosPID_ImuYaw.PosObservation.Measure = HI14UartCom.Data.Q31Angle.Yaw;
    //     PosPID_ImuYaw.SpeedObservation.Measure = HI14UartCom.Data.Q31Gyr.Yaw;
    //     PosPID_ImuYaw.Compute();
    //     FinalOut_Can.YawM6020 = PosPID_ImuYaw.SpeedObservation.Output;
    //     // if(Report.HI14.Disconnection == true)
    //     // FinalOut_Can.YawM6020 = 0;
    // }
}

/**
**********************************************************************
* @brief:      	PitchM3508Control: 控制Pitch轴电机的函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于对从其他任务传来的Pitch轴电机目标值进行PID处理
***********************************************************************
**/
void PitchM3508Control()
{
    // if(JointMotorRunMode.PitchM3508 == MotorMode)
    // {
    //     PosPID_MotorPitch.PosObservation.Measure = PitchM3508Data.Angle;
    //     PosPID_MotorPitch.SpeedObservation.Measure = PitchM3508Data.RPM;
    //     PosPID_MotorPitch.Compute();
    //     FinalOut_Can.PitchM3508 = PosPID_MotorPitch.SpeedObservation.Output;
    // }
    // else if (JointMotorRunMode.PitchM3508 == ImuMode)
    // {
    //     PosPID_ImuPitch.PosObservation.Measure = HI14UartCom.Data.Q31Angle.Pitch;
    //     PosPID_ImuPitch.SpeedObservation.Measure = PitchM3508Data.RPM;
    //     PosPID_ImuPitch.Compute();
    //     FinalOut_Can.PitchM3508 = PosPID_ImuPitch.SpeedObservation.Output;
    //     // if(Report.HI14.Disconnection == true)
    //     // FinalOut_Can.PitchM3508 = 0;
    // }
}

/**
**********************************************************************
* @brief:      	DebounceAngleSensor: 处理AngleSensorM3508.Angle的消抖
* @param[in]: 	float raw_angle: 原始角度值
* @retval:      float: 平滑后的角度值
* @details:    	该函数使用滑动平均法对角度值进行平滑处理
***********************************************************************
**/
float DebounceAngleSensor(float raw_angle) {
    static float buffer[100] = {0}; // 滑动窗口缓冲区
    static int index = 0;         // 当前索引
    static float sum = 0;         // 窗口内值的总和

    // 更新滑动窗口
    sum -= buffer[index]; // 移除旧值
    buffer[index] = raw_angle; // 添加新值
    sum += raw_angle; // 更新总和

    // 移动索引
    index = (index + 1) % 100;

    // 返回平均值
    return sum / 100.0f;
}

/**
**********************************************************************
* @brief:      	FrM3508Control: 控制摩擦轮电机的函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于对从其他任务传来的摩擦轮电机目标值进行PID处理
***********************************************************************
**/
// Init.cpp,PID.hpp,MotorControl.cpp
void FrM3508Control()
{
    // 对AngleSensorM3508.Angle进行消抖处理
    float smoothed_angle = DebounceAngleSensor(AngleSensorM3508.Angle);
    SpeedPID_AngleSensorM3508.Current = smoothed_angle;

    SpeedPID_RightDownFriction.Current  = RightDownM3508Data.RPM;
    SpeedPID_RightUpFriction.Current    = RightUpM3508Data.RPM;
    SpeedPID_LeftUpFriction.Current     = LeftUpM3508Data.RPM;
    SpeedPID_LeftDownFriction.Current   = LeftDownM3508Data.RPM;
    SpeedPID_YawM6020Data.Current       = AngleSensorM3508.RPM;
    SpeedPID_SlidePlatformM2006.Current = SlidePlatformM2006.RPM;
    SpeedPID_VerticalLiftM2006.Current  = VerticalLiftM2006.RPM;
    //SpeedPID_AngleSensorM3508.Current   = AngleSensorM3508.Angle;

    SpeedPID_RightDownFriction.Compute();
    SpeedPID_RightUpFriction.Compute();
    SpeedPID_LeftUpFriction.Compute();
    SpeedPID_LeftDownFriction.Compute();
    // SpeedPID_YawM6020Data.Compute();
    SpeedPID_SlidePlatformM2006.Compute();
    SpeedPID_VerticalLiftM2006.Compute();
    SpeedPID_AngleSensorM3508.Compute();

    SpeedPID_YawM6020Data.Target = SpeedPID_AngleSensorM3508.Final_Output;
    //SpeedPID_YawM6020Data.Target = TdFilter(&TD_AngleSensor, SpeedPID_AngleSensorM3508.Final_Output);
    SpeedPID_YawM6020Data.Compute();

    FinalOut_Can.RightDownFriction  = SpeedPID_RightDownFriction.Final_Output;
    FinalOut_Can.RightUpFriction    = SpeedPID_RightUpFriction.Final_Output;
    FinalOut_Can.LeftUpFriction     = SpeedPID_LeftUpFriction.Final_Output;
    FinalOut_Can.LeftDownFriction   = SpeedPID_LeftDownFriction.Final_Output;
    FinalOut_Can.YawMotor6020       = SpeedPID_YawM6020Data.Final_Output;
    FinalOut_Can.SlidePlatformM2006 = SpeedPID_SlidePlatformM2006.Final_Output;
    FinalOut_Can.VerticalLiftM2006  = SpeedPID_VerticalLiftM2006.Final_Output;
    // FinalOut_Can.AngleSensorM3508 = SpeedPID_AngleSensorM3508.Final_Output;

    // if(JointMotorRunMode.DialM3508 == PosMode)
    // {
    //     PosPID_Dial.PosObservation.Measure = DailM3508Data.Angle;
    //     PosPID_Dial.SpeedObservation.Measure = DailM3508Data.RPM;
    //     PosPID_Dial.Compute();
    //     FinalOut_Can.DialM3508 = PosPID_Dial.SpeedObservation.Output;
    // }
    // else if(JointMotorRunMode.DialM3508 == SpeedMode)
    // {
    //     SpeedPID_DialM3508.Current = DailM3508Data.RPM;
    //     SpeedPID_DialM3508.Compute();
    //     FinalOut_Can.DialM3508 = SpeedPID_DialM3508.Final_Output;
    // }
}

/* Private application code --------------------------------------------------*/
void PIDControl(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        taskDISABLE_INTERRUPTS();
        // YawM6020Control();
        // PitchM3508Control();
        FrM3508Control();
        taskENABLE_INTERRUPTS();
        RmMotorSendCanID0X1FFData(&hcan1, FinalOut_Can.YawMotor6020, FinalOut_Can.SlidePlatformM2006, FinalOut_Can.VerticalLiftM2006, 0);

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        RmMotorSendCanID0X200Data(&hcan1, FinalOut_Can.RightDownFriction, FinalOut_Can.RightUpFriction, FinalOut_Can.LeftUpFriction, FinalOut_Can.LeftDownFriction);
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

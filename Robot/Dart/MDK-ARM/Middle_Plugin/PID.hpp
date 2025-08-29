#ifndef __PID_Hpp
#define __PID_Hpp
/* C++代码的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
typedef class PID_Speed
{
public:
    int   Final_OutputLimit;
    int   Final_Output;

    int   Error;
    int   Error_Last;

    float KP_GainCoefficient;
    float KP_GainValue;
    int   KP_GainMiniL;
    int   KP_GainMaxL;
    float KP_Out;
    float KP;

    float KI_OutLimit;
    float KI_Itrerm;
    float KI_Time;
    float KI_Out;
    float KI;

    float KD_Out;
    float KD;

    int   Target;
    int   Current;
    int   Current_Last;

    void Compute();
}PID_Speed_Temp;
// extern PID_Speed_Temp SpeedPID_UpFriction;
// extern PID_Speed_Temp SpeedPID_LeftDownFriction;
// extern PID_Speed_Temp SpeedPID_RightDownFriction;
// extern PID_Speed_Temp SpeedPID_LeftFriction;
// extern PID_Speed_Temp SpeedPID_RightFriction;
// extern PID_Speed_Temp SpeedPID_DialM3508;

extern PID_Speed_Temp SpeedPID_RightDownFriction;
extern PID_Speed_Temp SpeedPID_RightUpFriction;
extern PID_Speed_Temp SpeedPID_LeftUpFriction;
extern PID_Speed_Temp SpeedPID_LeftDownFriction;
extern PID_Speed_Temp SpeedPID_YawM6020Data;
extern PID_Speed_Temp SpeedPID_SlidePlatformM2006;
extern PID_Speed_Temp SpeedPID_VerticalLiftM2006;
extern PID_Speed_Temp SpeedPID_AngleSensorM3508;

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
typedef class DoublePosPID
{
public:
/*  =========================== PID的优化功能以及属性 ===========================  */ 
    struct
    {
        unsigned char ImprovementMode;
        char Counter_Period;
        char Count_Period;
        int Final_OutputLimit;
        int Half_TurnRange;
    }attribute;
/*  =========================== PID的角度环参数 ===========================  */   
    struct
    {
        float KI_GainCoefficient;
        float Output_MaxLimit;
        int KI_TimeMiniL;
        int KI_TimeMaxL;
        int KI_Saturate;
        int KI_OutLimit;
        char  KD_Count;
        double Target;
        float KP;
        float KI;
        float KD;
        float KR;
    }PosParameter;
    struct
    {
        float FeedBack_Out;
        int FeedBack_Error;
        float Output;
        int Target_Last;
        int Measure;
        int Measure_Last[5];
        int Error;
        int Error_Last;
        int KD_Error;
        float ITime;
        float Pout;
        float Iout; 
        float Dout;
        float ITerm;
    }PosObservation;
/*  =========================== PID的速度环参数 ===========================  */   
    struct
    {
        float KI_GainCoefficient;
        float Output_MaxLimit;
        int KI_TimeMiniL;
        int KI_TimeMaxL;
        int KI_Saturate;
        int KI_OutLimit;
        char  KD_Count;
        float KP;
        float KI;
        float KD;
        float KR;
    }SpeedParameter;
    struct
    {
        float Output;
        float Target;
        int Measure;
        int Measure_Last;
        float Error;
        float Error_Last;
        float ITime;
        float Pout;
        float Iout; 
        float Dout;
        float ITerm;
    }SpeedObservation;
/*  =========================== PID的滤波器设置和堵转保护 ===========================  */  
    struct
    {     
        float v1,v2; 
        float R;           
        float H;           
    }TD;
    struct PID
    {
        int Count;
        float Sensitivity;
        int Warn;
        int Max;
        char Flag;
    }BlockedParameter;
    enum  
    {
        NONE = 0x00, 
        FilterOpen = 0x01,
        BlockedProtection = 0x02,
        ZeroDispose = 0x04,
    }PIDImprovement;
/*  =========================== 以上 ===========================  */ 
    void Compute();
}DoublePosPID_Temp;
extern DoublePosPID_Temp PosPID_MotorYaw;
extern DoublePosPID_Temp PosPID_ImuYaw;
extern DoublePosPID_Temp PosPID_MotorPitch;
extern DoublePosPID_Temp PosPID_ImuPitch;
extern DoublePosPID_Temp PosPID_Dial;



/* USER CODE END Includes */

/* C++代码的声明 ----------------------------------------------------------*/
#endif

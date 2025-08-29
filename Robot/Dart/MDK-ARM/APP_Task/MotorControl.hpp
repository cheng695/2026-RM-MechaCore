#ifndef __PIDControl_Hpp
#define __PIDControl_Hpp

enum JointMotorRunTypdef
{
    MotorMode = 1
    ,ImuMode = 2
    ,PosMode = 3
    ,SpeedMode = 4
};

typedef struct 
{
    JointMotorRunTypdef YawM6020;
    JointMotorRunTypdef PitchM3508;
    JointMotorRunTypdef DialM3508;
}JointMotorRunMode_t;
extern JointMotorRunMode_t JointMotorRunMode;

typedef struct 
{
    // int YawM6020;
    // int PitchM3508;
    // int UpFriction;
    // int RightDownFriction;
    // int LeftDownFriction;
    // int DialM3508;
    // int LeftFriction;
    // int RightFriction;
    int RightDownFriction;
    int RightUpFriction;
    int LeftUpFriction;
    int LeftDownFriction;
    int YawMotor6020;
    int SlidePlatformM2006;
    int VerticalLiftM2006;
    int AngleSensorM3508;
}FinalOut_Can_t;
extern FinalOut_Can_t FinalOut_Can;



#endif

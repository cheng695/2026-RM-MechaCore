/**
 * @file RmMotor.hpp
 * @brief RM电机CAN通信相关函数声明
 */

#ifndef __DJIMotor_Hpp
#define __DJIMotor_Hpp
#include "main.h"

static CAN_TxHeaderTypeDef  RmCanTxMessage;
static uint8_t              RmCanTxData[8];

typedef struct
{
    int32_t Angle;
    int16_t RPM;
    int16_t Current;
    uint8_t Temperature;
}RmMotorMeasure_t;


// extern RmMotorMeasure_t UpFrM3508Data;
// extern RmMotorMeasure_t LeftDdownFrM3508Data;
// extern RmMotorMeasure_t RightDownFrM3508Data;
// extern RmMotorMeasure_t LeftFrM3508Data;
// extern RmMotorMeasure_t RightFrM3508Data;
// extern RmMotorMeasure_t DailM3508Data;
// extern RmMotorMeasure_t YawM6020Data;
// extern RmMotorMeasure_t PitchM3508Data;

extern RmMotorMeasure_t RightDownM3508Data;
extern RmMotorMeasure_t RightUpM3508Data;
extern RmMotorMeasure_t LeftUpM3508Data;
extern RmMotorMeasure_t LeftDownM3508Data;
extern RmMotorMeasure_t YawM6020Data;
extern RmMotorMeasure_t SlidePlatformM2006;
extern RmMotorMeasure_t VerticalLiftM2006;
extern RmMotorMeasure_t AngleSensorM3508;

void RmMotorGetCanData(void);
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);



#endif

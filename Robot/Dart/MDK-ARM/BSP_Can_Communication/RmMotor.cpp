/**
 * @file RmMotor.cpp
 * @brief 机甲大师电机CAN通信相关函数
 */

#include "InHpp.hpp"
#include "MotorOfflineDetector.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
// RmMotorMeasure_t UpFrM3508Data        = {0};
// RmMotorMeasure_t LeftDdownFrM3508Data = {0};
// RmMotorMeasure_t RightDownFrM3508Data = {0};
// RmMotorMeasure_t LeftFrM3508Data      = {0};
// RmMotorMeasure_t RightFrM3508Data     = {0};
// RmMotorMeasure_t DailM3508Data        = {0};
// RmMotorMeasure_t YawM6020Data         = {0};
// RmMotorMeasure_t PitchM3508Data       = {0};
RmMotorMeasure_t RightDownM3508Data = {0};
RmMotorMeasure_t RightUpM3508Data   = {0};
RmMotorMeasure_t LeftUpM3508Data    = {0};
RmMotorMeasure_t LeftDownM3508Data  = {0};
RmMotorMeasure_t YawM6020Data       = {0};
RmMotorMeasure_t SlidePlatformM2006 = {0};
RmMotorMeasure_t VerticalLiftM2006  = {0};
RmMotorMeasure_t AngleSensorM3508   = {0};

uint8_t motor_connection_status[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // 初始化为连接状态

/*  =========================== 进程变量的初始化 =========================== */

/*  =========================== 函数的声明 =========================== */
void RmMotorGetCanData();
void RMmotorClockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[]);
void RMmotorCounterclockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[]);
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);





/**
 * @brief 获取CAN数据
 * @details 该函数解析接收到的CAN数据并更新电机测量数据
 */
void RmMotorGetCanData()
{
    switch (Can_RX.Header.StdId) {
        case 0x201: // 右下摩擦轮
            RMmotorClockwiseGetData(&RightDownM3508Data, Can_RX.Data);
            RightDownM3508Data.RPM = TdFilter(&TD_RightDownFriction, RightDownM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_1);
            break;
            
        case 0x202: // 右上摩擦轮
            RMmotorClockwiseGetData(&RightUpM3508Data, Can_RX.Data);
            RightUpM3508Data.RPM = TdFilter(&TD_RightUpFriction, RightUpM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_2);
            break;
            
        case 0x203: // 左上摩擦轮
            RMmotorClockwiseGetData(&LeftUpM3508Data, Can_RX.Data);
            LeftUpM3508Data.RPM = TdFilter(&TD_LeftUpFriction, LeftUpM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_3);
            break;
            
        case 0x204: // 左下摩擦轮
            RMmotorClockwiseGetData(&LeftDownM3508Data, Can_RX.Data);
            LeftDownM3508Data.RPM = TdFilter(&TD_LeftDownFriction, LeftDownM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_4);
            break;
            
        case 0x205: // Yaw轴电机
            RMmotorClockwiseGetData(&YawM6020Data, Can_RX.Data);
            // YawM6020Data.Angle = MyTool::Round_ZeroSetup(YawM6020Data.Angle, 3830, 8192);
            // YawM6020Data.Angle = MyTool::Round_MileageWithLimit(YawM6020Data.Angle, 8192, 2, Mileage.YawM6020.CircleCount, Mileage.YawM6020.Temp);
            //  Report.YawM6020.Count_Paparazzi++;
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_5);
            break;
            
        case 0x206: // 左右丝杆电机
            RMmotorClockwiseGetData(&SlidePlatformM2006, Can_RX.Data);
            //RMmotorCounterclockwiseGetData(&SlidePlatformM2006, Can_RX.Data);
            // PitchM3508Data.Angle = MyTool::Round_Mileage(PitchM3508Data.Angle, 8192, Mileage.PitchM3508.CircleCount, Mileage.PitchM3508.Temp);
            //  Report.PitchM3508.Count_Paparazzi++;
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_6);
            break;
            
        case 0x207: // 上下丝杆推进器电机
            RMmotorClockwiseGetData(&VerticalLiftM2006, Can_RX.Data);
            // LeftFrM3508Data.RPM = TdFilter(&TD_LeftFrictionWheel,LeftFrM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_7);
            break;
            
        case 0x208: // 编码器电机
            RMmotorClockwiseGetData(&AngleSensorM3508, Can_RX.Data);
            //AngleSensorM3508.Angle = TdFilter(&TD_AngleSensor, AngleSensorM3508.Angle);
            // RightFrM3508Data.RPM = TdFilter(&TD_RightFrictionWheel,RightFrM3508Data.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_8);
            break;
    }

}

/**
 * @brief 解析电机顺时针旋转的数据
 * @param ptr 指向RmMotorMeasure_t结构体的指针
 * @param data 包含电机数据的字节数组
 * @details 该函数用于解析CAN数据并让其极性为顺时针
 */
void RMmotorClockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[])
{
    (ptr)->Angle       = (uint16_t)(((data)[0] << 8 | (data)[1]));
    (ptr)->RPM         = (uint16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->Current     = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->Temperature = (data)[6];
}

/**
 * @brief 解析电机逆时针旋转的数据
 * @param ptr 指向RmMotorMeasure_t结构体的指针
 * @param data 包含电机数据的字节数组
 * @details 该函数用于解析CAN数据并让其极性为逆时针
 */
void RMmotorCounterclockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[])
{
    (ptr)->Angle       = (uint16_t)(((data)[0] << 8 | (data)[1]));
    (ptr)->Angle       = 8192 - (ptr)->Angle;
    (ptr)->Angle       = ((ptr)->Angle >= 8192) ? 0 : (ptr)->Angle;
    (ptr)->RPM         = 0 - (uint16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->Current     = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->Temperature = (data)[6];
}

/**
 * @brief 发送CanID为0x1FF的数据帧
 * @param hcan CAN句柄
 * @param motor1 第一个电机的目标值
 * @param motor2 第二个电机的目标值
 * @param motor3 第三个电机的目标值
 * @param motor4 第四个电机的目标值
 * @details 该函数用于发送CAN数据帧
 */
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    RmCanTxMessage.StdId = 0x1FF;
    RmCanTxMessage.IDE   = CAN_ID_STD;
    RmCanTxMessage.RTR   = CAN_RTR_DATA;
    RmCanTxMessage.DLC   = 0x08;
    RmCanTxData[0]       = motor1 >> 8;
    RmCanTxData[1]       = motor1;
    RmCanTxData[2]       = motor2 >> 8;
    RmCanTxData[3]       = motor2;
    RmCanTxData[4]       = motor3 >> 8;
    RmCanTxData[5]       = motor3;
    RmCanTxData[6]       = motor4 >> 8;
    RmCanTxData[7]       = motor4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0) {
        if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }
}

/**
 * @brief 发送CanID为0x200的数据帧
 * @param hcan CAN句柄
 * @param motor1 第一个电机的目标值
 * @param motor2 第二个电机的目标值
 * @param motor3 第三个电机的目标值
 * @param motor4 第四个电机的目标值
 * @details 该函数用于发送CAN数据帧
 */
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    RmCanTxMessage.StdId = 0x200;
    RmCanTxMessage.IDE   = CAN_ID_STD;
    RmCanTxMessage.RTR   = CAN_RTR_DATA;
    RmCanTxMessage.DLC   = 0x08;
    RmCanTxData[0]       = motor1 >> 8;
    RmCanTxData[1]       = motor1;
    RmCanTxData[2]       = motor2 >> 8;
    RmCanTxData[3]       = motor2;
    RmCanTxData[4]       = motor3 >> 8;
    RmCanTxData[5]       = motor3;
    RmCanTxData[6]       = motor4 >> 8;
    RmCanTxData[7]       = motor4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0) {
        if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }
}

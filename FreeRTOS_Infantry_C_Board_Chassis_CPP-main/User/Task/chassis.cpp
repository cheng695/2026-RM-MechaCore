/**
 * @file chassis.cpp
 * @author XMX
 * @brief 底盘运行任务
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis.hpp"
#include "stdint.h"  //stdint.h需要在cmsis_armclang.h前面才能过编译
#include <cmsis_armclang.h>
#include <cstdint>
#include "arm_math.h"
#include "clamp.hpp"
#include "cmsis_os2.h"
#include "simple_math.hpp"
#include "state_machine.hpp"
#include "uart.hpp"
#include "variables.hpp"
#include "power_limit.hpp"
#include "UI_Queue.hpp"
#include "darw_static.hpp"

/*  =========================== 常量定义 ===========================  */

constexpr auto kRemoteDeadBand = 10;              //遥控器死区
constexpr auto kFollowCoefficient = 75.0f;       //底盘跟随系数
constexpr auto kVisionFollowCoefficient = 25.0f;  //视觉模式下的底盘跟随系数
constexpr auto kReverseOperateCoeficient = 2;     //按键反按时缓启动的变化倍数

/*  =========================== 函数声明 ===========================  */

static void ORE_Solve();
void HaltFunction();
void SubMode00Function();
void SubMode11Function();
void SubMode12Function();
void SubMode13Function();
void SubMode21Function();
void SubMode22Function();
void SubMode23Function();
void SubMode31Function();
void SubMode32Function();
void SubMode33Function();
void TimeStampClear();
void SubStateUpdate();
int SmoothChange(int current_value, int target_value, int step);
void AllocateVelocityComponents(float* value1, float* value2, float* value3, float max_value);
void ChassisFollow();
void dead_detect();

/*  =========================== 变量定义 ===========================  */

//进入子模式时间戳
uint32_t enter_mode_11_timestamp;
uint32_t enter_mode_12_timestamp;
uint32_t enter_mode_13_timestamp;
uint32_t enter_mode_21_timestamp;
uint32_t enter_mode_22_timestamp;
uint32_t enter_mode_23_timestamp;
uint32_t enter_mode_31_timestamp;
uint32_t enter_mode_32_timestamp;
uint32_t enter_mode_33_timestamp;

//子模式时间戳
uint32_t mode_11_timestamp;
uint32_t mode_12_timestamp;
uint32_t mode_13_timestamp;
uint32_t mode_21_timestamp;
uint32_t mode_22_timestamp;
uint32_t mode_23_timestamp;
uint32_t mode_31_timestamp;
uint32_t mode_32_timestamp;
uint32_t mode_33_timestamp;

constexpr auto smooth_change_coefficient = 20;  //平滑函数步进值

float theta;  //底盘跟随使用的夹角

//速度分量
float vx;
float vy;
float vw;

//速度分量计算临时值
float temp_vx;
float temp_vy;

//电机目标速度值
float target_rpm_201;
float target_rpm_202;
float target_rpm_203;
float target_rpm_204;

//电机最大转速
float max_rpm = 3000.0f;//2000
float Spannin = 1500.0f;
float max_rpm_save;

bool adjust_max_rpm_flag;  //最大转速调整标志位

//按键锁存

bool Q_latch;
bool Z_latch;
bool X_latch;
bool C_latch;
bool F_latch;
bool V_latch;
bool CTRL_latch;


bool rotate_flag;         //小陀螺标志位
bool follow_flag = true;  //底盘跟随标志位

bool dead_latch;  //死亡状态锁存
uint16_t a;
/// @brief 底盘任务
/// @param argument
void ChassisTask(void* argument) {
    for (;;) {

        ORE_Solve();  //串口错误清除

        SubStateUpdate();  //状态机子状态更新

        //状态机
        switch (state_machine.main_state_) {
            case kMainStateNone:
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_203.is_enable_ = false;
                motor_204.is_enable_ = false;
                state_machine.HandleEvent(kEventEnterOperate);
                break;

            case kOperate:  //运行模式

                switch (state_machine.sub_state_) {
                    case kSubStateNone:
                        motor_201.is_enable_ = false;
                        motor_202.is_enable_ = false;
                        motor_203.is_enable_ = false;
                        motor_204.is_enable_ = false;
                        state_machine.HandleEvent(kEventEnterHalt);
                        break;

                    case kSubMode00:  //切换到接收机断联模式
                        SubMode00Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    case kSubMode11:  //切到维护模式
                        state_machine.HandleEvent(kEventEnterHalt);
                        SubMode11Function();
                        break;

                    case kSubMode12:  //遥控器小陀螺顺时针模式
                        SubMode12Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    case kSubMode13:  //遥控器小陀螺逆时针模式
                        SubMode13Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    case kSubMode21:  //底盘不跟随模式
                        SubMode21Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

//                    case kSubMode22:  //底盘移动时跟随模式
//                        SubMode22Function();
//                        motor_201.is_enable_ = true;
//                        motor_202.is_enable_ = true;
//                        motor_203.is_enable_ = true;
//                        motor_204.is_enable_ = true;
//                        break;

                    case kSubMode23:  //底盘跟随模式
                        SubMode23Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    case kSubMode31:
                    case kSubMode32:  //遥控射击模式
                        SubMode32Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    case kSubMode33:  //键鼠模式
                        SubMode33Function();
                        motor_201.is_enable_ = true;
                        motor_202.is_enable_ = true;
                        motor_203.is_enable_ = true;
                        motor_204.is_enable_ = true;
                        break;

                    default:
                        break;
                }

                break;  // kOperate

            case kHalt:  //维护模式
                motor_201.is_enable_ = false;
                motor_202.is_enable_ = false;
                motor_203.is_enable_ = false;
                motor_204.is_enable_ = false;
                HaltFunction();

                switch (state_machine.sub_state_) {
                    case kSubMode12:  //遥控器小陀螺顺时针模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode13:  //遥控器小陀螺逆时针模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode21:
                    case kSubMode22:
                    case kSubMode23:  //转场模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode31:
                    case kSubMode32:  //遥控射击模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    case kSubMode33:  //键鼠模式
                        state_machine.HandleEvent(kEventEnterOperate);
                        break;

                    default:
                        break;
                }

                break;

            default:
                break;

        }  // state machines

        //轮速限幅
        target_rpm_201 = clamp(target_rpm_201, -max_rpm, max_rpm);
        target_rpm_202 = clamp(target_rpm_202, -max_rpm, max_rpm);
        target_rpm_203 = clamp(target_rpm_203, -max_rpm, max_rpm);
        target_rpm_204 = clamp(target_rpm_204, -max_rpm, max_rpm);

        TimeStampClear();  //时间戳清除
        dead_detect();     //死亡关小陀螺和跟随

        osDelay(1);  //延时
    }
}

/// @brief 遥控器目标值处理
/// @return 是否移动
static bool RemoteTargetHandle() {
    bool is_move = false;
    if (dr16.remote_.ch2_ < 1024 - kRemoteDeadBand || dr16.remote_.ch2_ > 1024 + kRemoteDeadBand) {
        temp_vx = -((dr16.remote_.ch2_ - 1024) / 660.0f * max_rpm);
        is_move = true;
    } else {
        temp_vx = 0;
    }
    if (dr16.remote_.ch3_ < 1024 - kRemoteDeadBand || dr16.remote_.ch3_ > 1024 + kRemoteDeadBand) {
        temp_vy = -((dr16.remote_.ch3_ - 1024) / 660.0f * max_rpm);
        is_move = true;
    } else {
        temp_vy = 0;
    }
    return is_move;
}

/// @brief 旋转矩阵计算
/// @param theta 云台底盘夹角
static void RotateMatrixCompute(float theta) {
      auto radian = deg2rad(theta);
    vx = (temp_vx * arm_cos_f32(radian)) + (-temp_vy * arm_sin_f32(radian));
    vy = (temp_vx * arm_sin_f32(radian)) + (temp_vy * arm_cos_f32(radian));
}

/// @brief 轮速计算
static void WheelsRpmCompute() {
   target_rpm_201 = vx - vy+vw ;
   target_rpm_202 = -vx + vy +vw;
   target_rpm_203 = -vx - vy +vw;
   target_rpm_204 = vx + vy+vw;
}

/// @brief 子模式函数
void HaltFunction() {}
void SubMode00Function() {
    target_rpm_201 = 0;
    target_rpm_202 = 0;
    target_rpm_203 = 0;
    target_rpm_204 = 0;
}
void SubMode11Function() {}
void SubMode12Function() {
    //获取系统时间戳，单位为ms
    auto current_timestamp = HAL_GetTick();
    if (enter_mode_12_timestamp == 0) {  //刚进入该模式
        enter_mode_12_timestamp = current_timestamp;
        vw = 0;  //车体停转
    } else if (current_timestamp - enter_mode_12_timestamp > 1000) {
        // vw = SmoothChange(vw, max_rpm, smooth_change_coefficient);
    }
    vw = SmoothChange(vw, Spannin, smooth_change_coefficient);
	
    RemoteTargetHandle();                                          //摇杆值处理
    AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
    //RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
    WheelsRpmCompute();                                            //轮速计算
}
void SubMode13Function() {
    //获取系统时间戳，单位为ms
    auto current_timestamp = HAL_GetTick();
    if (enter_mode_13_timestamp == 0) {  //刚进入该模式
        enter_mode_13_timestamp = current_timestamp;
        vw = 0;  //车体停转
    } else if (current_timestamp - enter_mode_13_timestamp > 1000) {
        // vw = SmoothChange(vw, -max_rpm, smooth_change_coefficient);
    }
    vw = SmoothChange(vw, -Spannin, smooth_change_coefficient);
    RemoteTargetHandle();                                          //摇杆值处理
    AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
    RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
    WheelsRpmCompute();                                            //轮速计算
}
void SubMode21Function() {
    vw = 0;
    RemoteTargetHandle();                                          //摇杆值处理
    AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
    RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
    WheelsRpmCompute();                                            //轮速计算
}
//void SubMode22Function() {
//    //若摇杆为有效值则进行移动和底盘跟随
//    if (RemoteTargetHandle()) {
//        // if (comm.theta > 180) {
//        //         theta = comm.theta - 360.0f;
//        //     } else {
//        //         theta = comm.theta;
//        //     }
//        //     vw = -theta * kFollowCoefficient;
//        // } else {
//        //     vw = 0;
//        ChassisFollow();
//    }
//    AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
//    RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
//    WheelsRpmCompute();                                            //轮速计算
//}
void SubMode23Function() {
    RemoteTargetHandle();                                          //摇杆值处理
    AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
    RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
    // if (comm.theta > 180) {
    //     theta = comm.theta - 360.0f;
    // } else {
    //     theta = comm.theta;
    // }
    // vw = -theta * kFollowCoefficient;
    ChassisFollow();
    WheelsRpmCompute();  //轮速计算
}
void SubMode31Function() {}
void SubMode32Function() {
    target_rpm_201 = 0;
    target_rpm_202 = 0;
    target_rpm_203 = 0;
    target_rpm_204 = 0;
}
void SubMode33Function() {

    //按键平移
    if (dr16.KeyBoard_.key_.W_key) {
        temp_vy = SmoothChange(temp_vy, -max_rpm, smooth_change_coefficient);
    } else if (dr16.KeyBoard_.key_.S_key) {
        temp_vy = SmoothChange(temp_vy, max_rpm, smooth_change_coefficient);
    } else {
        temp_vy = SmoothChange(temp_vy, 0, smooth_change_coefficient);
    }
		

    if (dr16.KeyBoard_.key_.A_key) {
        temp_vx = SmoothChange(temp_vx, max_rpm, smooth_change_coefficient);
    } else if (dr16.KeyBoard_.key_.D_key) {
        temp_vx = SmoothChange(temp_vx, -max_rpm, smooth_change_coefficient);
    } else {
        temp_vx = SmoothChange(temp_vx, 0, smooth_change_coefficient);
    }

    //小陀螺切换
    if (dr16.KeyBoard_.key_.Q_key && Q_latch == false) {
        if (rotate_flag) {
            rotate_flag = false;
        } else {
            rotate_flag = true;
        }
        Q_latch = true;
    } else if (dr16.KeyBoard_.key_.Q_key == 0) {
        Q_latch = false;
		//ChassisFollow();
    }

    //增加最大转速
	
    if (dr16.KeyBoard_.key_.F_key && F_latch == false) {
        //power_care.chassis_max_power += 5;
		PowerControl.Wheel_PowerData.MAXPower += 5; 
		
		a+=5;
				capacity.SetMaxChargePower(60+a);

        adjust_max_rpm_flag = true;
        F_latch = true; 
    } else if (dr16.KeyBoard_.key_.F_key == 0) {
        F_latch = false;
    }

    //降低最大转速
    if (dr16.KeyBoard_.key_.V_key && V_latch == false) {
		//power_care.chassis_max_power -= 5;
		PowerControl.Wheel_PowerData.MAXPower -= 5;
		a-=5;
				capacity.SetMaxChargePower(60+a);

        adjust_max_rpm_flag = true;
        V_latch = true;
    } else if (dr16.KeyBoard_.key_.V_key == 0) {
        V_latch = false;
    }

    //重新恢复默认转速
    if (dr16.KeyBoard_.key_.CTRL_key && CTRL_latch == false) {
        adjust_max_rpm_flag = false;
		UI::UI_send_queue.is_Delete_all = true;
		UI::Static::UI_static.Init();
        CTRL_latch = true;
    } else if (dr16.KeyBoard_.key_.CTRL_key == 0) {
        CTRL_latch = false; 
    }
   //底盘跟随
   // if (dr16.KeyBoard_.key_.Z_key && C_latch == false) {
       // if (follow_flag == false) {
            follow_flag = true;
//        } else {
//            follow_flag = false;
//        }
//        C_latch = true;
//    } else if (dr16.KeyBoard_.key_.Z_key == 0) {
//        C_latch = false;
//    }

    //判断底盘小陀螺或跟随
    if (rotate_flag) {
        vw = SmoothChange(vw, max_rpm, smooth_change_coefficient);
    }
    //底盘跟随
    // else {
    //     //  ====================== adjust in 2024/7/26 ======================
    //     if (comm.vision_is_use) {
    //         if (comm.theta > 180) {
    //             theta = comm.theta - 360.0f;
    //         } else {
    //             theta = comm.theta;
    //         }
    //         vw = -theta * kVisionFollowCoefficient;
    //     } else {
    //         if (comm.theta > 180) {
    //             theta = comm.theta - 360.0f;
    //         } else {
    //             theta = comm.theta;
    //         }
    //         vw = -theta * kFollowCoefficient;
    //     }
    //     //  =================================================================
    // }
    else if (follow_flag) {
        ChassisFollow();
    } else {
        vw = 0;
    }

    //超速
    if (dr16.KeyBoard_.key_.SHIFT_key) {
        if (max_rpm_save == 0) {
            max_rpm_save = max_rpm;
        } else {
            max_rpm = 6000;
						//power_care.chassis_max_power = 120;
						PowerControl.Wheel_PowerData.MAXPower = 120;
        }
    } else {
        if (dr16.KeyBoard_.key_.SHIFT_key == 0 && max_rpm_save != 0) {
            max_rpm = max_rpm_save;
            max_rpm_save = 0;
			//power_care.chassis_max_power =  referee.robot_status.chassis_power_limit;
			PowerControl.Wheel_PowerData.MAXPower = referee.robot_status.chassis_power_limit - 10.f;
        }
    }

    if (dr16.KeyBoard_.key_.R_key) {
        //获取系统时间戳，单位为ms
        auto current_timestamp = HAL_GetTick();
        if (enter_mode_33_timestamp == 0) {  //刚进入该模式
            enter_mode_33_timestamp = current_timestamp;
        } else if (current_timestamp - enter_mode_33_timestamp > 900) {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    } else if (dr16.KeyBoard_.key_.R_key == 0) {
        enter_mode_33_timestamp = 0;
    }

    //AllocateVelocityComponents(&temp_vx, &temp_vy, &vw, max_rpm);  //速度分量分配
    RotateMatrixCompute(comm.theta);                               //旋转矩阵计算
    WheelsRpmCompute();                                            //轮速计算
}

/// @brief 串口ORE清除
static void ORE_Solve() {
    //解决串口ORE问题
    if (__HAL_UART_GET_FLAG(kCommUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kCommUart);  //清除ORE位

        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    }

    //解决串口ORE问题
    if (__HAL_UART_GET_FLAG(kRefereeUart, UART_FLAG_ORE) != RESET) {

        __HAL_UART_CLEAR_OREFLAG(kRefereeUart);  //清除ORE位

        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kRefereeUart, referee_rx_buf, kRefereeRecvSize);
    }
}

/// @brief 子模式时间戳清除
void TimeStampClear() {
    if (state_machine.sub_state_ != kSubMode11) {
        enter_mode_11_timestamp = 0;
        mode_11_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode12) {
        enter_mode_12_timestamp = 0;
        mode_12_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode13) {
        enter_mode_13_timestamp = 0;
        mode_13_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode21) {
        enter_mode_21_timestamp = 0;
        mode_21_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode22) {
        enter_mode_22_timestamp = 0;
        mode_22_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode23) {
        enter_mode_23_timestamp = 0;
        mode_23_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode31) {
        enter_mode_31_timestamp = 0;
        mode_31_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode32) {
        enter_mode_32_timestamp = 0;
        mode_32_timestamp = 0;
    }
    if (state_machine.sub_state_ != kSubMode33) {
        enter_mode_33_timestamp = 0;
        mode_33_timestamp = 0;
    }
}

/// @brief 子模式更新
void SubStateUpdate() {
    switch (dr16.remote_.s1_) {
        case 0:
            if (dr16.remote_.s2_ == 0) {
                state_machine.HandleEvent(kEventSwitchSubMode00);
            }
            break;

        case 1:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode11);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode12);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode13);
                    break;

                default:
                    break;
            }
            break;
        case 3:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode21);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode22);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode23);
                    break;

                default:
                    break;
            }
            break;
        case 2:
            switch (dr16.remote_.s2_) {
                case 1:
                    state_machine.HandleEvent(kEventSwitchSubMode31);
                    break;
                case 3:
                    state_machine.HandleEvent(kEventSwitchSubMode32);
                    break;
                case 2:
                    state_machine.HandleEvent(kEventSwitchSubMode33);
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

/// @brief 平滑函数
/// @param current_value 当前值
/// @param target_value 目标值
/// @param step 步进值
/// @return 计算后的值
int SmoothChange(int current_value, int target_value, int step) {
    if (current_value * target_value < 0) {
        if (current_value + step < target_value) {
            current_value += step * kReverseOperateCoeficient;
        } else if (current_value - step > target_value) {
            current_value -= step * kReverseOperateCoeficient;
        } else {
            current_value = target_value;
        }
    } else {
        if (current_value + step < target_value) {
            current_value += step;
        } else if (current_value - step > target_value) {
            current_value -= step;
        } else {
            current_value = target_value;
        }
    }

    return current_value;
}

/// @brief 速度分量分配
/// @param value1 值1
/// @param value2 值2
/// @param value3 值3
/// @param max_value 最大值
void AllocateVelocityComponents(float* value1, float* value2, float* value3, float max_value) {
    float sum = *value1 + *value2 + *value3;
    if (sum > max_value) {
        float value1_compenent_coefficient = *value1 / sum;
        float value2_compenent_coefficient = *value2 / sum;
        float value3_compenent_coefficient = *value3 / sum;
        *value1 = value1_compenent_coefficient * max_value;
        *value2 = value2_compenent_coefficient * max_value;
        *value3 = value3_compenent_coefficient * max_value;
    }
}

/// @brief 底盘跟随
void ChassisFollow() {
    //  ====================== 两头跟随 add in 2024/8/2 ====================
    if (comm.theta >= 0 && comm.theta <= 90) {
        theta = comm.theta;
    } else if (comm.theta > 90 && comm.theta <= 180) {
        theta = (comm.theta - 180);
    } else if (comm.theta > 180 && comm.theta <= 270) {
        theta = (comm.theta - 180);
    } else {
        theta = comm.theta - 360.0f;
    }

    if (comm.vision_is_use) {
        vw = -theta * kVisionFollowCoefficient;
    } else {
        vw = -theta * kFollowCoefficient;
    }
    //  ===================================================================
}

/// @brief 死亡检测
void dead_detect() {
    if (referee.robot_status.current_HP == 0 && dead_latch == false) {
        rotate_flag = false;
        follow_flag = false;
        dead_latch = true;
    } else if (referee.robot_status.current_HP != 0) {
        dead_latch = false;
    }
}
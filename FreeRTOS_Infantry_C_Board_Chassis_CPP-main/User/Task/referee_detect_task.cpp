///**
// * @file referee_detect_task.cpp
// * @author XMX
// * @brief 与裁判系统数据相关的检测任务
// * @version 1.0
// * @date 2024-08-07
// * 
// * @copyright Copyright (c) 2024
// * 
// */
//#include "referee_detect_task.hpp"
//#include "chassis.hpp"
//#include "cmsis_os2.h"
//#include "variables.hpp"
//#include "power_limit.hpp"

//extern uint16_t more_power;

//void capacity_power_detection();

///// @brief 功率保护任务
///// @param argument 
//void RefereeDetectTask(void* argument) {
//    for (;;) {
//        capacity_power_detection();  //底盘功率检测
//        osDelay(50);
//    }
//}

///// @brief 充电功率选择
//void capacity_power_detection() {
//    if (referee.power_heat_data.buffer_energy < 20) {
//        more_power = -200;
//    } else {
//        more_power = 0;
//    }
//		
//    switch (referee.robot_status.chassis_power_limit) {

////        case 45:
////            capacity.charge_power = 4200;
////            if (adjust_max_rpm_flag == false) {
////                max_rpm = 1900;
////								PowerControl.setMaxPower(43);
////            }else {
////						PowerControl.Wheel_PowerData.k1 = 6.91e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////						PowerControl.Wheel_PowerData.k2 = 7.07e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////						}
////            break;

////        case 50:
////            capacity.charge_power = 4700;
////            if (adjust_max_rpm_flag == false) {
////                max_rpm = 2100;
////								PowerControl.setMaxPower(46);
////            }else {
////							PowerControl.Wheel_PowerData.k1 = 6.91e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////        PowerControl.Wheel_PowerData.k2 = 7.07e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////						}
////            break;

////        case 55:
////            capacity.charge_power = 5200;
////            if (adjust_max_rpm_flag == false) {
////                max_rpm = 2300;
////								PowerControl.setMaxPower(51);
////            }else {
////							PowerControl.Wheel_PowerData.k1 = 6.91e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////        PowerControl.Wheel_PowerData.k2 = 7.07e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////						}
////            break;

//        case 60:
//            capacity.charge_power = 5700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3000;
//								PowerControl.setMaxPower(56);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 65:
//            capacity.charge_power = 6200;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3200;
//								PowerControl.setMaxPower(61);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}   
//            break;

//        case 70:
//            capacity.charge_power = 6700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3400;
//								PowerControl.setMaxPower(66);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 75:
//            capacity.charge_power = 7200;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3600;
//								PowerControl.setMaxPower(71);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 80:
//            capacity.charge_power = 7700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3800;
//								PowerControl.setMaxPower(76);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 85:
//            capacity.charge_power = 8200;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4000;
//								PowerControl.setMaxPower(81);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 90:
//            capacity.charge_power = 8700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4200;
//								PowerControl.setMaxPower(86);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 95:
//            capacity.charge_power = 9200;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4500;
//								PowerControl.setMaxPower(91);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

//        case 100:
//            capacity.charge_power = 9700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 5000;
//								PowerControl.setMaxPower(96);
//            }else {
//							PowerControl.setMaxPower ( referee.robot_status.chassis_power_limit);
//						}
//            break;

////        case 120:
////            capacity.charge_power = 11700;
////            if (adjust_max_rpm_flag == false) {
////                max_rpm = 6000;
////								PowerControl.setMaxPower(116);
////            }else {
////							PowerControl.Wheel_PowerData.k1 = 6.91e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////				PowerControl.Wheel_PowerData.k2 = 7.07e-7 * (60.0f/referee.robot_status.chassis_power_limit);
////						}
////            break;

//        default:
//            break;
//    }
//}
///**
// * @file referee_detect_task.cpp
// * @author XMX
// * @brief 与裁判系统数据相关的检测任务
// * @version 1.0
// * @date 2024-08-07
// * 
// * @copyright Copyright (c) 2024
// * 
// */
//#include "referee_detect_task.hpp"
//#include "chassis.hpp"
//#include "cmsis_os2.h"
//#include "variables.hpp"
//#include "power_limit.hpp"

//extern uint16_t more_power;

//void capacity_power_detection();

///// @brief 功率保护任务
///// @param argument 
//void RefereeDetectTask(void* argument) {
//    for (;;) {
//        capacity_power_detection();  //底盘功率检测
//        osDelay(50);
//    }
//}

///// @brief 充电功率选择
//void capacity_power_detection() {
//    if (referee.power_heat_data.buffer_energy < 20) {
//        more_power = -200;
//    } else {
//        more_power = 0;
//    }
////	   power_care.chassis_max_power = referee.robot_status.chassis_power_limit;//referee.robot_status.chassis_power_limit

//	power_care.PowerCare();
//	motor_201.input_ = power_care.speed_pid[0];  
//    motor_202.input_ = power_care.speed_pid[1];
//    motor_203.input_ = power_care.speed_pid[2];
//    motor_204.input_ = power_care.speed_pid[3];
//		
//    switch (referee.robot_status.chassis_power_limit) {

//        case 45:
//            capacity.charge_power = 4000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 1900;
//								power_care.chassis_max_power = 40;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 50:
//            capacity.charge_power = 4500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 2100;
//								power_care.chassis_max_power = 45;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 55:
//            capacity.charge_power = 5000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 2300;
//								power_care.chassis_max_power = 50;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 60:
//            capacity.charge_power = 5500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3000;
//								power_care.chassis_max_power = 55;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 65:
//            capacity.charge_power = 6000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3200;
//								power_care.chassis_max_power = 60;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 70:
//            capacity.charge_power = 6500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3400;
//								power_care.chassis_max_power = 65;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 75:
//            capacity.charge_power = 7000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3600;
//								power_care.chassis_max_power = 70;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 80:
//            capacity.charge_power = 7500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 3800;
//								power_care.chassis_max_power = 75;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 85:
//            capacity.charge_power = 8000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4000;
//								power_care.chassis_max_power = 80;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 90:
//            capacity.charge_power = 8500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4200;
//								power_care.chassis_max_power = 85;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 95:
//            capacity.charge_power = 9000;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4400;
//								power_care.chassis_max_power = 90;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 100:
//            capacity.charge_power = 9500;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 4600;
//								power_care.chassis_max_power = 95;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        case 120:
//            capacity.charge_power = 11700;
//            if (adjust_max_rpm_flag == false) {
//                max_rpm = 6000;
//								power_care.chassis_max_power = 116;
//            }else {
//							power_care.chassis_max_power = referee.robot_status.chassis_power_limit;
//						}
//            break;

//        default:
//            break;
//    }
//}
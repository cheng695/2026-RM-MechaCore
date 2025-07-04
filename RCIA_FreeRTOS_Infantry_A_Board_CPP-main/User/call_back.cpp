#include "call_back.hpp"
#include "clamp.hpp"
#include "error_handle.hpp"
#include "gimbal.hpp"
#include "variables.hpp"
#include "can.h"
#include "Heat_Detector.hpp"
#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "math.h"
#ifdef __cplusplus
}	
#endif  // __cplusplus
float lv_motor_205;
float ch110_Z;
uint8_t send_str2[64];
void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = 4;

    *((float *)&send_str2[sendSize * 0]) = x1;
    *((float *)&send_str2[sendSize * 1]) = x2;
    *((float *)&send_str2[sendSize * 2]) = x3;
    *((float *)&send_str2[sendSize * 3]) = x4;
    *((float *)&send_str2[sendSize * 4]) = x5;
    *((float *)&send_str2[sendSize * 5]) = x6;

    *((uint32_t *)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
    HAL_UART_Transmit_DMA(&huart8, send_str2, sizeof(float) * (7 + 1));
}
float Hz = 0.5;
int16_t B = 1;
uint16_t first_flag;
float target_speed = 0;

static bool is_timing = false;    // 标记是否正在计时

void CanCallBack(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == kMotorCan->Instance) {
        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};

        HAL_CAN_GetRxMessage(kMotorCan, CAN_RX_FIFO0, &rx_header, buf);  //接收数据
			if (rx_header.StdId == 0x201){
                dji_motor_list[3]->DataUpdate(buf);   //更新电机数据
                dji_motor_list[3]->is_reply_ = true;  //应答
				dji_motor_list[3]->recv_id_ = 0x201;
			}else {
				for (uint8_t i = 0; i < kMotorCount; i++) {
					if (rx_header.StdId == dji_motor_list[i]->recv_id_) {
						dji_motor_list[i]->DataUpdate(buf);   //更新电机数据
						dji_motor_list[i]->is_reply_ = true;  //应答
						break;
					}
				}
			}
    } else if (hcan->Instance == kImuCan->Instance) {

        CAN_RxHeaderTypeDef rx_header{0};
        uint8_t buf[8]{0};

        HAL_CAN_GetRxMessage(kImuCan, CAN_RX_FIFO0, &rx_header, buf);  //接收数据
			
        if(rx_header.StdId != master_id){
							if (rx_header.StdId == 0x388) {
									ch110.EulerUpdate(buf);  //欧拉角更新
							} else if (rx_header.StdId == 0x288) {
									ch110.VelocityUpdate(buf);  //角速度更新
							}
							ch110.is_reply_ = true;  //应答
							
						if(first_flag != 12)ctrl_enable();
        }else 
			if(rx_header.StdId == master_id){
            dm4310_fbdata(&(motor[Motor1]), buf);
			if(first_flag != 12)first_flag ++;
        }
    }
}
uint8_t gete;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == kRemoteUart->Instance) {
        if (Size == kRemoteSize) {
            dr16.DataUpdate(remote_rx_buf);  //遥控器数据更新
            dr16.is_reply_ = true;           //应答
        }
        //重新启动接收
        if (HAL_UARTEx_ReceiveToIdle_DMA(kRemoteUart, remote_rx_buf, kRemoteSize) != HAL_OK) {
            /*ErrorHandle(kHalLibError);*/
        }

    } else if (huart->Instance == kCommUart->Instance && Size == kCommRecvSize) {
        comm.RecvUpdate(comm_rx_buf);  //通信数据更新
        comm.is_reply_ = true;         //应答
        //重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(kCommUart, comm_rx_buf, kCommRecvSize);
    } else if (huart->Instance == kVisionUart->Instance) {
        if (Size == kVisionRecvSize) {
            vision.RecvUpdate(vision_rx_buf);  //视觉数据更新
            vision.recv_time = HAL_GetTick();
            vision.is_reply = true;  //应答
        }
        HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize);
        //重新启动接收
        //if (HAL_UARTEx_ReceiveToIdle_DMA(kVisionUart, vision_rx_buf, kVisionRecvSize) != HAL_OK) {
        //    ErrorHandle(kHalLibError);
        //}
    }
}
int time_shoot;
int enter_shoot = 0;
extern uint8_t blocking_flag;
void MainCallBack() {                                               
    /*  =========================== 摩擦轮、拨盘电机控制 ===========================  */

	//target_speed = 2000;
	
	//pitch_target_euler = sinf  (2 * 3.14 * HAL_GetTick() * 0.001 * Hz) * B;
	
    auto measure_rpm_201 = td_201.Compute(motor_201.actual_rpm_);  //获取微分跟踪器滤波后转速
    auto dji_motor_201_input = pid_vel_201.Compute(friction_target_rpm, measure_rpm_201);
    motor_201.input_ = dji_motor_201_input;  //设置电机输出

    //摩擦轮转速PID
    auto measure_rpm_202 = td_202.Compute(motor_202.actual_rpm_);  //获取微分跟踪器滤波后转速
    auto dji_motor_202_input = pid_vel_202.Compute(-friction_target_rpm, measure_rpm_202);
    motor_202.input_ = dji_motor_202_input;  //设置电机输出

    //if (blocking_flag == 0) {
//        //拨盘外环PID
		//auto temp_204 = pid_pos_204.Compute(trigger_target_pos, motor_204.encoder_integral_);
		 enter_shoot +=1;
			if(motor_204.actual_rpm_ == 0)
			{
				enter_shoot =0;
			}
		if(motor_204.actual_rpm_ < -500)
		{  
			//
		}
		target_speed = -(dr16.remote_.ch3_ - 1024) * 10;
        auto measure_rpm_204 = td_204.Compute(motor_204.actual_rpm_);  //获取微分跟踪器滤波后转速
        auto dji_motor_204_input = pid_vel_204.Compute(target_speed, measure_rpm_204);
        motor_204.input_ = dji_motor_204_input;  //设置电机输出
    //}
    

    /*  =========================== YAW、PITCH电机控制 ===========================  */

    //视觉作用于目标值时需要关闭前馈，不然会有高频振动（不太影响控制但会响声，会影响部件寿命）
    if (vision.is_use_ && vision.is_aimed_ && vision.is_reply) {
        //pid_pos_205.k_feed_forward_ = 0;
        pid_pos_206.k_feed_forward_ = 0;
    } else {
        //pid_pos_205.k_feed_forward_ = kPitchFeedForward;
        pid_pos_206.k_feed_forward_ = kYawFeedForward;
    }

    //pitch外环PID
    /*auto euler_error_205 = clamp(pitch_target_euler, kLowestEuler, kHighestEuler) - ch110.roll_;*/
			auto temp_205 =
			pid_pos_205.Compute(clamp(pitch_target_euler, kLowestEuler, kHighestEuler), motor[Motor1].para.pos);
//			//auto temp_205 = pitch_target_euler;
////		if(!vision.is_use_ || !vision.is_aimed_ || !vision.is_reply){
////				temp_205 =
////				pid_pos_205.Compute(clamp(pitch_target_euler, kLowestEuler, kHighestEuler), motor[Motor1].para.pos);
////		}else{
////			temp_205 = clamp(temp_205,5.0,-5.0);
////		}

//    //pitch内环PID
//	float lv_motor_205 = td_205.Compute(pid_pos_205.output_);
    auto dji_motor_205_input = pid_vel_205.Compute(temp_205, motor[Motor1].para.vel);
    motor[Motor1].ctrl.tor_set = //设置电机输出
        clamp(dji_motor_205_input - EGC.Compute(motor[Motor1].para.pos_vision), -2.5f, 2.5f);  //重力补偿
	//mit_ctrl(&hcan2,0x03,target_pitch,0,120.0f,3.0f,clamp( -EGC.Compute(motor[Motor1].para.pos_vision), -2.0f, 2.0f));
    //motor[Motor1].ctrl.tor_set = dji_motor_205_input;

    //yaw外环PID
    //yaw_target_euler =
        //clamp(yaw_target_euler, ch110.yaw_integral_ - 180.0f, ch110.yaw_integral_ + 180.0f);
    //auto temp_206 = pid_pos_206.Compute(yaw_target_euler, ch110.yaw_integral_);
	 
//    //yaw内环PID
		target_speed = (dr16.remote_.ch0_ - 1024) * 4;
		ch110_Z = td_ch110_Z.Compute(ch110.z_velocity_);
    auto dji_motor_206_input = pid_vel_206.Compute(-target_speed, ch110.z_velocity_);
    motor_206.input_ = clamp(dji_motor_206_input + RFF.Compute(), -16384.0f, 16384.0f);

	if((dr16.remote_.s1_ != 1 || dr16.remote_.s2_ != 1 ) ){
		DjiMotorSend();  //电机数据发送
		ctrl_send();
	}else {
		motor[Motor1].ctrl.tor_set = 0;
		ctrl_send();
	}
    //uint8_t send_str2[(vofa_cnt + 1) * 4] = {0};
    //*((float*)&send_str2[0 * 4]) = (float)ch110.z_velocity_;
    //*((float*)&send_str2[1 * 4]) = (float)0;
    //*((uint32_t*)&send_str2[sizeof(float) * (vofa_cnt)]) = 0x7f800000;
    ////开始发送数据
    //HAL_UART_Transmit_DMA(&huart7, send_str2, sizeof(float) * (vofa_cnt + 1));
			vofaSend(target_speed, ch110.z_velocity_, 0, 0, 0, 0) ;
}

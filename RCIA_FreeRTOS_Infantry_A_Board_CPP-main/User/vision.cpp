#include "vision.hpp"
#include <stm32f4xx_hal.h>
#include "gimbal.hpp"
#include "math.h"
#include "stdio.h"
#include "uart.hpp"
#include "variables.hpp"
#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
bool Vision::AbsoluteFilte() const {
    if (fabsf(yaw_increment_temp) < 21 && fabsf(pitch_increment_temp) < 21 &&
        fabsf(yaw_hub_increment_temp) < 21 && fabsf(pitch_hub_increment_temp) < 21) {
        return true;
    } else {
        return false;
    }
};
uint8_t vision_ready;
vision_t vision_buf;
void Vision::RecvUpdate(const uint8_t* buf) {
	
	if (buf[0] == 0x39 && buf[1] == 0x39)
    {	
		origin_pitch_ = (float)(buf[2] << 24 | buf[3] << 16 | buf[4] << 8 | buf[5]) / 100.0f;
				
		origin_yaw_ = (float)(buf[6] << 24 | buf[7] << 16 | buf[8] << 8 | buf[9]) / 100.0f;
		vision_ready = buf[10];
        fire_flag = (buf[11]);
		recv_time = HAL_GetTick();
//        rx_other.tail = buf[12];
		}
	if (vision_ready != 1)
        {
            origin_yaw_ = 0;
            origin_pitch_ = 0;
        }
	
//    origin_pitch_hub =
//        (int16_t)((buf[1] - 48) * 1000 + (buf[2] - 48) * 100 + (buf[3] - 48) * 10 + (buf[4] - 48));
//    origin_yaw_hub_ =
//        (int16_t)((buf[6] - 48) * 1000 + (buf[7] - 48) * 100 + (buf[8] - 48) * 10 + (buf[9] - 48));
//    origin_pitch_ = (int16_t)((buf[11] - 48) * 1000 + (buf[12] - 48) * 100 + (buf[13] - 48) * 10 +
//                              (buf[14] - 48));
//    origin_yaw_ = (int16_t)((buf[16] - 48) * 1000 + (buf[17] - 48) * 100 + (buf[18] - 48) * 10 +
//                            (buf[19] - 48));
//    fire_flag = buf[21] - 48;

//    //开销有点大了，肉眼可见的影响云台响应
//    // sscanf((const char*)buf, "[%4hu/%4hu/%4hu/%4hu/%1hu]", &origin_pitch_hub, &origin_yaw_hub_,
//    //        &origin_pitch_, &origin_yaw_, &fire_flag_temp);
//    // fire_flag = fire_flag_temp;  //开火位赋值到布尔量

//    if ((origin_pitch_ == 0 && origin_pitch_hub == 0 && origin_yaw_ == 0 && origin_yaw_hub_ == 0) ||
//        (HAL_GetTick() - recv_time) > 200) {
//        yaw_increament = 0;
//        pitch_increment = 0;
//        yaw_hub_increment = 0;
//        pitch_hub_increment = 0;
//        fire_flag = false;
//        is_aimed_ = false;
//        return;
//    }

//    is_aimed_ = false;  //先假设接收失败

    yaw_increment_temp =((origin_yaw_));
    pitch_increment_temp =((origin_pitch_));
//    yaw_hub_increment_temp = (float)((origin_yaw_hub_ - 5000) / 100.0f);
//    pitch_hub_increment_temp = (float)((origin_pitch_hub - 5000) / 100.0f);

    if (AbsoluteFilte()) {
        is_aimed_ = true;
        if (vision.is_use_) {
            yaw_increament = yaw_increment_temp + ch110.yaw_integral_;
            pitch_increment = (-pitch_increment_temp)	+ motor[Motor1].para.pos;
//            yaw_hub_increment = yaw_hub_increment_temp;
//            pitch_hub_increment = pitch_hub_increment_temp;

            //  ============================== 绝对角度 add in 2024/7/22 ============================
//            if (vision.aim_type_ == kArmor) {
                yaw_target_euler = yaw_increament;
                pitch_target_euler = pitch_increment;
//            } 
//						else {
//                yaw_target_euler = -yaw_hub_increment + ch110.yaw_integral_;
//                pitch_target_euler = -pitch_hub_increment * 22.75 + motor[0].para.pos;
//            }
            //  ====================================================================================
        }
		}
}

float time_kp = 0.1, time_out;
float int_time = 5;
uint8_t vision_flag;
uint32_t demo_time;
uint32_t send_time,tx_gimbal_time,rx_target_time;
void Vision::Send() {
//    yaw = (int16_t)(ch110.yaw_ * 10.0f);
//    yaw += 1800;
//    yaw_angle = (uint16_t)yaw;

//    auto theta = CalculateTheta(motor_206.encoder_value_, kYawInitialEncoderValue);
//    auto chassis_theta = (int16_t)(theta * 10.0f);
//    auto chhassis_theta_angle = (uint16_t)chassis_theta;

	vision_buf.pitch_360 = motor[0].para.pos_vision * 100;
	vision_buf.yaw_360 = ch110.yaw_ * 100;
	vision_buf.vision_TX_data[0] = 0x39;
	vision_buf.vision_TX_data[1] = 0x39;
	vision_buf.vision_TX_data[2] = (int32_t)vision_buf.pitch_360>>24;
	vision_buf.vision_TX_data[3] = (int32_t)vision_buf.pitch_360>>16;
	vision_buf.vision_TX_data[4] = (int32_t)vision_buf.pitch_360>>8;
	vision_buf.vision_TX_data[5] = (int32_t)vision_buf.pitch_360;
	vision_buf.vision_TX_data[6] = (int32_t)vision_buf.yaw_360>>24;
	vision_buf.vision_TX_data[7] = (int32_t)vision_buf.yaw_360>>16;
	vision_buf.vision_TX_data[8] = (int32_t)vision_buf.yaw_360>>8;
	vision_buf.vision_TX_data[9] = (int32_t)vision_buf.yaw_360;
	vision_buf.vision_TX_data[10] = 26; 
	vision_buf.vision_TX_data[11] = 0x42; //0x42?   0X52??
	vision_buf.vision_TX_data[12] = vision_flag;
	vision_buf.vision_TX_data[13] = 0XFF;
	send_time++;
    tx_gimbal_time = send_time;
	rx_target_time = (vision_rx_buf[13] << 24 | vision_rx_buf[14] << 16 | vision_rx_buf[15] << 8 | vision_rx_buf[16]);
	
    demo_time = tx_gimbal_time - rx_target_time;

    vision_buf.vision_TX_data[14] = (int32_t)tx_gimbal_time >> 24;
    vision_buf.vision_TX_data[15] = (int32_t)tx_gimbal_time >> 16;
    vision_buf.vision_TX_data[16] = (int32_t)tx_gimbal_time >> 8;
    vision_buf.vision_TX_data[17] = (int32_t)tx_gimbal_time;

    HAL_UART_Transmit_DMA(kVisionUart, vision_buf.vision_TX_data, 18);
		
		if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = (uint32_t)(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

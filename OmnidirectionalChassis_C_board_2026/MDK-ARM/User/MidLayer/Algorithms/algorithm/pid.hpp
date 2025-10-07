#ifndef _PID_HPP_
#define _PID_HPP_ 

#include "../User/MidLayer/Controllers/chassis_control/PidTarget.hpp"

namespace PID
{
    class PID_
    {
        public:

            enum PID_MODE
            {
                PID_POSITION = 0,
                PID_SPEED = 1,
                PID_DELTA = 2
            };

            PID_(uint8_t mode, float kp, float ki, float kd, float max_out, float max_iout, float max_error_i)
                : mode_(mode), kp_(kp), ki_(ki), kd_(kd), max_error_i_(max_error_i), max_out_(max_out), max_iout_(max_iout){};
            
            PID_(float angle_kp, float angle_ki, float angle_kd, float angle_max_out, float angle_max_iout, float angle_max_error_i, 
                float speed_kp, float speed_ki, float speed_kd, float speed_max_out, float speed_max_iout, float speed_max_error_i)
                    : kp_speed_(speed_kp), ki_speed_(speed_ki), kd_speed_(speed_kd), max_out_speed_(speed_max_out), max_iout_speed_(speed_max_iout), max_error_i_speed_(speed_max_error_i),
                      kp_angle_(angle_kp), ki_angle_(angle_ki), kd_angle_(angle_kd), max_out_angle_(angle_max_out), max_iout_angle_(angle_max_iout), max_error_i_angle_(angle_max_error_i){};

            uint8_t mode_;
            //三参数
            float kp_;
            float ki_;
            float kd_;

            float max_error_i_;  //用于积分分离
            float max_out_;      //最大输出
            float max_iout_;     //最大积分

            float set_;   //目标值
            float fact_;  //实际反馈值

            float out_;
            float p_out;
            float i_out;
            float d_out;

            float angle_add;

            float Dbuf[3];
            float error[3];  //d的计算所用 现在，前一个，再前一个 误差值   

            float angle_set_;
            float angle_fact_;

            float speed_set_;
            float speed_fact_;

            float kp_speed_;
            float ki_speed_;
            float kd_speed_;
            float max_out_speed_;
            float max_iout_speed_;
            float max_error_i_speed_;

            float kp_angle_;
            float ki_angle_;
            float kd_angle_;
            float max_out_angle_;
            float max_iout_angle_;
            float max_error_i_angle_;

            float SpeedError[3];
            float AngleError[3];
            float SpddeDbuf[3];
            float AngleDbuf[3];

            float p_out_angle;
            float i_out_angle;
            float d_out_angle;
            float angle_out_;

            float p_out_speed;
            float i_out_speed;
            float d_out_speed;
            float speed_out_;

            float NormalPID(float ref, float set);
            float Double_Ring_PID(float ref_angle, float ref_speed, float set_angle);
        
        private:
            void Limit_max(float *out, float max_out);  //限幅
            float Zero_crossing_processing (float ref, float set);  //过零处理
    };
}

#endif

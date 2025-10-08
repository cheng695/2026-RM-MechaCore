#include "pid.hpp"

PID::PID_ M3508[4] = {
    PID::PID_(PID::PID_::PID_MODE::PID_SPEED, 5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    PID::PID_(PID::PID_::PID_MODE::PID_SPEED, 5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    PID::PID_(PID::PID_::PID_MODE::PID_SPEED, 5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    PID::PID_(PID::PID_::PID_MODE::PID_SPEED, 5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f)
};
PID::PID_ Revolve_PidInstance(PID::PID_::PID_MODE::PID_POSITION, 2000.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f);

float PID::PID_::NormalPID(float ref, float set)
{
    this->error[2] = this->error[1];
    this->error[1] = this->error[0];
    this->set_ = set;
    this->fact_ = ref;

    if(this->mode_ != this->PID_DELTA)
    {
         if (this->mode_ == this->PID_SPEED)
        {
            this->error[0] = set - ref;
        }
        else if(this->mode_ == this->PID_POSITION)
        {
            this->error[0] = PID::PID_::Zero_crossing_processing(ref, set);
        }

        this->p_out = this->kp_ * this->error[0];

        //ki
        if(this->error[0] < this->max_error_i_)        //积分分离
        {
            this->i_out += this->ki_ * this->error[0];
            PID::PID_::Limit_max(&this->i_out, this->max_iout_); // 积分限幅
        }
        else if(this->error[0] >= this->max_error_i_)
        {
            this->i_out = 0;
        }

        //kd
        this->d_out = this->kd_ * (this->error[0] - this->error[1]);

        this->out_ = this->p_out + this->i_out + this->d_out;

        PID::PID_::Limit_max(&this->out_, this->max_out_);//函数没写 输出限幅
    }
   
    else if (this->mode_ == this->PID_DELTA)
    {
        this->p_out = this->kp_ * (this->error[0] - this->error[1]);
        this->i_out = this->ki_ * this->error[0];
        this->Dbuf[2] = this->Dbuf[1];
        this->Dbuf[1] = this->Dbuf[0];
        this->Dbuf[0] = (this->error[0] - 2.0f * this->error[1] + this->error[2]);
        this->d_out = this->kd_ * this->Dbuf[0];
        this->out_ = this->p_out + this->i_out + this->d_out;
        PID::PID_::Limit_max(&this->i_out, this->max_iout_);
    }
    return this->out_;
}

float PID::PID_::Double_Ring_PID(float ref_angle, float ref_speed, float set_angle)
{
    this->AngleError[2] = this->AngleError[1];
    this->AngleError[1] = this->AngleError[0];
    this->angle_set_ = set_angle;
    this->angle_fact_ = ref_angle;
    this->AngleError[0] = PID::PID_::Zero_crossing_processing(ref_angle, set_angle);

    this->p_out_angle = this->kp_angle_ * this->AngleError[0];

    //ki
    if(this->AngleError[0] < this->max_error_i_angle_)        //积分分离
    {
        this->i_out_angle += this->ki_angle_ * this->AngleError[0];
        PID::PID_::Limit_max(&this->i_out_angle, this->max_iout_angle_); // 积分限幅
    }
    else if(this->AngleError[0] >= this->max_error_i_angle_)
    {
        this->i_out_angle = 0;
    }

    //kd
    this->d_out_angle = this->kd_angle_ * (this->AngleError[0] - this->AngleError[1]);

    this->angle_out_ = this->p_out_angle + this->i_out_angle + this->d_out_angle;//加了东西

    PID::PID_::Limit_max(&this->angle_out_, this->max_out_angle_);//函数没写 输出限幅
    

    this->SpeedError[2] = this->SpeedError[1];
    this->SpeedError[1] = this->SpeedError[0];
    this->speed_set_ = this->angle_out_;
    this->speed_fact_ = ref_speed;
    this->SpeedError[0] = this->angle_out_ - ref_speed;

    this->p_out_speed = this->kp_speed_ * this->SpeedError[0];

    //ki
    if(this->SpeedError[0] < this->max_error_i_speed_)        //积分分离
    {
        this->i_out_speed += this->ki_speed_ * this->SpeedError[0];
        PID::PID_::Limit_max(&this->i_out_speed, this->max_iout_speed_); // 积分限幅
    }
    else if(this->SpeedError[0] >= this->max_error_i_speed_)
    {
        this->i_out_speed = 0;
    }

    //kd
    this->d_out_speed = this->kd_speed_ * (this->SpeedError[0] - this->SpeedError[1]);

    this->speed_out_ = this->p_out_speed + this->i_out_speed + this->d_out_speed;//加了东西

    PID::PID_::Limit_max(&this->speed_out_, this->max_out_speed_);//函数没写 输出限幅
    
    return this->speed_out_;
}



/*  =========================== 限幅 ===========================  */ 
void PID::PID_::Limit_max(float *out, float max_out)//限幅
{
    if(*out >= max_out)
    {
        *out = max_out;
    }
    else if(*out <= -max_out)
    {
        *out = -max_out;
    }
}

/*  =========================== 过零处理 ===========================  */ 
float PID::PID_::Zero_crossing_processing (float ref, float set)   //过零处理
{
	float angle;

	if (set - ref > 180)
	{
		ref += 360;
	}
	else if (set - ref < -180)
	{
		ref = ref - 360;
	}
	angle = set - ref;

	return angle;
}

#include "pid.hpp"

float PID::clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    }
    return value;
}

float PID::Compute(float target, float measure) {
    error_ = target - measure;
    // 只有在输出未饱和时才更新积分项（积分分离）
    bool output_not_saturated = (output_ > -output_limit_) && (output_ < output_limit_);
    if (output_not_saturated) {
		integral_ += error_;       
    }
		//integral_ += error_;
    //integral_ = clamp(integral_, -integral_limit_, integral_limit_);
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;
    feed_forward_ = (target - prev_target_) * k_feed_forward_;
    prev_target_ = target;
    output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative_ + feed_forward_;
    output_ = clamp(output_, -output_limit_, output_limit_);
    return output_;
}

float PID::Compute(float error) {
    error_ = error;
    bool output_not_saturated = (output_ > -output_limit_) && (output_ < output_limit_);
    if (output_not_saturated) {
        integral_ += error_;
    }
		//integral_ += error_;
    //integral_ = clamp(integral_, -integral_limit_, integral_limit_);
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;
    output_ = kp_ * error + ki_ * integral_ + kd_ * derivative_;
    output_ = clamp(output_, -output_limit_, output_limit_);
    return output_;
}

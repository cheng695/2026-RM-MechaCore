namespace ALG::PID
{
class PID
{
  private:
    float k_[3];
    float k_out_[3];
    float integral_;
    float previous_error_;
    float output_;
    float max_;
    float min_;
    float target_;
    float feedback_;
    float error_;
    float integral_limit_;                // 积分限幅值
    float integral_separation_threshold_; // 积分隔离阈值

  public:
    PID(float kp, float ki, float kd, float max);
    float Calc(float feedback);
    void reset();
    void setTarget(float target);
    void setFeedback(float feedback);
    void setK(float kp, float ki, float kd);
    void setMax(float max);
    void setIntegralLimit(float integral_limit);
    void setIntegralSeparation(float threshold);
    float getOutput();
    float getError();
};

/**
 * @brief 重置PID控制器
 */
inline void PID::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    output_ = 0.0f;
    error_ = 0.0f;
    k_out_[0] = k_out_[1] = k_out_[2] = 0.0f;
}

/**
 * @brief 设置目标值
 *
 * @param target 目标值
 */
inline void PID::setTarget(float target)
{
    target_ = target;
}

/**
 * @brief 设置反馈值
 *
 * @param feedback 反馈值
 */
inline void PID::setFeedback(float feedback)
{
    feedback_ = feedback;
}

/**
 * @brief 设置PID增益
 *
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
inline void PID::setK(float kp, float ki, float kd)
{
    k_[0] = kp;
    k_[1] = ki;
    k_[2] = kd;
}

/**
 * @brief 设置PID输出限幅
 *
 * @param max 输出限幅
 */
inline void PID::setMax(float max)
{
    max_ = max;
    min_ = -max;
}

/**
 * @brief 设置积分限幅
 *
 * @param integral_limit
 */
inline void PID::setIntegralLimit(float integral_limit)
{
    integral_limit_ = integral_limit;
}

/**
 * @brief 设置积分隔离阈值
 *
 * @param threshold
 */
inline void PID::setIntegralSeparation(float threshold)
{
    integral_separation_threshold_ = threshold;
}

/**
 * @brief 获取PID输出
 *
 * @return float
 */
inline float PID::getOutput()
{
    return output_;
}

/**
 * @brief 获取PID误差
 *
 * @return float
 */
inline float PID::getError()
{
    return error_;
}

} // namespace ALG::PID
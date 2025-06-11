#pragma once

#include <cmath>

namespace Alg::LADRC
{
class TDquadratic
{
  public:
    /**
     * @brief 二阶TD微分跟踪器的参数，一般来说确定好R就可以
     *
     * @param r       决定x1快慢的跟踪因子，越大则越快，小则反之。单位2*pi*Hz
     * @param max_x2  跟踪器的最大输出
     * @param h       采样周期，单位s
     */
    TDquadratic(float r = 300.0f, float h = 0.001f) : r(r), h(h)
    {
    }

    /**
     * @brief 获取x1
     *
     * @return float
     */
    float getX1()
    {
        return x1;
    }

    /**
     * @brief 获取x2
     *
     * @return float
     */
    float getX2()
    {
        return x2;
    }

    /**
     * @brief TD微分跟踪器，其主要作用为消除系统的初期超调
     *
     * @param u 跟踪信号
     * @return float 跟踪器输出
     */
    float Calc(float u);

  private:
    float u_;
    float x1, x2, max_x2;
    float r, h = 0.001, r2_1;
};

/**
 * @brief 一阶ADRC控制器
 *
 */
class Adrc
{
  public:
    Adrc(TDquadratic td = TDquadratic(1, 0), float Kp = 0, float wc = 0, float b0 = 1, float h = 0.001f, float max = 0)
        : Kp_(Kp), wc_(wc), b0_(b0), h_(h), max_(max), td_(td)
    {
    }

    /**
     * @brief 更新全部adrc参数
     *
     * @param target    目标值
     * @param feedback  反馈值
     * @return float
     */
    float UpData(float feedback);

    /**
     * @brief 设置目标值
     *
     * @param target 目标值
     */
    void setTarget(float target)
    {
        target_ = target;
    }

    /**
     * @brief 设置反馈值
     *
     * @param feedback 反馈值
     */
    void setFeedback(float feedback)
    {
        feedback_ = feedback;
    }

    /**
     * @brief 获取目标值
     *
     * @return float
     */
    float getTarget()
    {
        return target_;
    }

    /**
     * @brief 获取反馈值
     *
     * @return float
     */
    float getFeedback()
    {
        return feedback_;
    }

    /**
     * @brief 获取控制器输出
     *
     * @return float
     */
    float getU()
    {
        return u;
    }

    /**
     * @brief 获取z1
     *
     * @return float
     */
    float getZ1()
    {
        return z1;
    }

    /**
     * @brief 重置控制器
     *
     */
    void reSet()
    {
        u = 0;
        target_ = feedback_;
    }

  private:
    float Kp_;
    float z1, z2;
    float wc_;                   // 观测器带宽
    float target_, feedback_, e; // 反馈值和误差
    float u;                     // 控制器输出
    float b0_ = 1;               // 控制器增益（调节单位用）
    float beta1, beta2;          // ESO增益
    float h_ = 0.001f;           // 采样周期
    float max_;
    /**
     * @brief 二阶线性扩张状态观测器（ESO）
     *
     * @param target    // 目标值
     * @param feedback  // 反馈值
     */
    void ESOCalc(float target, float feedback);

    /**
     * @brief   //状态误差反馈控制律（SEF）
     *
     */
    void SefCalc();

    TDquadratic td_;
};

} // namespace Alg::LADRC

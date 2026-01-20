#include "adrc.hpp"

/**
 * @brief 一阶线性自抗扰控制 (1st-Order LADRC)
 * @details 适用于一阶惯性系统控制。包含线性扩张状态观测器 (LESO) 和线性误差反馈 (LSEF)。
 * 
 * @param input 目标值 (SetPoint)
 * @param feedback 反馈值 (Measurement)
 * @return float 控制量输出 (u)
 */
float ALG::ADRC::FirstLADRC::LADRC_1(float input, float feedback)
{
    LESO_1(feedback);
    LSEF_1(input);
    std::clamp(U, GetMin(), GetMax());
    return U;
}

/**
 * @brief 线性状态误差反馈 (LSEF) - 一阶
 * @details 简单的比例控制器 (P control)，利用 LESO 观测到的总扰动 Z2 进行补偿。
 *          算法: u0 = Kp * (v - z1)
 *               u  = (u0 - z2) / b0
 * 
 * @param target 目标值
 */
void ALG::ADRC::FirstLADRC::LSEF_1(float target)
{
    KP = GetWc(); // 控制带宽决定 KP

    U0 = KP * (target - Z1);
    U = (U0 - Z2) / GetB0(); // 扰动补偿
}

/**
 * @brief 线性扩张状态观测器 (LESO) - 一阶
 * @details 观测系统状态 z1 (近似y) 和扩张状态 z2 (总扰动)。
 *          状态方程:
 *          z1_dot = z2 + b0*u + beta1*(y - z1)
 *          z2_dot = beta2*(y - z1)
 * 
 * @param feedback 系统当前输出 (y)
 */
void ALG::ADRC::FirstLADRC::LESO_1(float feedback)
{
    // 参数计算 (基于观测器带宽 W0)
    Beta1 = 2.0f * GetW0();
    Beta2 = GetW0() * GetW0();

    E = feedback - Z1; // 观测误差
    
    // Euler 离散化更新
    Z1 += GetH() * (Beta1 * E + Z2 + GetB0() * U);
    Z2 += GetH() * (Beta2 * E);
}





/**
 * @brief 二阶线性自抗扰控制 (2nd-Order LADRC)
 * @details 适用于二阶系统 (如位置伺服)。包含 TD, LESO, LSEF。
 * 
 * @param input 目标输入
 * @param feedback 系统反馈
 * @return float 控制量
 */
float ALG::ADRC::SecondLADRC::LADRC_2(float input, float feedback)
{
    TD_2(input);       // 安排过渡过程
    LESO_2(feedback);  // 观测状态和扰动
    LSEF_2();          // 误差反馈控制
    std::clamp(U, GetMin(), GetMax());
    return U;
}

/**
 * @brief 线性状态误差反馈 (LSEF) - 二阶
 * @details 类似 PD 控制器，加上扰动补偿。
 *          u0 = Kp*(v1 - z1) + Kd*(v2 - z2)
 *          u  = (u0 - z3) / b0
 */
void ALG::ADRC::SecondLADRC::LSEF_2()
{
    // 参数整定 (基于控制带宽 Wc)
    KP = GetWc() * GetWc();
    KD = 2.0f * GetWc();

    U0 = KP * (V1 - Z1) + KD * (V2 - Z2);
    U = (U0 - Z3) / GetB0(); // Z3 为观测到的总扰动
}

/**
 * @brief 线性扩张状态观测器 (LESO) - 二阶
 * @details 观测状态 z1 (位置), z2 (速度), z3 (总扰动)。
 *          状态方程基于 Luenberger 观测器设计。
 * 
 * @param feedback 系统位置反馈
 */
void ALG::ADRC::SecondLADRC::LESO_2(float feedback)
{
    // 参数整定 (基于观测器带宽 W0)
    Beta1 = 3.0f * GetW0();
    Beta2 = 3.0f * GetW0() * GetW0();
    Beta3 = GetW0() * GetW0() * GetW0();

    E = feedback - Z1;

    // Euler 离散化更新
    Z1 += GetH() * (Z2 + Beta1 * E);
    Z2 += GetH() * (Z3 + GetB0() * U + Beta2 * E);
    Z3 += GetH() * (Beta3 * E);
}

/**
 * @brief 跟踪微分器 (TD)
 * @details 安排过渡过程 v1，并提取微分信号 v2。
 *          使用快速最优控制综合函数 (fhan) 的线性简化形式或直接积分。
 *          此处采用的是简单的二阶线性系统响应形式: 
 *          v1_dot = v2
 *          v2_dot = -r^2*(v1 - v) - 2*r*v2
 * 
 * @param input 阶跃目标值
 */
void ALG::ADRC::SecondLADRC::TD_2(float input)
{ 
    float fh= -R * R * (V1 - input) - 2 * R * V2;
    V1 += V2 * GetH();
    V2 += fh * GetH();
}


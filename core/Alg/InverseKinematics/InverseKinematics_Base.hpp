#ifndef INVERSE_KINEMATICS_BASE_HPP  // 去掉双下划线，符合标准
#define INVERSE_KINEMATICS_BASE_HPP

namespace Alg::InverseKinematics
{
    class IKBase
    {
    public:
        virtual ~IKBase() = default;  // 虚析构函数放在public

        // 使用成员初始化列表
        IKBase() : Signal_x(0.0f), Signal_y(0.0f), Signal_w(0.0f), 
                  Phase(0.0f), SpeedGain(1.0f), RotationalGain(1.0f) {}

        // Setter方法可以添加参数验证
        virtual void SetSignal_x(float x) { Signal_x = x; }
        virtual void SetSignal_y(float y) { Signal_y = y; }
        virtual void SetSignal_w(float w) { Signal_w = w; }
        
        virtual void SetPhase(float phase) { Phase = phase; }
        virtual void SetSpeedGain(float gain) { 
            // 可以添加范围检查
            SpeedGain = (gain >= 0.0f) ? gain : 0.0f; 
        }
        virtual void SetRotationalGain(float gain) { 
            RotationalGain = (gain >= 0.0f) ? gain : 0.0f; 
        }

        // Getter方法标记为const
        virtual float GetSignal_x() const { return Signal_x; }
        virtual float GetSignal_y() const { return Signal_y; }
        virtual float GetSignal_w() const { return Signal_w; }
        virtual float GetPhase() const { return Phase; }
        virtual float GetSpeedGain() const { return SpeedGain; }
        virtual float GetRotationalGain() const { return RotationalGain; }

    protected:  // 改为protected，方便派生类访问
        float Signal_x;
        float Signal_y;
        float Signal_w;
        float Phase;
        float SpeedGain;
        float RotationalGain;
    };
}

#endif
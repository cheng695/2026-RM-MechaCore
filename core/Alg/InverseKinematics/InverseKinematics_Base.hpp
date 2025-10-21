#ifndef INVERSE_KINEMATICS_BASE_HPP  
#define INVERSE_KINEMATICS_BASE_HPP

namespace Alg::InverseKinematics
{
    class IKBase
    {
    public:
        virtual ~IKBase() = default;  

        IKBase() : Signal_x(0.0f), Signal_y(0.0f), Signal_w(0.0f), 
                  Phase(0.0f), SpeedGain(1.0f), RotationalGain(1.0f) {}

        virtual void SetSignal_xyw(float x, float y, float w)
        { 
            this->Signal_x = x; 
            this->Signal_y = y;
            this->Signal_w = w;
        }
        

        virtual void SetPhase(float phase) { Phase = phase; }
        virtual void SetSpeedGain(float gain) { SpeedGain = gain; }
        virtual void SetRotationalGain(float gain) { RotationalGain = gain; }


        virtual float GetSignal_x() const { return Signal_x; }
        virtual float GetSignal_y() const { return Signal_y; }
        virtual float GetSignal_w() const { return Signal_w; }
        virtual float GetPhase() const { return Phase; }
        virtual float GetSpeedGain() const { return SpeedGain; }
        virtual float GetRotationalGain() const { return RotationalGain; }


    protected:  
        float Signal_x;
        float Signal_y;
        float Signal_w;
        float Phase;
        float SpeedGain;
        float RotationalGain;
    };
}

#endif
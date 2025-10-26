#ifndef INVERSE_DYNAMICS_BASE_HPP  
#define INVERSE_DYNAMICS_BASE_HPP

namespace Alg::InverseDynamics
{
    class IDBase
    {
        public:
            virtual ~IDBase() = default;

            IDBase() : Fx(0.0f), Fy(0.0f), Torque(0.0f){}

            virtual float GetFx() const { return Fx; }
            virtual float GetFy() const { return Fy; }
            virtual float GetTorque() const { return Torque; }

            virtual void Set_FxFyTor(float fx, float fy, float torque)
            {
                this->Fx = fx;
                this->Fy = fy;
                this->Torque = torque;
            }


        protected:
            float Fx;
            float Fy;
            float Torque;

    };
}

#endif

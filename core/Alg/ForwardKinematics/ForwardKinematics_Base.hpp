#ifndef FORWARD_KINEMATICS_BASE_HPP
#define FORWARD_KINEMATICS_BASE_HPP 

namespace Alg::ForwardKinematics
{
    class FKBase
    {
        public:
            virtual ~FKBase() = default;

            FKBase() : w0(0.0f), w1(0.0f), w2(0.0f), w3(0.0f){}

            void Set_w0w1w2w3(float w0, float w1, float w2, float w3)
            {
                this->w0 = w0;
                this->w1 = w1;
                this->w2 = w2;
                this->w3 = w3;
            }

            float Get_w0() const { return w0; }
            float Get_w1() const { return w1; }
            float Get_w2() const { return w2; }
            float Get_w3() const { return w3; }

        protected:
            float w0;
            float w1;
            float w2;
            float w3;
    };
}

#endif

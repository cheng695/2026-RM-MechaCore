#ifndef _CHASSIS_KINEMATICS_HPP_
#define _CHASSIS_KINEMATICS_HPP_ 

#include "../User/LowLayer/Equipment/motor/djimotor.hpp"

extern motor::GM3508<4> Motor3508;

namespace Kinematics
{
    template<uint8_t N> class Chassis
    {
        public:
            Chassis(uint8_t Wheelset[N]) : Wheelset_(Wheelset){}
            void Omnidirectional();
            void Target();
            uint8_t GetCount()
            {
                return N;
            }

        private:
            float VehicleDirection_w;
            float VehicleDirection_vx;
            float VehicleDirection_vy;
            float target;
            float Wheelset_;
    };
}

#endif

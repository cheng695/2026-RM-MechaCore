#ifndef _PID_TARGET_HPP_
#define _PID_TARGET_HPP_ 

#include "../User/MidLayer/Algorithms/kinematics/chassis_kinematics.hpp"

#define PIDQuantity 4

extern Kinematics::Wheelset<4> Omini;

namespace PidTarget
{
    template<uint8_t N> class pid_target 
    {
        public:
            void stop(uint8_t count)
            {
                for(uint8_t i = 0; i < N; i++)
                {
                    Omini.setVehicleDirection_w(0.0f);
                    Omini.setVehicleDirection_vy(0.0f);
                    Omini.setVehicleDirection_vx(0.0f);
                }
            }

            void pidset(uint8_t idenx)
            {
                Omini.Target();
                PidTarget[idenx] = Omini.GetTarget(idenx);
            }

            uint8_t GetCount()
            {
                return N;
            }

            float GetPidTarget(uint8_t idenx)
            {
                return PidTarget[idenx];
            }

        private:
            float PidTarget[N];

    };
}

#endif

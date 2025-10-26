#pragma once

#include "core/Alg/ADRC/adrc.hpp"
#include "core/Alg/FSM/alg_fsm.hpp"
#include "core/Alg/PID/pid.hpp"

#ifdef __cplusplus

namespace TASK::MotorPid
{
enum MotorPid_Status
{
    DISABLE,
    PID,
    ADRC,
};

class MotorPid : public Class_FSM
{
  public:
    MotorPid();
    ~MotorPid();
    void UpData();

  private:
    void AdrcUpData(void);
    void PidUpData(void);
    void Disable(void);
    void sendCan(void);

    ALG::PID::PID pid_pos;
    ALG::PID::PID pid_vel;
    ALG::LADRC::Adrc adrc_vel;

    float target_pos;
    float target_vel;
};

} // namespace TASK::MotorPid

extern "C"
{
#endif // __cplusplus
    void MotorPidTask(void *argument);
#ifdef __cplusplus
}
#endif // __cplusplus

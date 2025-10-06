#include "chassis_control.hpp"

PidTarget::pid_target<PIDQuantity> ChassisPidTarget;
ChassisControl::kinematics_target KinematicsTarget;
ChassisControl::motortarget<PIDQuantity> MotorTarget;
ChassisControl::modelchoose model_choose;
void ChassisControl::modelchoose::ModeSelection()
{
    chass_model.updateState();
    if(chass_model.isStateChanged())
    {
        switch(chass_model.getLastState())
        {
            case State::model::STOP:
                
                break;
            case State::model::FOLLOW:

                break;
            case State::model::NOTFOLLOW:

                break;
            case State::model::SHOT_FOLLOW:

                break;
            case State::model::SHOT_Rotate:

                break;
            case State::model::VISION_NOTSHOT:

                break;
            case State::model::VISION_SHOT:

                break;
            case State::model::KEYBOARD:

                break;
        }
    }
    else
    {
        switch(chass_model.getCurrentState())
        {
            case State::model::STOP:
                ChassisPidTarget.stop(PIDQuantity);
                break;
            case State::model::FOLLOW:
                KinematicsTarget.follow(13.78f);
                break;
            case State::model::NOTFOLLOW:
                KinematicsTarget.nofollow(0.005512f);
                break;
            case State::model::SHOT_FOLLOW:
                KinematicsTarget.follow(13.78f);
                break;
            case State::model::SHOT_Rotate:
                KinematicsTarget.follow_rotate(13.78f);
                break;
            case State::model::VISION_NOTSHOT:
                KinematicsTarget.nofollow(0.005512f);
                break;
            case State::model::VISION_SHOT:
                KinematicsTarget.nofollow(0.005512f);
                break;
            case State::model::KEYBOARD:
                KinematicsTarget.keyboard();
                break;
        }
    }
}

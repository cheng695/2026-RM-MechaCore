#include "ControlTask.hpp"
#define PI_ 3.1415926535897932384626433832795

ALG::PowerControl::PowerControl<4> power3508;
ALG::PowerControl::PowerControl<4> power6020;
float coefficients3508[6] = { 2.144951, -0.002828, 0.000025,
                                0.016525,  0.115369, 0.000015  };//3508
float coefficients6020[6] = { 1.586024, 0.013252, 0.000229, 
                          0.530772, 7.509297, 0.000320 };//6020


Alg::Utility::SlopePlanning string_target[3] = {
    Alg::Utility::SlopePlanning(0.006f, 0.006f),
    Alg::Utility::SlopePlanning(0.006f, 0.006f),
    Alg::Utility::SlopePlanning(5.0f, 5.0f)
};

Chassis_FSM chassis_fsm;

float wheel_azimuth[4] = {7*PI_/4, PI_/4, 3*PI_/4, 5*PI_/4};
float phase_offset[4] = {-1.29999f, 3.43989f, 2.84999f, 1.32980f};
Alg::CalculationBase::String_IK string_ik(0.17f, 0.055f, wheel_azimuth, phase_offset);
Alg::CalculationBase::String_FK string_fk(0.17f, 0.055f, wheel_azimuth, phase_offset);
Alg::CalculationBase::String_ID string_id(0.17f, 0.055f, wheel_azimuth, phase_offset);

ALG::PID::PID stringAngle_pid[4] = {
    ALG::PID::PID(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 5.0f),
    ALG::PID::PID(9.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f),
    ALG::PID::PID(9.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f),
    ALG::PID::PID(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 20.0f)
};
ALG::PID::PID stringVelocity_pid[4] = {
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(80.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)
};
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 150.0f)
};

ALG::PID::PID translational_pid[2] = {
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)
};
ALG::PID::PID rotational_pid(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);

ALG::PID::PID powertest_pid(9.0f, 0.035f, 0.0f, 25000.0f, 10000.0f, 150.0f);

ControlTask chassis_target;
Output_chassis chassis_output;

void chassis_fsm_init()
{
    chassis_fsm.Init();
}

bool check_online()
{
    bool isconnected = true;
    for(int i = 0; i < 4; i++)
    {
        if(!Motor3508.isConnected(i+1, i+1) || !Motor6020.isConnected(i+1, i+5))
        {
            isconnected = false;
        }
    }

    // if(!Cboard.isConnected() || !DT7.isConnected())
    // {
    //     isconnected = false;
    // }
    
    if(!isconnected)
    {
        return false;
    }

    return true;
}

void Settarget_chassis()
{
    string_target[0].TIM_Calculate_PeriodElapsedCallback(2.702f * DT7.get_left_y(), string_fk.GetChassisVx());
    chassis_target.target_translation_x = string_target[0].GetOut();

    string_target[1].TIM_Calculate_PeriodElapsedCallback(2.702f * DT7.get_left_x(), string_fk.GetChassisVy());
    chassis_target.target_translation_y = string_target[1].GetOut();

    string_target[2].TIM_Calculate_PeriodElapsedCallback(15.88f * DT7.get_scroll_(), string_fk.GetChassisVw());
    chassis_target.target_rotation = string_target[2].GetOut();

    // chassis_target.target_translation_x = DT7.get_left_y();
    // chassis_target.target_translation_y = DT7.get_left_x();
    // chassis_target.target_rotation = DT7.get_scroll_();
}

void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        wheel_pid[i].reset();
        stringAngle_pid[i].reset();
        stringVelocity_pid[i].reset();
        
        chassis_output.out_string[i] = 0.0f;
        chassis_output.out_wheel[i] = 0.0f;
    }
}



float PowerMax = 50.0f;
float I6020[4], I3508[4], I_other[4], V6020[4], V3508[4];

void chassis_notfollow()
{
    // 设置当前舵向角
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);
        string_fk.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);
        string_id.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);
    }
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    string_fk.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 2.7f, 15.88f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算舵向输出和轮向补偿
        stringAngle_pid[i].UpDate(string_ik.GetMotor_direction(i)*57.3f, Motor6020.getAngleDeg(i+1));
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor6020.getVelocityRpm(i+1));

        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRpm(i+1));

        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput();
        //chassis_output.out_wheel[i] = wheel_pid[i].getOutput();

        // 正运动学相关 通过PID算牵引力和旋转力矩
        translational_pid[0].UpDate(chassis_target.target_translation_x, string_fk.GetChassisVx());
        translational_pid[1].UpDate(chassis_target.target_translation_y, string_fk.GetChassisVy());
        rotational_pid.UpDate(chassis_target.target_rotation, string_fk.GetChassisVw());

        // 逆动力学 算轮向电机力矩
        string_id.StringInvDynamics(translational_pid[0].getOutput(), translational_pid[1].getOutput(), rotational_pid.getOutput());

        // 轮向电机力矩转控制电流
        chassis_output.out_wheel[i] = string_id.GetMotorTorque(i) / 15.76f / 0.7f / 0.3f * 819.2f + wheel_pid[i].getOutput();
    }
    
    for(int i = 0; i < 4; i++)
    {
        I6020[i] = chassis_output.out_string[i] * 3.0f/16384.0f;
        V6020[i] = Motor6020.getVelocityRads(i+1);

        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;
        I_other[i] = 0.0f;
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f);
    }

    float pmax6020 = PowerMax*0.5f;
    float pmax3508 = PowerMax*0.5f;
    power6020.AttenuatedPower(I6020, V6020, coefficients6020, 0.0f, pmax6020);
    float PowerTotal_6020 = power6020.getPowerTotal();
    if(PowerTotal_6020 < pmax6020)
    {
        pmax3508 = PowerMax - PowerTotal_6020;
    }
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508);

    for(int i = 0; i < 4; i++)
    {
        chassis_output.out_string[i] = power6020.getCurrentCalculate(i) * 16384.0f/3.0f;
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
    }
}

void chassis_follow()
{

}

void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online);
    Settarget_chassis();

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        case KEYBOARD:
            //chassis_keyboard_control();
            break;
        default:
            chassis_stop();
            break;
    }
}

extern "C"{
void Control(void const * argument)
{
    // 初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    //chassis_fsm_init();
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());

        osDelay(1);
    } 
}
}
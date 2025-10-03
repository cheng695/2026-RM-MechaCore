#include "Buzzer.hpp"

Buzzer::C_buzzer c_buzzer;

const Buzzer::C_buzzer::BuzzerSound Buzzer::C_buzzer::buzzer_sound[4] = {
    &Buzzer::C_buzzer::m1,
    &Buzzer::C_buzzer::m2,
    &Buzzer::C_buzzer::m3,
    &Buzzer::C_buzzer::m4
};

void Buzzer::C_buzzer::Sound(int index)
{
    if(index >= 0 && index < 4)
    {
        (this->*buzzer_sound[index])();
    }
}

void Buzzer::C_buzzer::m1()
{
    myPwmDriver.setPwm(0, 2, 168);
    osDelay(50);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(200);
}

void Buzzer::C_buzzer::m2()
{
    for(int i = 0; i < 2; i++)
    {
        myPwmDriver.setPwm(0, 2, 168);
        osDelay(50);
        myPwmDriver.setPwm(0, 2, 0);
        osDelay(50);
    }
    osDelay(200);
}

void Buzzer::C_buzzer::m3()
{
    for(int i = 0; i < 3; i++)
    {
        myPwmDriver.setPwm(0, 2, 168);
        osDelay(50);
        myPwmDriver.setPwm(0, 2, 0);
        osDelay(50);
    }
    osDelay(200);
}

void Buzzer::C_buzzer::m4()
{
    for(int i = 0; i < 4; i++)
    {
        myPwmDriver.setPwm(0, 2, 168);
        osDelay(50);
        myPwmDriver.setPwm(0, 2, 0);
        osDelay(50);
    }
    osDelay(200);
}

void Buzzer::C_buzzer::imu()
{
    myPwmDriver.setPwm(0, 2, 168);
    osDelay(380);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(50);

    myPwmDriver.setPwm(0, 2, 168);
    osDelay(100);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(50);

    myPwmDriver.setPwm(0, 2, 168);
    osDelay(380);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(200);
}

void Buzzer::C_buzzer::to_bord()
{
    myPwmDriver.setPwm(0, 2, 168);
    osDelay(100);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(50);

    myPwmDriver.setPwm(0, 2, 168);
    osDelay(850);
    myPwmDriver.setPwm(0, 2, 0);
    osDelay(200);
}

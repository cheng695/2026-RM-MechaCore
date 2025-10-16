#include "User/Task/Inc/motorTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

void MotorTask(void *argument)
{
    for (;;)
    {
        auto &can_motor = CAN_MOTOR;
        Motor6020.setCAN(1000, 1);
        Motor6020.sendCAN(&can_motor);

        osDelay(1);
    }
}

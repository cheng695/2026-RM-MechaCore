#include "ChassisTask.hpp"

extern "C" {
void Task(void const * argument)
{
    for(;;)
    {
        osDelay(1);
    }
}

}

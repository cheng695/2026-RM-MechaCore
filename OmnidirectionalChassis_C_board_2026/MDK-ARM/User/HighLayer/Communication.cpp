#include "Communication.hpp"

Communication::communication Comm;

void Communication::communication::sendBoard()
{
    Cboard.TransmitDataUpdata();
}

void Communication::communication::sendTools()
{
    vofa_send(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

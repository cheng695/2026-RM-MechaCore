#include "Dr16.hpp"

Clicker::DR16 dr16;
uint8_t pData[18];
void Clicker::DR16::DR16DataUpdata(const uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
    rc.ch0  = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    rc.ch1  = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    rc.ch2  = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    rc.ch3  = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    rc.s1   = ((pData[5] >> 4) & 0x000C) >> 2;
    rc.s2   = ((pData[5] >> 4) & 0x0003);
    mouse.x = ((int16_t)pData[6])  | ((int16_t)pData[7]  << 8);
    mouse.y = ((int16_t)pData[8])  | ((int16_t)pData[9]  << 8);
    mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
    mouse.press_l = pData[12];
    mouse.press_r = pData[13];
    key.w = (pData[14] & 0x01) >> 0;
    key.s = (pData[14] & 0x02) >> 1;
    key.a = (pData[14] & 0x04) >> 2;
    key.d = (pData[14] & 0x08) >> 3;
    key.shift = (pData[14] & 0x10) >> 4;
    key.ctrl  = (pData[14] & 0x20) >> 5;
    key.q = (pData[14] & 0x40) >> 6;
    key.e = (pData[14] & 0x80) >> 7;
        
    reserved_.reserved = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8);
}

void RemoteRxData(const uint8_t* rx_buffer, uint16_t size)
{
    if(size == 18)
    {
        dr16.DR16DataUpdata(rx_buffer);
    }
}
#pragma once
#include "can.hpp"
#include "uart.hpp"


#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus
void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);
void MainCallBack();

#ifdef __cplusplus
}
#endif  // __cplusplus

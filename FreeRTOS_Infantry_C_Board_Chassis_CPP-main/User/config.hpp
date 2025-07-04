#pragma once
#include <cstdint>

constexpr uint8_t kMotorCount = 4;  //电机数量
constexpr uint8_t kpid_vel = 4;
constexpr auto kWarningPowerBuffer = 30;  //底盘功率缓冲安全值
constexpr auto kDefaultBatteryVoltage = 24.0f;
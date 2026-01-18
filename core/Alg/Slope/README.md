# Slope - 斜坡函数

## 模块作用

斜坡函数模块提供平滑过渡功能，防止控制量突变导致的机械冲击。适用于：

- 遥控器输入平滑
- 电机速度/位置平滑过渡
- 启停过程平滑化

---

## 核心文件

### SlopePlanning.hpp

斜坡/S曲线规划类。

```cpp
// S曲线规划
class SlopePlanning
{
public:
    SlopePlanning(float acc, float dt);  // acc:加速度, dt:周期

    float Plan(float current, float target);
    void SetAcceleration(float acc);
};
```

### Slope.h

传统斜坡函数。

```cpp
// 简单斜坡
void Slope_Calculate(float* output, float target, float step);
```

---

## 使用方法

### 基本斜坡

```cpp
#include "Slope.h"

float current_speed = 0;
float target_speed = 100;
float step = 5;  // 每次递增步长

// 每个控制周期调用
Slope_Calculate(&current_speed, target_speed, step);
// current_speed 逐渐从0增加到100
```

### S曲线规划

```cpp
#include "SlopePlanning.hpp"

SlopePlanning planner(100.0f, 0.001f);  // 加速度100, 周期1ms

// 控制循环
float smoothed = planner.Plan(current_value, target_value);
```

---

## 数学公式

简单斜坡：

```
if(current < target)
    current = min(current + step, target)
else
    current = max(current - step, target)
```

---

## 验证方法

1. **曲线检查**：通过 LOG 输出过渡曲线，检查是否平滑
2. **时间验证**：计算从0到目标值的时间是否符合预期
3. **极限测试**：目标值突变时，检查输出是否平滑过渡

```cpp
LOG_INFO("Slope: target=%.1f current=%.1f", target, current);
```

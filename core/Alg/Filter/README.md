# 滤波器类使用说明

## 简介

本项目提供了四种常用的数字滤波器实现，包括卡尔曼滤波器、TD跟踪微分器、一阶低通滤波器和限幅滤波器。这些滤波器可用于信号处理、传感器数据平滑、噪声抑制等应用场景。

## 滤波器类型及使用方法

### 1. 卡尔曼滤波器 ([KalmanFilter](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L5-L33))

卡尔曼滤波器是一种递归的状态估计算法，适用于存在噪声的动态系统状态估计。

#### 构造与初始化
```cpp
#include "FilterClass.hpp"

// 创建卡尔曼滤波器实例
KalmanFilter kalman(0.1f, 0.5f);  // 参数分别为Q(过程噪声协方差)和R(观测噪声协方差)
```

#### 主要方法
- [filter(float dat)](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L25-L25) - 对输入数据进行滤波处理，返回滤波后的最优估计值
- [reinit(float T_Q, float T_R)](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L28-L28) - 重新设置过程噪声和观测噪声参数
- [getState()](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L30-L30) - 获取当前状态估计值
- [getPrediction()](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L31-L31) - 获取当前预测值
- [getGain()](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L32-L32) - 获取当前卡尔曼增益

#### 使用示例
```cpp
float rawData = 100.0f;
float filteredData = kalman.filter(rawData);
float currentState = kalman.getState();
```

### 2. TD跟踪微分器 ([TDFilter](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L38-L57))

TD跟踪微分器能够快速跟踪输入信号并提供微分信号，常用于信号预处理和微分估计。

#### 构造与初始化
```cpp
#include "FilterClass.hpp"

// 创建TD滤波器实例
TDFilter td(100.0f, 0.01f);  // 参数分别为R(速度因子)和H(积分步长)
```

#### 主要方法
- `filter(float Input)` - 对输入信号进行跟踪处理，返回跟踪信号
- `setParams(float new_R, float new_H)` - 重新设置参数
- `getTrackingSignal()` - 获取跟踪信号
- `getDerivative()` - 获取微分信号

#### 使用示例
```cpp
float inputSignal = 50.0f;
float trackedSignal = td.filter(inputSignal);
float derivativeSignal = td.getDerivative();
```

### 3. 一阶低通滤波器 ([LPFFilter](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L63-L82))

一阶低通滤波器可有效滤除高频噪声，保留低频信号成分。

#### 构造与初始化
```cpp
#include "FilterClass.hpp"

// 创建低通滤波器实例
LPFFilter lpf(0.3f);  // 参数为滤波系数(0-1)，值越大响应越快
```

#### 主要方法
- `filter(float Input)` - 对输入信号进行低通滤波处理
- `setRatio(float ratio)` - 设置滤波系数
- `getOutput()` - 获取当前输出值
- `getRatio()` - 获取当前滤波系数

#### 使用示例
```cpp
float noisySignal = 75.0f;
float smoothSignal = lpf.filter(noisySignal);
float currentOutput = lpf.getOutput();
```

### 4. 限幅滤波器 (`LMFFilter`)

限幅滤波器通过限制信号的最大变化幅度来消除突发性干扰。

#### 构造与初始化
```cpp
#include "FilterClass.hpp"

// 创建限幅滤波器实例
LMFFilter lmf(5.0f);  // 参数为最大允许变化量
```

#### 主要方法
- [filter(float Input)](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L50-L50) - 对输入信号进行限幅处理
- [setLimit(float limit_ratio)](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L102-L102) - 设置限制幅度
- [getOutput()](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L80-L80) - 获取当前输出值
- [getLimitRatio()](file://c:\Users\nianyi\Desktop\git\Filter\FilterClass.hpp#L106-L106) - 获取当前限制幅度

#### 使用示例
```cpp
float sensorData = 120.0f;
float limitedData = lmf.filter(sensorData);
float outputValue = lmf.getOutput();
```

## 组合使用示例

多种滤波器可以组合使用以达到更好的滤波效果：

```cpp
// 创建各种滤波器实例
KalmanFilter kalman(0.1f, 0.5f);
TDFilter td(100.0f, 0.01f);
LPFFilter lpf(0.3f);
LMFFilter lmf(5.0f);

// 串联使用多种滤波器处理数据
float rawData = 100.0f;
float filteredData = kalman.filter(rawData);  // 卡尔曼滤波
filteredData = td.filter(filteredData);       // TD跟踪
filteredData = lpf.filter(filteredData);      // 低通滤波
filteredData = lmf.filter(filteredData);      // 限幅滤波
```

## 注意事项

1. 各滤波器参数需要根据实际应用场景进行调整
2. 卡尔曼滤波器的Q和R参数影响滤波效果：Q增大滤波器更相信观测值，R增大滤波器更相信预测值
3. LPF滤波器的Ratio参数应在0-1之间，值越大响应越快但滤波效果越弱
4. 建议在使用前充分测试各参数对具体应用的影响
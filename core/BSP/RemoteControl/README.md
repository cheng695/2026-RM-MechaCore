### DT7 遥控器解析库（RemoteController）

本模块提供对大疆 DT7（或同协议 18 字节帧）的数据解析能力，基于 STM32 HAL 串口接收回调/空闲中断回调在接收一帧后调用 `RemoteController::parseData` 完成解析，并通过一组简洁的 getter 接口获取摇杆、按键、开关、鼠标与滚轮数据。

---

## 功能概览

- **协议帧长度**: 18 字节（`PROTOCOL_LENGTH = 18`）
- **通道精度**: 每个摇杆通道 11 位（0–2047），库内做范围裁剪和原点平移
- **通道范围**:
  - `CHANNEL_VALUE_MIN = 364`
  - `CHANNEL_VALUE_MID = 1024`
  - `CHANNEL_VALUE_MAX = 1684`
- **已解析数据**:
  - 摇杆通道: `ch0..ch3`（右X、右Y、左X、左Y）
  - 拨杆开关: `s1`、`s2`（枚举 `UP=1/MIDDLE=3/DOWN=2`）
  - 鼠标: `x,y,z,left,right`
  - 键盘: 16 位位掩码（`WASD、Shift、Ctrl、QERF...`）
  - 滚轮/滑轮: `scroll`（int16）
- **坐标与离散化**:
  - 坐标系以中值为原点: `left_x/left_y/right_x/right_y = ch - 1024`
  - 离散摇杆位置: 每轴 `-1/0/1`（阈值近似 660，见 `discreteAxis`）
- **在线状态监测**: 继承 `BSP::WATCH_STATE::StateWatch`，在每次解析时更新时间戳，可用 `isConnected()`/`getDisconnectedTime()` 查询。

---

## 目录结构

- `BSP/RemoteControl/DT7.hpp`：对外头文件（类接口、常量、类型）
- `BSP/RemoteControl/DT7.cpp`：实现（解析、工具函数）
- `BSP/Common/StateWatch/state_watch.hpp|.cpp`：在线/离线状态监测基类

---

## 快速上手

### 1) 添加到工程

- 将以下源文件加入 CMake/Keil/STM32CubeIDE 工程并以 C++ 编译：
  - `BSP/RemoteControl/DT7.cpp`
  - `BSP/Common/StateWatch/state_watch.cpp`
- 包含头文件目录：
  - `BSP/RemoteControl/`
  - `BSP/Common/StateWatch/`
- 确保以 C++ 语言标准编译（`-std=c++17` 或工程默认 C++ 标准）。

### 2) 在 HAL 串口回调中调用

推荐使用两种方式之一：

- HAL 空闲中断 + DMA（优先，能自动按帧触发回调）
- HAL 接收完成回调（固定长度 18 字节）

下面分别给出示例。

#### 方案 A：空闲中断 + DMA（HAL_UARTEx_ReceiveToIdle_DMA）

```c
// main.c 或初始化处
extern UART_HandleTypeDef huartX; // 替换为实际 UART 句柄

#define DT7_FRAME_LEN 18
static uint8_t dt7_rx_buf[DT7_FRAME_LEN];

void DT7_UART_Start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huartX, dt7_rx_buf, DT7_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(huartX.hdmarx, DMA_IT_HT); // 关闭半传输中断，避免噪声回调
}

// C++ 对象（在 C 文件中声明，在 C++ 文件中定义）
// 方式1：若此文件为 C，可在头部添加 `#ifdef __cplusplus extern "C" { #endif` 包裹声明。
void DT7_OnFrameReceived(const uint8_t* data, uint16_t len);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huartX) {
        if (Size == DT7_FRAME_LEN) {
            DT7_OnFrameReceived(dt7_rx_buf, Size);
        }
        // 继续开启下一次接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huartX, dt7_rx_buf, DT7_FRAME_LEN);
        __HAL_DMA_DISABLE_IT(huartX.hdmarx, DMA_IT_HT);
    }
}
```

在一个 C++ 源文件中实现 `DT7_OnFrameReceived`，并持有 `RemoteController` 实例：

```cpp
// dt7_bridge.cpp （C++ 编译单元）
#include "BSP/RemoteControl/DT7.hpp"

static RemoteController g_dt7(100); // 超时 100ms

extern "C" void DT7_OnFrameReceived(const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len != 18) return;
    g_dt7.parseData(data);
}

// 对外提供查询函数（供 C 环境调用，可选）
extern "C" int8_t DT7_GetLeftX(void) { return g_dt7.get_stick_left_x(); }
extern "C" uint8_t DT7_GetS1(void) { return g_dt7.get_s1(); }
```

#### 方案 B：固定长度接收完成回调（HAL_UART_Receive_DMA）

```c
extern UART_HandleTypeDef huartX;
static uint8_t dt7_rx_buf[18];

void DT7_UART_Start(void)
{
    HAL_UART_Receive_DMA(&huartX, dt7_rx_buf, sizeof(dt7_rx_buf));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huartX) {
        DT7_OnFrameReceived(dt7_rx_buf, sizeof(dt7_rx_buf));
        // 重新启动 DMA 接收
        HAL_UART_Receive_DMA(&huartX, dt7_rx_buf, sizeof(dt7_rx_buf));
    }
}
```

> 注意：若实际硬件帧间隔或抖动导致粘包/拆包，优先采用空闲中断方案；若链路稳定且每帧恰好 18 字节，固定长度也可行。

---

## API 使用说明

在完成接收回调中调用 `parseData(const uint8_t* data)` 后，可通过以下接口读取数据：

- **通道与坐标**
  - `get_ch0..get_ch3()`: 原始通道（裁剪至 [364,1684]）
  - `get_leftX/get_leftY/get_rightX/get_rightY()`: 已以 1024 为原点的坐标
- **拨杆开关**
  - `get_s1()/get_s2()`：原值 1/2/3，可结合 `SwitchPosition` 枚举判断
- **键盘**
  - `get_key(Keyboard key)`：例如 `KEY_W/KEY_SHIFT/...`
- **鼠标/滚轮**
  - `get_mouseLeft()/get_mouseRight()` 返回布尔值
  - `get_scroll()` 返回滚轮/滑轮值（int16）
- **离散摇杆位置**
  - `get_stick_left_x/y/right_x/y()`：每轴 `-1/0/1`
- **连接状态**
  - `isConnected()`：是否在线
  - `getDisconnectedTime()`：离线持续时间（ms）

---

## 解析细节（简述）

- 通道 0..3 使用 11 位紧凑打包，从第 0、11、22、33 位开始提取
- `s1`、`s2` 使用 2 位
- 鼠标 `x,y,z` 与 `left/right` 按字节字段组合
- 键盘为 16 位位域
- 离散化逻辑：以 660 为近似单位步长，提供容差判断与边界夹取

对应实现可参见 `BSP/RemoteControl/DT7.cpp` 中：

```startLine:endLine:BSP/RemoteControl/DT7.cpp
// 解析原始 18 字节数据（提取通道/开关/鼠标/键盘并更新坐标与时间戳）
void RemoteController::parseData(const uint8_t *data)
{
    // ... more code ...
    channels_.ch0 = mapChannelValue(extractBits(data, 0, 11));
    channels_.ch1 = mapChannelValue(extractBits(data, 11, 11));
    channels_.ch2 = mapChannelValue(extractBits(data, 22, 11));
    channels_.ch3 = mapChannelValue(extractBits(data, 33, 11));
    channels_.s1 = extractBits(data, 44, 2);
    channels_.s2 = extractBits(data, 46, 2);
    // ... more code ...
}
```

---

## 与 C/HAL 工程的互操作建议

- 由于回调多在 C 文件中，建议在 C++ 源文件中定义 `RemoteController` 实例，并对外暴露 `extern "C"` 桥接函数供 C 调用（见示例）。
- 若使用 RTOS，回调内仅做轻量解析，应用层通过队列/事件/轮询读取 getter，避免长阻塞。
- 若需要线程安全，可在外层加互斥访问；库本身不持有锁。

---

## 常见问题（FAQ）

- 数据帧长度不对？请确认 DMA 长度或空闲中断回调 `Size` 恰为 18。
- 出现拆包/粘包？优先使用空闲中断 `HAL_UARTEx_ReceiveToIdle_DMA`，并确保每帧只调用一次解析。
- 无法判断在线状态？请确保接收后调用了 `parseData`（内部会 `updateTimestamp()`）。

---

## 最小可用示例（汇总）

```c
// main.c
int main(void)
{
    // ... HAL/时钟/串口初始化 ...
    DT7_UART_Start();
    while (1) {
        // 任务循环，必要时查询状态
        // 例如：if (DT7_GetS1() == 1) { /* do something */ }
    }
}
```

```cpp
// dt7_bridge.cpp
#include "BSP/RemoteControl/DT7.hpp"

static RemoteController g_dt7(100);

extern "C" void DT7_OnFrameReceived(const uint8_t* data, uint16_t len)
{
    if (data && len == 18) g_dt7.parseData(data);
}

// 也可在 C++ 中直接使用：
// auto lx = g_dt7.get_leftX();
// bool w  = g_dt7.get_key(RemoteController::KEY_W);
```

---

## 许可证与致谢

本模块为项目内部组件。若引用或改造，请在文档中保留来源说明。



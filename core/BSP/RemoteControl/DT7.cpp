#include "DT7.hpp"
#include <cstdlib>

// ============================================================================
// 主要逻辑：构造与数据解析
// ============================================================================

// 构造函数：初始化基类与成员
RemoteController::RemoteController(uint32_t timeout_ms)
    : BSP::WATCH_STATE::StateWatch(timeout_ms), channels_({0}), mouse_({0}), keyboard_(0)
{
}

// 解析原始 18 字节数据（提取通道/开关/鼠标/键盘并更新坐标与时间戳）
void RemoteController::parseData(const uint8_t *data)
{
    if (data == nullptr)
        return;

    updateTimestamp();

    // 通道（11位）
    channels_.ch0 = mapChannelValue(extractBits(data, 0, 11));
    channels_.ch1 = mapChannelValue(extractBits(data, 11, 11));
    channels_.ch2 = mapChannelValue(extractBits(data, 22, 11));
    channels_.ch3 = mapChannelValue(extractBits(data, 33, 11));
    channels_.s1 = extractBits(data, 44, 2);
    channels_.s2 = extractBits(data, 46, 2);
    channels_.scroll = extract16Bits(data[16], data[17]); // 解析滚轮/滑轮值

    // 坐标（以中值为中心）
    coordinates_.left_x = channels_.ch2 - CHANNEL_VALUE_MID;
    coordinates_.left_y = channels_.ch3 - CHANNEL_VALUE_MID;
    coordinates_.right_x = channels_.ch0 - CHANNEL_VALUE_MID;
    coordinates_.right_y = channels_.ch1 - CHANNEL_VALUE_MID;

    // 鼠标（按字节组合）
    mouse_.x = extract16Bits(data[6], data[7]);
    mouse_.y = extract16Bits(data[8], data[9]);
    mouse_.z = extract16Bits(data[10], data[11]);
    mouse_.left = extractBool(data[12], 0);
    mouse_.right = extractBool(data[12], 1);

    // 键盘（16位）
    keyboard_ = extract16Bits(data[14], data[15]);

    // 计算并更新离散摇杆位置（分别赋值四个轴）
    stick_position_.stick_left_x = discreteAxis(coordinates_.left_x, 0);
    stick_position_.stick_left_y = discreteAxis(coordinates_.left_y, 0);
    stick_position_.stick_right_x = discreteAxis(coordinates_.right_x, 0);
    stick_position_.stick_right_y = discreteAxis(coordinates_.right_y, 0);
    // stick_position_.scroll = discreteAxis(channels_.scroll, 0);//这个不知道是不是660，待定
}

// ============================================================================
// 外部接口：供模块外部调用的 API
// ============================================================================

// 外部接口由头文件 inline get_xxx 提供

// ============================================================================
// 内部辅助函数：工具与解析实现（模块内部使用）
// ============================================================================

// 离散化轴值辅助函数（文件作用域）
static inline int8_t discreteAxis(int16_t value, int16_t tolerance)
{
    // 基于容差的邻域判定：靠近 0 / ±660 则直接归类
    const int32_t v = static_cast<int32_t>(value);
    if (std::abs(v - 0) < tolerance)
        return 0;
    if (std::abs(v - 660) < tolerance)
        return 1;
    if (std::abs(v + 660) < tolerance)
        return -1;

    // 未命中容差邻域时，整除 660，得到 -1/0/1，并做边界夹取
    int32_t q = v / 660;
    if (q > 1)
        q = 1;
    else if (q < -1)
        q = -1;
    return static_cast<int8_t>(q);
}

// 按位提取（跨字节）
uint16_t RemoteController::extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const
{
    uint16_t result = 0;
    uint32_t currentByte = startBit / 8;
    uint8_t bitOffset = startBit % 8;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t currentBit = (data[currentByte] >> bitOffset) & 0x01;
        result |= (currentBit << i);

        bitOffset++;
        if (bitOffset == 8)
        {
            bitOffset = 0;
            currentByte++;
        }
    }

    return result;
}

// 合并两个字节为 int16
int16_t RemoteController::extract16Bits(const uint8_t low_byte, const uint8_t high_byte) const
{
    return static_cast<int16_t>(low_byte | (high_byte << 8));
}

// 提取字节中指定位为 bool
bool RemoteController::extractBool(const uint8_t byte, uint8_t bit_position) const
{
    return (byte >> bit_position) & 0x01;
}

// 限制通道值在有效范围
int16_t RemoteController::mapChannelValue(uint16_t value) const
{
    return std::min(std::max(static_cast<int16_t>(value),
                             static_cast<int16_t>(CHANNEL_VALUE_MIN)),
                    static_cast<int16_t>(CHANNEL_VALUE_MAX));
}

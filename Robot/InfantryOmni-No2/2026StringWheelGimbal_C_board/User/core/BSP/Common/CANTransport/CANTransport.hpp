#ifndef CANTRANSPORT_HPP
#define CANTRANSPORT_HPP

#include "core/HAL/CAN/can_hal.hpp"
#include <cstring>

/**
 * @brief CAN 多帧分包传输层
 *
 * 用于解决 CAN 单帧 8 字节限制，支持最大 112 字节数据包的分包发送和接收重组。
 *
 * 帧格式 (每帧 8 字节):
 *   Data[0] = (seq << 4) | (total_frames - 1)
 *     - 高 4 位: 当前帧序号 (0~15)
 *     - 低 4 位: 总帧数 - 1 (0~15, 即 1~16 帧)
 *   Data[1..7]: 7 字节有效载荷 (最后一帧不足 7 字节时补零)
 *
 * 最大有效载荷: 16 帧 × 7 字节 = 112 字节
 */

namespace BSP::CANTransport
{

static constexpr size_t kPayloadPerFrame = 7;
static constexpr size_t kMaxFrames = 16;
static constexpr size_t kMaxPayload = kPayloadPerFrame * kMaxFrames; // 112
static constexpr uint32_t kTimeoutMs = 50;

// ──── 发送 ──────────────────────────────────────────────────────────

/**
 * @brief 将数据分包并通过 CAN 发送
 *
 * @param can      CAN 设备 (can1 或 can2)
 * @param base_id  所有分片帧共用的 CAN ID
 * @param data     数据指针
 * @param len      数据长度 (必须 ≤ 112)
 * @return         实际发送的帧数 (0 表示失败)
 */
inline uint8_t sendPacket(HAL::CAN::ICanDevice &can, uint32_t base_id, const void *data, size_t len)
{
    if (len == 0 || len > kMaxPayload)
        return 0;
    if (base_id > 0x7FF)
        return 0; // 标准帧 ID 上限

    const auto *src = static_cast<const uint8_t *>(data);
    const uint8_t total = static_cast<uint8_t>((len + kPayloadPerFrame - 1) / kPayloadPerFrame);
    uint8_t sent = 0;

    for (uint8_t seq = 0; seq < total; ++seq)
    {
        HAL::CAN::Frame frame{};
        frame.id = base_id;
        frame.dlc = 8;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        frame.data[0] = static_cast<uint8_t>((seq << 4) | (total - 1));

        const size_t offset = seq * kPayloadPerFrame;
        const size_t remain = len - offset;
        const size_t copy_bytes = (remain >= kPayloadPerFrame) ? kPayloadPerFrame : remain;
        std::memcpy(&frame.data[1], src + offset, copy_bytes);
        if (copy_bytes < kPayloadPerFrame)
            std::memset(&frame.data[1 + copy_bytes], 0, kPayloadPerFrame - copy_bytes);

        if (can.send(frame))
            ++sent;
        else
        {
            // 邮箱满则等待 (~130μs/帧 @1Mbps), 最多 ~500μs
            uint32_t retry = 50000;
            while (!can.send(frame) && --retry);
            if (retry > 0) ++sent;
        }
    }
    return sent;
}

// ──── 接收 ──────────────────────────────────────────────────────────

/**
 * @brief 接收重组缓冲区
 *
 * 每个数据流（一个 CAN ID 方向）创建一个实例即可。
 *
 * 用法：
 *   1. 在 CAN 接收回调中调用 feed(frame)
 *   2. feed() 返回 true 时，通过 data()/len() 获取完整数据包
 *   3. 处理完后调用 reset() 准备下一包
 */
class RxBuffer
{
  public:
    /**
     * @brief 喂入一帧 CAN 数据
     * @return true = 已收齐所有帧，可调用 data()/len() 获取
     */
    bool feed(const HAL::CAN::Frame &frame)
    {
        const uint8_t seq = (frame.data[0] >> 4) & 0x0F;
        const uint8_t total_minus_1 = frame.data[0] & 0x0F;
        const uint8_t total = total_minus_1 + 1;

        if (static_cast<size_t>(total) > kMaxFrames || seq >= total)
            return false;

        const uint32_t now = HAL_GetTick();

        // 超时或新包 → 重置
        if (complete_ || (received_mask_ != 0 && static_cast<uint32_t>(now - last_tick_) > kTimeoutMs))
            reset();

        if (received_mask_ == 0)
        {
            // 第一帧到达
            total_frames_ = total;
            last_tick_ = now;
        }
        else if (total != total_frames_)
        {
            // 帧数不一致（可能是新包）→ 丢弃旧包，开始新包
            reset();
            total_frames_ = total;
            last_tick_ = now;
        }

        // 已收到此序号，跳过重复帧
        if (received_mask_ & (1 << seq))
            return false;

        last_tick_ = now;

        // 拷贝有效载荷
        const size_t offset = seq * kPayloadPerFrame;
        std::memcpy(&buffer_[offset], &frame.data[1], kPayloadPerFrame);
        received_mask_ |= static_cast<uint16_t>(1 << seq);

        // 检查是否收齐
        const uint16_t expected = (static_cast<uint16_t>(1) << total) - 1;
        if (received_mask_ == expected)
        {
            complete_ = true;
            return true;
        }
        return false;
    }

    /// 重组后的数据缓冲区
    const uint8_t *data() const { return buffer_; }

    /// 数据长度 (= 总帧数 × 7，含末帧填充零)
    size_t len() const { return static_cast<size_t>(total_frames_) * kPayloadPerFrame; }

    /// 是否已完整收齐
    bool complete() const { return complete_; }

    /// 重置缓冲区，准备接收下一包
    void reset()
    {
        received_mask_ = 0;
        total_frames_ = 0;
        complete_ = false;
    }

  private:
    uint8_t buffer_[kMaxPayload]{};
    uint16_t received_mask_ = 0;
    uint8_t total_frames_ = 0;
    uint32_t last_tick_ = 0;
    bool complete_ = false;
};

} // namespace BSP::CANTransport

#endif

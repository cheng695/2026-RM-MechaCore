#include "can_device_impl.hpp"

#include <algorithm>

namespace HAL::CAN
{

// CanDevice实现
CanDevice::CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo)
    : handle_(handle), filter_bank_(filter_bank), fifo_(fifo), mailbox_(0)
{
}

void CanDevice::init()
{
    configure_filter();
}

void CanDevice::start()
{
    HAL_CAN_Start(handle_);

    // 设置中断
    if (fifo_ == CAN_FILTER_FIFO0)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (fifo_ == CAN_FILTER_FIFO1)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

bool CanDevice::send(const Frame &frame)
{
    /* 先发送已经排队的高优先级帧，避免普通帧插队。 */
    process_tx_queue();

    if (send_now(frame))
    {
        return true;
    }

    const uint8_t frame_priority = get_frame_priority(frame);

    /* 控制帧过期后没有意义，因此同 ID 帧直接覆盖为最新值。 */
    for (auto &pending : tx_queue_)
    {
        if (pending.id == frame.id &&
            pending.is_extended_id == frame.is_extended_id)
        {
            /* 安全帧不能被普通控制帧覆盖。 */
            if (get_frame_priority(pending) <= frame_priority)
            {
                pending = frame;
            }
            return true;
        }
    }

    if (tx_queue_.size() >= TX_QUEUE_MAX)
    {
        /* 队列只保存最新控制状态，满时丢弃最旧的普通帧。 */
        tx_queue_.pop_front();
    }
    auto insert_position = std::find_if(
        tx_queue_.begin(), tx_queue_.end(),
        [this, &frame](const Frame &pending)
        {
            return get_frame_priority(pending) > get_frame_priority(frame);
        });
    tx_queue_.insert(insert_position, frame);
    return true;
}

uint8_t CanDevice::get_frame_priority(const Frame &frame) const
{
    /* 达妙命令帧：FC=使能，FD=失能，FB=清错误，均需要优先发送。 */
    if (frame.dlc == 8)
    {
        bool prefix_is_ff = true;
        for (uint8_t i = 0; i < 7; ++i)
        {
            prefix_is_ff = prefix_is_ff && (frame.data[i] == 0xFF);
        }
        if (prefix_is_ff &&
            (frame.data[7] == 0xFB || frame.data[7] == 0xFC || frame.data[7] == 0xFD))
        {
            return 0;
        }
    }

    return 1;
}

bool CanDevice::send_now(const Frame &frame)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0)
    {
        return false;
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = frame.dlc;
    tx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    uint32_t temp_mailbox = frame.mailbox;

    if (frame.is_extended_id)
    {
        tx_header.ExtId = frame.id;
        tx_header.StdId = 0;
    }
    else
    {
        tx_header.StdId = frame.id;
        tx_header.ExtId = 0;
    }

    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(handle_, &tx_header, const_cast<uint8_t *>(frame.data), &temp_mailbox) != HAL_OK)
    {
        return false;
    }

    return true;
}

void CanDevice::process_tx_queue()
{
    while (!tx_queue_.empty() &&
           HAL_CAN_GetTxMailboxesFreeLevel(handle_) > 0)
    {
        if (!send_now(tx_queue_.front()))
        {
            break;
        }
        tx_queue_.pop_front();
    }
}

bool CanDevice::receive(Frame &frame)
{
    CAN_RxHeaderTypeDef rx_header;

    if (HAL_CAN_GetRxFifoFillLevel(handle_, fifo_) == 0)
    {
        return false;
    }

    if (HAL_CAN_GetRxMessage(handle_, fifo_, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    // 填充Frame结构体
    frame.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
    frame.dlc = rx_header.DLC;
    frame.is_extended_id = (rx_header.IDE == CAN_ID_EXT);
    frame.is_remote_frame = (rx_header.RTR == CAN_RTR_REMOTE);

    // 自动触发所有注册的回调函数
    trigger_rx_callbacks(frame);

    return true;
}

CAN_HandleTypeDef *CanDevice::get_handle() const
{
    return handle_;
}

void CanDevice::configure_filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE; // 使能过滤器
    filter.FilterBank = filter_bank_;            // 通道
    filter.FilterFIFOAssignment = fifo_;         // 缓冲器
    filter.FilterIdHigh = 0x0;                   // 高16
    filter.FilterIdLow = 0x0;                    // 低16
    filter.FilterMaskIdHigh = 0x0;               // 高16
    filter.FilterMaskIdLow = 0x0;                // 低16
    filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(handle_, &filter);
}

void CanDevice::register_rx_callback(RxCallback callback)
{
    if (callback)
    {
        rx_callbacks_.push_back(callback);
    }
}

void CanDevice::trigger_rx_callbacks(const Frame &frame)
{
    for (auto &callback : rx_callbacks_)
    {
        if (callback)
        {
            callback(frame);
        }
    }
}

} // namespace HAL::CAN

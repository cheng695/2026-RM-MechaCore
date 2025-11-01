#ifndef DT7_HPP
#define DT7_HPP

#pragma once

// =======================================================================================================
// 头文件包含
// =======================================================================================================
#include "BSP/Common/StateWatch/state_watch.hpp"
#include <stdint.h>
#include <algorithm>

#define DT7_LIB_VERSION "0.1.0"

// 遥控器控制器，解析 DT7 数据并提供访问接口
class RemoteController : public BSP::WATCH_STATE::StateWatch
{
public:
	// ======================================================
	// 类型定义（公有）
	// ======================================================
	// 通道数据（摇杆与开关）
	struct Channels
	{
		int16_t ch0;	// 通道0（右摇杆X）
		int16_t ch1;	// 通道1（右摇杆Y）
		int16_t ch2;	// 通道2（左摇杆X）
		int16_t ch3;	// 通道3（左摇杆Y）
		uint8_t s1;		// 开关S1
		uint8_t s2;		// 开关S2
		int16_t scroll; // 滚轮/滑轮值
	};

	// 坐标（以中值为中心）
	struct Coordinates
	{
		int16_t left_x;	 // 左摇杆X坐标（已减去中值）
		int16_t left_y;	 // 左摇杆Y坐标（已减去中值）
		int16_t right_x; // 右摇杆X坐标（已减去中值）
		int16_t right_y; // 右摇杆Y坐标（已减去中值）
	};

	// 摇杆位置（每轴 -1/0/1）
	struct StickPosition
	{
		int8_t stick_left_x;
		int8_t stick_left_y;
		int8_t stick_right_x;
		int8_t stick_right_y;
		int8_t scroll;
	};

	// 鼠标数据
	struct Mouse
	{
		int16_t x;	   // X轴移动量
		int16_t y;	   // Y轴移动量
		int16_t z;	   // Z轴移动量（滚轮）
		uint8_t left;  // 左键状态
		uint8_t right; // 右键状态
	};

	// 开关位置枚举
	enum SwitchPosition
	{
		UP = 1,		// 上
		MIDDLE = 3, // 中
		DOWN = 2	// 下
	};

	// 键盘按键位掩码
	enum Keyboard
	{
		KEY_W = 0x01,	  // W键
		KEY_S = 0x02,	  // S键
		KEY_A = 0x04,	  // A键
		KEY_D = 0x08,	  // D键
		KEY_SHIFT = 0x10, // Shift键
		KEY_CTRL = 0x20,  // Ctrl键
		KEY_Q = 0x40,	  // Q键
		KEY_E = 0x80,	  // E键
		KEY_R = 0x100,	  // R键
		KEY_F = 0x200,	  // F键
		KEY_G = 0x400,	  // G键
		KEY_Z = 0x800,	  // Z键
		KEY_X = 0x1000,	  // X键
		KEY_C = 0x2000,	  // C键
		KEY_V = 0x4000,	  // V键
		KEY_B = 0x8000	  // B键
	};

	// ======================================================
	// 构造 / 析构
	// ======================================================

	RemoteController(uint32_t timeout_ms = 100);
	~RemoteController() = default;

	// ======================================================
	// 核心函数：数据解析入口
	// ======================================================

	void parseData(const uint8_t *data); // 解析接收到的原始数据

	// ======================================================
	// 精简对外接口（仅保留这 6 个外部接口）
	// ======================================================

	// 通道数据（含滚轮）
	inline int16_t get_ch0() const { return channels_.ch0; }
	inline int16_t get_ch1() const { return channels_.ch1; }
	inline int16_t get_ch2() const { return channels_.ch2; }
	inline int16_t get_ch3() const { return channels_.ch3; }
	inline int16_t get_scroll() const { return channels_.scroll; }
	// 坐标数据
	inline int16_t get_leftX() const { return coordinates_.left_x; }
	inline int16_t get_leftY() const { return coordinates_.left_y; }
	inline int16_t get_rightX() const { return coordinates_.right_x; }
	inline int16_t get_rightY() const { return coordinates_.right_y; }
	// 鼠标数据
	inline bool get_mouseLeft() const { return mouse_.left; }
	inline bool get_mouseRight() const { return mouse_.right; }
	// 开关数据
	inline uint8_t get_s1() const { return channels_.s1; } // S1开关
	inline uint8_t get_s2() const { return channels_.s2; } // S2开关
	// 键盘数据
	inline bool get_key(Keyboard key) const { return (keyboard_ & static_cast<uint16_t>(key)) != 0; }
	// 摇杆数据
	inline int8_t get_stick_left_x() const { return stick_position_.stick_left_x; }
	inline int8_t get_stick_left_y() const { return stick_position_.stick_left_y; }
	inline int8_t get_stick_right_x() const { return stick_position_.stick_right_x; }
	inline int8_t get_stick_right_y() const { return stick_position_.stick_right_y; }

	// ======================================================
	// 设备连接状态查询（保留）
	// ======================================================
	
	bool isConnected() const { return getStatus() == BSP::WATCH_STATE::Status::ONLINE; }
	uint32_t getDisconnectedTime() const { return getOfflineTime(); }

private:
	// ======================================================
	// 常量（私有）
	// ======================================================
	static constexpr uint16_t CHANNEL_VALUE_MAX = 1684;
	static constexpr uint16_t CHANNEL_VALUE_MID = 1024;
	static constexpr uint16_t CHANNEL_VALUE_MIN = 364;
	static constexpr uint8_t PROTOCOL_LENGTH = 18;

	// ======================================================
	// 内部辅助（私有）
	// ======================================================
	uint16_t extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const; // 按位提取（跨字节）
	int16_t extract16Bits(const uint8_t low_byte, const uint8_t high_byte) const;		// 合并两个字节为 int16
	bool extractBool(const uint8_t byte, uint8_t bit_position) const;					// 提取字节中指定位为 bool
	int16_t mapChannelValue(uint16_t value) const;										// 限制通道值在有效范围

	// ======================================================
	// 成员变量（私有）
	// ======================================================
	Channels channels_;		  // 通道数据
	Coordinates coordinates_; // 坐标数据
	Mouse mouse_;			  // 鼠标数据
	uint16_t keyboard_;		  // 键盘数据
	StickPosition stick_position_; // 摇杆位置（-1/0/1）
};

#endif // DT7_HPP
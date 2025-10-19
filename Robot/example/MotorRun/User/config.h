#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "HAL/DWT/DWT.hpp"
#include "HAL/LOGGER/logger.hpp"
/**
 * @brief 电机实例
 * 模板内的参数为电机的总数量，这里为假设有两个电机
 * 构造函数的第一个参数为初始ID，第二个参数为电机ID列表,第三个参数是发送的ID
 *
 */
inline BSP::Motor::Dji::GM2006<1> Motor2006(0x200, {1}, 0x200);
inline BSP::Motor::Dji::GM3508<2> Motor3508(0x200, {2, 3}, 0x200);
inline BSP::Motor::Dji::GM6020<4> Motor6020(0x204, {1, 2, 3, 4}, 0x1FF);

#define CAN_INSTANCE HAL::CAN::get_can_bus_instance()
#define LOG HAL::LOGGER::Logger::getInstance()
#define DWTimer HAL::DWTimer::getInstance(168)

#define CHASSIS_CAN HAL::CAN::CanDeviceId::HAL_Can1

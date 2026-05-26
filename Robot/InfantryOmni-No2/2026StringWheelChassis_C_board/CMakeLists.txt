cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME 2026StringWheelChassis_C_board)
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
# MCU flags (applied to all languages via CMAKE_*_FLAGS)
# ============================================================
set(MCU_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp -MMD -MP")

# ============================================================
# Global include directories
# ============================================================
set(GLOBAL_INC
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# ============================================================
# User include directories
# ============================================================
set(USER_INC
    User
    User/Task
    User/core
    User/core/APP/Referee
    User/core/Alg/ADRC
    User/core/Alg/ChassisCalculation
    User/core/Alg/Feedforward
    User/core/Alg/Filter
    User/core/Alg/FSM
    User/core/Alg/PID
    User/core/Alg/PowerControl
    User/core/Alg/PowerControl-TestVersion
    User/core/Alg/UtilityFunction
    User/core/BSP/Common/FiniteStateMachine
    User/core/BSP/Common/StateWatch
    User/core/BSP/IMU
    User/core/BSP/Motor
    User/core/BSP/Motor/DM
    User/core/BSP/Motor/Dji
    User/core/BSP/Motor/LK
    User/core/BSP/RemoteControl
    User/core/BSP/SimpleKey
    User/core/HAL/ASSERT
    User/core/HAL/CAN
    User/core/HAL/CAN/impl
    User/core/HAL/CAN/interface
    User/core/HAL/DWT
    User/core/HAL/LOGGER
    User/core/HAL/LOGGER/SEGGER/Config
    User/core/HAL/LOGGER/SEGGER/RTT
    User/core/HAL/PWM
    User/core/HAL/UART
    User/core/HAL/UART/impl
    User/core/HAL/UART/interface
)

# ============================================================
# Global macro definitions
# ============================================================
set(GLOBAL_DEFS
    USE_HAL_DRIVER
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# ============================================================
# CubeMX application sources (Core/)
# ============================================================
set(CORE_SRC
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/can.c
    Core/Src/dma.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_hal_timebase_tim.c
    Core/Src/system_stm32f4xx.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    startup_stm32f407xx.s
)

# ============================================================
# STM32 HAL driver sources
# ============================================================
set(HAL_SRC
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# ============================================================
# FreeRTOS sources
# ============================================================
set(FREERTOS_SRC
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# ============================================================
# User application sources
# ============================================================
set(USER_SRC
    User/Task/Callback.cpp
    User/Task/CommunicationTask.cpp
    User/Task/ControlTask.cpp
    User/Task/MotorTask.cpp
    User/Task/SerialTask.cpp
    User/Task/UITask.cpp
    User/core/APP/Referee/RM_RefereeSystem.cpp
    User/core/APP/Referee/RM_RefereeSystemCRC.cpp
    User/core/Alg/ADRC/adrc.cpp
    User/core/Alg/Feedforward/Feedforward.cpp
    User/core/Alg/Filter/Filter.cpp
    User/core/Alg/FSM/alg_fsm.cpp
    User/core/Alg/PID/pid.cpp
    User/core/Alg/UtilityFunction/SlopPlanning.cpp
    User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.cpp
    User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.cpp
    User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.cpp
    User/core/BSP/Common/StateWatch/buzzer_manager.cpp
    User/core/BSP/Common/StateWatch/state_watch.cpp
    User/core/BSP/RemoteControl/DT7.cpp
    User/core/HAL/ASSERT/assert.cpp
    User/core/HAL/CAN/impl/can_bus_impl.cpp
    User/core/HAL/CAN/impl/can_device_impl.cpp
    User/core/HAL/CAN/interface/can_bus.cpp
    User/core/HAL/CAN/interface/can_device.cpp
    User/core/HAL/DWT/DWT.cpp
    User/core/HAL/LOGGER/logger.cpp
    User/core/HAL/UART/impl/uart_bus_impl.cpp
    User/core/HAL/UART/impl/uart_device_impl.cpp
    User/core/HAL/UART/interface/uart_bus.cpp
    User/core/HAL/LOGGER/SEGGER/RTT/SEGGER_RTT.c
    User/core/HAL/LOGGER/SEGGER/RTT/SEGGER_RTT_printf.c
)

# ============================================================
# Create executable
# ============================================================
add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    ${CORE_SRC}
    ${HAL_SRC}
    ${FREERTOS_SRC}
    ${USER_SRC}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GLOBAL_INC}
    ${USER_INC}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ${GLOBAL_DEFS}
)

# ============================================================
# Linker options
# ============================================================
target_link_options(${PROJECT_NAME} PRIVATE
    "SHELL:${MCU_FLAGS}"
    -T${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld
    --specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

target_link_libraries(${PROJECT_NAME} PRIVATE m)

# ============================================================
# Post-build: generate .hex and .bin
# ============================================================
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating .hex and .bin files"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES "${PROJECT_NAME}.map;${PROJECT_NAME}.hex;${PROJECT_NAME}.bin"
)

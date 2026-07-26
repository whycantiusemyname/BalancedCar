#ifndef BLUETOOTH_PROTOCOL_H
#define BLUETOOTH_PROTOCOL_H

/**
 * @file BluetoothProtocol.h
 * @brief 将蓝牙DMA字节流转换成领域命令，并把遥测编码成文本或数据帧。
 */

#include <stdbool.h>
#include <stdint.h>

#include "BalanceCarTypes.h"
#include "BlueToothSerial.h"
#include "stm32f1xx_hal.h"

#ifndef BLUETOOTH_PROTOCOL_FRAME_BUFFER_SIZE
#define BLUETOOTH_PROTOCOL_FRAME_BUFFER_SIZE 64U
#endif

typedef struct
{
    /** DMA字节流仍由BSP拥有，本层只保存一条尚未组装完的方括号帧。 */
    BSP_BluetoothSerial *serial;
    char frame_buffer[BLUETOOTH_PROTOCOL_FRAME_BUFFER_SIZE];
    uint16_t frame_length;
    bool receiving_frame;
    bool frame_overflow;
    bool binary_telemetry_enabled;
    bool parameters_pending;
    uint16_t telemetry_sequence;
} BluetoothProtocol;

/** 60ms遥测快照；App只填当前数据，协议层负责定点压缩和CRC。 */
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t state;
    uint8_t flags;
    float pitch_deg;
    float accel_pitch_deg;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float yaw_bias_dps;
    float left_speed_counts_per_s;
    float right_speed_counts_per_s;
    float forward_speed_counts_per_s;
    float turn_speed_counts_per_s;
    float target_speed_counts_per_s;
    float target_pitch_deg;
    float target_yaw_rate_dps;
    float speed_integral_pitch_deg;
    float balance_output;
    float turn_output;
    float left_motor_command;
    float right_motor_command;
} BalanceCar_TelemetrySample;

/** 当前在线参数；进入二进制模式及参数变化后发送一次。 */
typedef struct
{
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float turn_kp;
    float turn_ki;
    float turn_kd;
    float balance_trim_deg;
    float motor_deadzone_offset;
    float speed_integral_limit_deg;
    float target_pitch_limit_deg;
    float motor_output_limit;
    float turn_output_limit;
    float turn_integral_limit;
    float joystick_speed_limit;
    float joystick_yaw_rate_limit_dps;
    float command_timeout_ms;
    float motor_deadzone_band;
    float position_hold_kp;
} BalanceCar_TelemetryParameters;

void BluetoothProtocol_Init(BluetoothProtocol *protocol,
                            BSP_BluetoothSerial *serial);

/**
 * @brief 非阻塞地消费现有接收字节，最多返回一条完整命令。
 * @return 得到命令返回true；没有完整命令或丢弃坏帧时返回false。
 * @note 与原教程上位机一致，以 '[' 开始、以 ']' 结束，不依赖换行符。
 */
bool BluetoothProtocol_Poll(
    BluetoothProtocol *protocol,
    BalanceCar_Command *command);

/**
 * @brief 启动一次非阻塞DMA控制曲线发送；串口忙时返回 HAL_BUSY。
 * @param plot_value_1 第一条曲线的显示值。
 * @param plot_value_2 第二条曲线的显示值。
 * @param plot_value_3 第三条曲线的显示值。
 */
HAL_StatusTypeDef BluetoothProtocol_SendTelemetry(
    BluetoothProtocol *protocol,
    const BalanceCar_TelemetrySample *sample,
    const BalanceCar_TelemetryParameters *parameters);

#endif /* BLUETOOTH_PROTOCOL_H */

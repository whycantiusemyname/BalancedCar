//
// Created by Nana Daiba on 2026/7/20.
//

#ifndef BALANCECAR_MOTOR_H
#define BALANCECAR_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @file Motor.h
 * @brief TB6612FNG 单路直流电机驱动接口。
 *
 * 上层使用 -1000 到 +1000 的有符号命令，不需要知道 TIM 的 ARR。
 * 命令符号决定 H 桥方向，绝对值决定 PWM 占空比。左右电机因安装
 * 方向相反时，可通过初始化参数 inverted 统一“正值=小车前进”。
 *
 * 当前底板将 TB6612FNG 的 STBY 硬件上拉到 3.3 V，因此本驱动不
 * 管理待机引脚。GPIO 与定时器仍由 CubeMX 负责初始化。
 */

/** 归一化电机命令的最大绝对值。 */
#define BSP_MOTOR_COMMAND_MAX 1000

/**
 * @brief 一路电机所需的 PWM 与 H 桥方向资源。
 *
 * 所有硬件句柄均由 CubeMX 创建，BSP_Motor 只保存非拥有型引用。
 */
typedef struct
{
    TIM_HandleTypeDef *pwm_timer; /**< 产生 PWM 的 HAL 定时器句柄。 */
    uint32_t pwm_channel;         /**< 对应的 TIM_CHANNEL_x。 */
    GPIO_TypeDef *in1_port;       /**< TB6612 IN1 所在 GPIO 端口。 */
    uint16_t in1_pin;             /**< TB6612 IN1 引脚掩码。 */
    GPIO_TypeDef *in2_port;       /**< TB6612 IN2 所在 GPIO 端口。 */
    uint16_t in2_pin;             /**< TB6612 IN2 引脚掩码。 */
    bool inverted;                /**< 是否反转逻辑命令的方向。 */
    bool initialized;             /**< PWM 通道是否已成功启动。 */
    int16_t command;              /**< 最近一次限幅后的逻辑命令。 */
} BSP_Motor;

/**
 * @brief 绑定一路电机资源并启动 PWM 通道。
 * @param motor 由调用方分配的电机对象。
 * @param pwm_timer 已由 CubeMX 初始化的 PWM 定时器句柄。
 * @param pwm_channel 该电机使用的 TIM_CHANNEL_x。
 * @param in1_port TB6612 IN1 的 GPIO 端口。
 * @param in1_pin TB6612 IN1 的 GPIO 引脚。
 * @param in2_port TB6612 IN2 的 GPIO 端口。
 * @param in2_pin TB6612 IN2 的 GPIO 引脚。
 * @param inverted true 时反转正负命令对应的物理方向。
 * @return HAL_OK 表示 PWM 通道启动成功；参数错误或启动失败时返回错误。
 *
 * 初始化会先清零 PWM 并拉低两个方向引脚，避免上电时电机误动作。
 */
HAL_StatusTypeDef BSP_Motor_Init(
    BSP_Motor *motor,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    GPIO_TypeDef *in1_port,
    uint16_t in1_pin,
    GPIO_TypeDef *in2_port,
    uint16_t in2_pin,
    bool inverted
);

/**
 * @brief 设置一路电机的逻辑方向和输出强度。
 * @param motor 已成功初始化的电机对象。
 * @param command -1000 到 +1000；超出范围会自动限幅。
 * @return HAL_OK 表示命令已写入；对象无效或未初始化时返回 HAL_ERROR。
 *
 * 改变方向时会先把 PWM 清零，再切换方向 GPIO，以减少反向瞬间的冲击。
 */
HAL_StatusTypeDef BSP_Motor_SetCommand(BSP_Motor *motor, int16_t command);

/**
 * @brief 将电机命令安全清零。
 * @return 与 BSP_Motor_SetCommand(motor, 0) 相同。
 */
HAL_StatusTypeDef BSP_Motor_Stop(BSP_Motor *motor);

/**
 * @brief 取得最近一次限幅后的逻辑命令。
 * @return 有效对象返回 -1000 到 +1000；无效对象返回 0。
 */
int16_t BSP_Motor_GetCommand(const BSP_Motor *motor);

#endif /* BALANCECAR_MOTOR_H */

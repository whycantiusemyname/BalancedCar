//
// Created by Nana Daiba on 2026/7/21.
//

#ifndef BALANCECAR_ENCODER_H
#define BALANCECAR_ENCODER_H

/**
 * @file Encoder.h
 * @brief STM32 定时器正交编码器的增量读取接口。
 *
 * CubeMX 负责把 TIM3/TIM4 配置成 Encoder Mode TI12。驱动启动两个
 * 输入通道，并记录相邻两次读取之间的计数差。速度换算、滤波和每圈
 * 计数标定属于控制层，不在 BSP 中完成。
 */

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief 一路正交编码器及其增量采样状态。
 */
typedef struct
{
    TIM_HandleTypeDef *timer; /**< CubeMX 创建的编码器定时器句柄。 */
    uint16_t last_count;      /**< 上一次采样时的 16 位 CNT。 */
    bool inverted;            /**< 是否反转上层看到的计数方向。 */
    bool initialized;         /**< 两个编码器通道是否已成功启动。 */
} BSP_Encoder;

/**
 * @brief 绑定定时器并启动两个正交编码器输入通道。
 * @param encoder 由调用方分配的编码器对象。
 * @param timer 已配置为 Encoder Mode TI12 的 HAL 定时器句柄。
 * @param inverted true 时反转读取到的增量，用于统一左右轮正方向。
 * @return HAL_OK 表示启动成功；参数无效或 HAL 启动失败时返回错误。
 *
 * 初始化不会清零硬件 CNT，而是把当前计数保存为第一次采样的基准。
 */
HAL_StatusTypeDef BSP_Encoder_Init(
    BSP_Encoder *encoder,
    TIM_HandleTypeDef *timer,
    bool inverted
);

/**
 * @brief 读取自上次调用以来的有符号编码器增量。
 * @param encoder 已成功初始化的编码器对象。
 * @return 本采样区间的计数变化；对象无效时返回 0。
 *
 * 函数会处理 16 位 CNT 在 65535 和 0 之间的回绕。调用间隔内的真实
 * 计数变化必须小于 32768，否则无法从环形计数值唯一判断移动方向。
 * 每次读取都会更新内部基准，所以同一段增量只会返回一次。
 */
int16_t BSP_Encoder_ReadDelta(BSP_Encoder *encoder);

#endif /* BALANCECAR_ENCODER_H */

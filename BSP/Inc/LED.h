//
// Created by Nana Daiba on 2026/7/20.
//

#ifndef BALANCECAR_LED_H
#define BALANCECAR_LED_H

/**
 * @brief 点亮板载 LED。
 *
 * 当前硬件上的 LED 为低电平有效。
 */
void BSP_LED_ON(void);

/**
 * @brief 熄灭板载 LED。
 */
void BSP_LED_OFF(void);

/**
 * @brief 翻转板载 LED 的当前状态。
 */
void BSP_LED_Toggle(void);

#endif //BALANCECAR_LED_H

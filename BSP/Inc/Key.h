//
// Created by Nana Daiba on 2026/7/20.
//

#ifndef BALANCECAR_KEY_H
#define BALANCECAR_KEY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief 按键边沿事件。
 *
 * 事件由 BSP_Key_Update() 产生，并通过 BSP_Key_TakeEvent() 一次性取走。
 */
typedef enum {
    BSP_KEY_EVENT_NONE,       /**< 当前没有待处理事件。 */
    BSP_KEY_EVENT_PRESSED,    /**< 按键经过消抖后确认按下。 */
    BSP_KEY_EVENT_RELEASED    /**< 按键经过消抖后确认松开。 */
} BSP_KeyEvent;

/**
 * @brief 单个按键的运行状态。
 *
 * 每个实体按键对应一个 BSP_Key 对象，由初始化函数绑定 GPIO。
 */
typedef struct {
    GPIO_TypeDef *gpio_port;          /**< 按键所在的 GPIO 端口。 */
    uint16_t gpio_pin;                /**< 按键对应的 GPIO 引脚掩码。 */
    GPIO_PinState pressed_level;      /**< 按下时的有效电平。 */

    bool raw_pressed;                 /**< 最近一次采样得到的原始按下状态。 */
    bool stable_pressed;              /**< 经过消抖确认的稳定按下状态。 */
    uint32_t raw_changed_at;          /**< 原始状态最近变化的时刻，单位 ms。 */

    BSP_KeyEvent pending_event;       /**< 等待应用层取走的单次事件。 */
    uint16_t debounce_ms;             /**< 消抖确认时间，单位 ms。 */
} BSP_Key;

/**
 * @brief 初始化一个按键对象并绑定其 GPIO。
 * @param key             待初始化的按键对象。
 * @param gpio_port       GPIO 端口，例如 KEY_1_GPIO_Port。
 * @param gpio_pin        GPIO 引脚，例如 KEY_1_Pin。
 * @param pressed_level   按下时的电平；上拉按键通常为 GPIO_PIN_RESET。
 * @param debounce_ms     消抖时间，通常可从 20 ms 开始。
 * @param now_ms          当前系统时间，通常传入 HAL_GetTick()。
 */
void BSP_Key_Init(
    BSP_Key *key,
    GPIO_TypeDef *gpio_port,
    uint16_t gpio_pin,
    GPIO_PinState pressed_level,
    uint16_t debounce_ms,
    uint32_t now_ms
);

/**
 * @brief 更新一次按键采样和消抖状态。
 * @param key     要更新的按键对象。
 * @param now_ms  当前系统时间，通常传入 HAL_GetTick()。
 *
 * 应在主循环或周期任务中持续调用；函数本身不阻塞。
 */
void BSP_Key_Update(BSP_Key *key, uint32_t now_ms);

/**
 * @brief 查询按键经过消抖后的持续状态。
 * @return 当前稳定按下时返回 true，否则返回 false。
 */
bool BSP_Key_IsPressed(const BSP_Key *key);

/**
 * @brief 取出一个待处理的按键事件。
 * @return 当前事件；读取后内部事件自动清空为 BSP_KEY_EVENT_NONE。
 */
BSP_KeyEvent BSP_Key_TakeEvent(BSP_Key *key);

#endif //BALANCECAR_KEY_H

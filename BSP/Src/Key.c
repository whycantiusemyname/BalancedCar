//
// Created by Nana Daiba on 2026/7/20.
//

#include "Key.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 绑定按键 GPIO，并用当前电平建立初始消抖状态。
 */
void BSP_Key_Init(
    BSP_Key *key,
    GPIO_TypeDef *gpio_port,
    uint16_t gpio_pin,
    GPIO_PinState pressed_level,
    uint16_t debounce_ms,
    uint32_t now_ms
) {
    /* 先绑定硬件信息，随后才能通过按键对象读取实体引脚。 */
    key->gpio_port = gpio_port;
    key->gpio_pin = gpio_pin;
    key->debounce_ms = debounce_ms;
    key->pressed_level = pressed_level;

    bool current_pressed =
        HAL_GPIO_ReadPin(key->gpio_port, key->gpio_pin) == key->pressed_level;
    key->raw_pressed = current_pressed;
    key->stable_pressed = current_pressed;
    key->raw_changed_at = now_ms;
    key->pending_event = BSP_KEY_EVENT_NONE;
}

/**
 * @brief 采样按键并执行非阻塞消抖。
 */
void BSP_Key_Update(BSP_Key *key, uint32_t now_ms) {
    /* 将物理电平转换成与高低有效方式无关的逻辑“是否按下”。 */
    bool current_pressed =
        HAL_GPIO_ReadPin(key->gpio_port, key->gpio_pin) == key->pressed_level;
    if (current_pressed != key->raw_pressed) {
        key->raw_pressed = current_pressed;
        key->raw_changed_at = now_ms;
    }

    if (key->raw_pressed != key->stable_pressed &&
        now_ms - key->raw_changed_at >= key->debounce_ms) {

        key->stable_pressed = key->raw_pressed;

        if (key->stable_pressed) {
            key->pending_event = BSP_KEY_EVENT_PRESSED;
        } else {
            key->pending_event = BSP_KEY_EVENT_RELEASED;
        }
    }

}

/**
 * @brief 返回按键经过消抖后的持续状态，不会消费事件。
 */
bool BSP_Key_IsPressed(const BSP_Key *key) {
    return key->stable_pressed;
}

/**
 * @brief 返回待处理事件，并将其清空，确保每个事件只被消费一次。
 */
BSP_KeyEvent BSP_Key_TakeEvent(BSP_Key *key) {
    BSP_KeyEvent event = key->pending_event;
    key->pending_event = BSP_KEY_EVENT_NONE;
    return event;
}

//
// Created by Nana Daiba on 2026/7/20.
//
#include "LED.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"

/**
 * @brief 点亮板载 LED。
 *
 * LED 采用低电平有效连接，因此输出 RESET 时点亮。
 */
void BSP_LED_ON(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);	// 输出低电平，LED 点亮
}

/**
 * @brief 熄灭板载 LED。
 *
 * LED 采用低电平有效连接，因此输出 SET 时熄灭。
 */
void BSP_LED_OFF(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);			// 输出高电平，LED 熄灭
}

/**
 * @brief 翻转板载 LED 的当前输出电平。
 */
void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

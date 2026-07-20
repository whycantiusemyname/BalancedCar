//
// Created by Nana Daiba on 2026/7/20.
//

/**
 * @file Motor.c
 * @brief TB6612FNG 单路直流电机控制实现。
 */

#include "Motor.h"

/**
 * @brief 判断 HAL PWM 通道参数是否属于标准的四个通道之一。
 */
static bool BSP_Motor_IsValidChannel(uint32_t channel)
{
    return channel == TIM_CHANNEL_1 ||
           channel == TIM_CHANNEL_2 ||
           channel == TIM_CHANNEL_3 ||
           channel == TIM_CHANNEL_4;
}

/**
 * @brief 把归一化命令幅值换算为当前定时器的 CCR 值。
 */
static uint32_t BSP_Motor_CommandToPulse(const BSP_Motor *motor,
                                         uint16_t magnitude)
{
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);

    return ((uint32_t)magnitude * period) / BSP_MOTOR_COMMAND_MAX;
}

HAL_StatusTypeDef BSP_Motor_Init(
    BSP_Motor *motor,
    TIM_HandleTypeDef *pwm_timer,
    uint32_t pwm_channel,
    GPIO_TypeDef *in1_port,
    uint16_t in1_pin,
    GPIO_TypeDef *in2_port,
    uint16_t in2_pin,
    bool inverted
)
{
    if (motor == NULL ||
        pwm_timer == NULL ||
        in1_port == NULL ||
        in2_port == NULL ||
        !BSP_Motor_IsValidChannel(pwm_channel))
    {
        return HAL_ERROR;
    }

    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->in1_port = in1_port;
    motor->in1_pin = in1_pin;
    motor->in2_port = in2_port;
    motor->in2_pin = in2_pin;
    motor->inverted = inverted;
    motor->initialized = false;
    motor->command = 0;

    /* PWM 启动前先建立无驱动输出，防止初始化期间电机突然转动。 */
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0U);
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(motor->pwm_timer,
                                                  motor->pwm_channel);
    if (status != HAL_OK)
    {
        return status;
    }

    motor->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_Motor_SetCommand(BSP_Motor *motor, int16_t command)
{
    if (motor == NULL || !motor->initialized || motor->pwm_timer == NULL)
    {
        return HAL_ERROR;
    }

    if (command > BSP_MOTOR_COMMAND_MAX)
    {
        command = BSP_MOTOR_COMMAND_MAX;
    }
    else if (command < -BSP_MOTOR_COMMAND_MAX)
    {
        command = -BSP_MOTOR_COMMAND_MAX;
    }

    motor->command = command;

    /* 先撤掉 PWM，再改变 H 桥方向，避免带占空比直接反向。 */
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0U);

    int16_t physical_command = motor->inverted ? (int16_t)-command : command;

    if (physical_command > 0)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    }
    else if (physical_command < 0)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
        return HAL_OK;
    }

    const uint16_t magnitude = physical_command > 0
        ? (uint16_t)physical_command
        : (uint16_t)-physical_command;
    const uint32_t pulse = BSP_Motor_CommandToPulse(motor, magnitude);

    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pulse);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_Motor_Stop(BSP_Motor *motor)
{
    return BSP_Motor_SetCommand(motor, 0);
}

int16_t BSP_Motor_GetCommand(const BSP_Motor *motor)
{
    if (motor == NULL || !motor->initialized)
    {
        return 0;
    }

    return motor->command;
}

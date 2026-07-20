/**
 * @file Encoder.c
 * @brief STM32 定时器正交编码器增量读取实现。
 */

#include "Encoder.h"

HAL_StatusTypeDef BSP_Encoder_Init(
    BSP_Encoder *encoder,
    TIM_HandleTypeDef *timer,
    bool inverted
) {
    if (encoder == NULL) { return HAL_ERROR; }
    if (timer == NULL) { return HAL_ERROR; }
    encoder->timer = timer;
    encoder->inverted = inverted;
    encoder->initialized = false;
    HAL_StatusTypeDef Status = HAL_TIM_Encoder_Start(timer,TIM_CHANNEL_ALL);

    if (Status != HAL_OK) { return Status; }

    encoder->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(timer);
    encoder->initialized = true;

    return HAL_OK;
}
int16_t BSP_Encoder_ReadDelta(BSP_Encoder *encoder) {
    if (encoder == NULL) { return 0; }
    if (encoder->initialized == false) { return 0; }
    if (encoder->timer == NULL) { return 0; }
    uint16_t current_count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder->timer);
    int32_t difference = (int32_t)current_count - (int32_t)encoder->last_count;
    encoder->last_count = current_count;
    /* 选择 16 位环形计数空间中的最短有符号距离。 */
    if (difference > 32767) {
        difference -= 65536;
    } else if (difference < -32768) {
        difference += 65536;
    }
    int16_t delta = (int16_t)difference;

    if (encoder->inverted) {
        delta = (int16_t)-delta;
    }

    return delta;

}

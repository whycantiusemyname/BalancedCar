/** @file PID.c @brief 带显式采样周期、积分限幅和输出限幅的位置式PID。 */

#include "PID.h"

#include <stddef.h>

void PID_Reset(PID *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral_output = 0.0F;
    pid->previous_error = 0.0F;
    pid->has_previous_error = false;
}

float PID_Update(PID *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds)
{
    if (pid == NULL || dt_seconds <= 0.0F)
    {
        return 0.0F;
    }

    const float error = setpoint - measurement;

    /* 第一次更新没有上一帧，令微分项为0，避免启动瞬间产生微分冲击。 */
    float derivative = 0.0F;
    if (pid->has_previous_error)
    {
        derivative = (error - pid->previous_error) / dt_seconds;
    }

    /*
     * 先计算候选I项，再根据最终输出限幅决定是否接受。若控制量已经饱和，
     * 只禁止继续把输出推向饱和方向；反向误差仍可立即释放已有积分。
     * integral_min/max继续负责限制I项自身，单位始终与控制器输出一致。
     */
    const float integral_delta = pid->ki * error * dt_seconds;
    float candidate_integral = pid->integral_output + integral_delta;
    if (candidate_integral < pid->integral_min)
    {
        candidate_integral = pid->integral_min;
    }
    else if (candidate_integral > pid->integral_max)
    {
        candidate_integral = pid->integral_max;
    }

    const float proportional_and_derivative =
        pid->kp * error + pid->kd * derivative;
    const float candidate_output =
        proportional_and_derivative + candidate_integral;
    const bool pushes_upper_saturation =
        candidate_output > pid->output_max && integral_delta > 0.0F;
    const bool pushes_lower_saturation =
        candidate_output < pid->output_min && integral_delta < 0.0F;
    if (!pushes_upper_saturation && !pushes_lower_saturation)
    {
        pid->integral_output = candidate_integral;
    }

    float output = proportional_and_derivative + pid->integral_output;

    pid->previous_error = error;
    pid->has_previous_error = true;

    if (output < pid->output_min)
    {
        output = pid->output_min;
    }
    else if (output > pid->output_max)
    {
        output = pid->output_max;
    }

    return output;
}

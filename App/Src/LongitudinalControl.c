/** @file LongitudinalControl.c @brief 平衡车速度环与角度环的串级控制。 */

#include "LongitudinalControl.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#include "BalanceCarConfig.h"

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void LongitudinalControl_Init(LongitudinalControl *control)
{
    if (control == NULL)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    /*
     * 速度环只生成相对平衡零点的倾角修正，不直接输出PWM。这里不重复
     * 限制相对修正量，最终目标倾角统一在UpdateSpeed中进行一次绝对限幅。
     */
    control->speed_pid = (PID){
        .kp = BALANCE_CAR_DEFAULT_SPEED_KP,
        .ki = BALANCE_CAR_DEFAULT_SPEED_KI,
        .kd = BALANCE_CAR_DEFAULT_SPEED_KD,
        .output_min = -FLT_MAX,
        .output_max = FLT_MAX,
        .integral_min = -BALANCE_CAR_DEFAULT_MAX_SPEED_INTEGRAL_PITCH_DEG,
        .integral_max = BALANCE_CAR_DEFAULT_MAX_SPEED_INTEGRAL_PITCH_DEG,
    };

    /* 角度环的输出才是左右轮共同使用的电机控制量。 */
    control->angle.pid = (PID){
        .kp = BALANCE_CAR_DEFAULT_ANGLE_KP,
        .ki = BALANCE_CAR_DEFAULT_ANGLE_KI,
        .kd = BALANCE_CAR_DEFAULT_ANGLE_KD,
        .output_min = -(float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT,
        .output_max = (float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT,
        .integral_min = -(float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT,
        .integral_max = (float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT,
    };
}

void LongitudinalControl_Reset(LongitudinalControl *control)
{
    if (control == NULL)
    {
        return;
    }

    PID_Reset(&control->speed_pid);
    PID_Reset(&control->angle.pid);
    control->target_pitch_deg = 0.0F;
}

float LongitudinalControl_UpdateSpeed(
    LongitudinalControl *control,
    float target_speed_counts_per_s,
    float measured_speed_counts_per_s,
    float balance_trim_deg,
    float dt_seconds)
{
    if (control == NULL || dt_seconds <= 0.0F)
    {
        return 0.0F;
    }

    PID *speed_pid = &control->speed_pid;
    /*
     * 速度环的实际输出边界取决于当前平衡点。把相对倾角边界直接交给
     * PID，令其在最终目标倾角饱和时停止同方向积分，而不是先积累后再
     * 被下方的整车安全边界静默裁掉。
     */
    speed_pid->output_min =
        -BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG - balance_trim_deg;
    speed_pid->output_max =
        BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG - balance_trim_deg;

    const float speed_pitch_delta_deg = PID_Update(
        speed_pid,
        target_speed_counts_per_s,
        measured_speed_counts_per_s,
        dt_seconds);

    /*
     * balance_trim_deg 是实车静态工作点；速度环只能在其附近前后倾斜。
     * 最后的绝对限幅是第二道安全边界，防止错误调参产生危险目标角。
     */
    control->target_pitch_deg = Clamp(
        balance_trim_deg + speed_pitch_delta_deg,
        -BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG,
        BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG);

    return control->target_pitch_deg;
}

float LongitudinalControl_UpdateAngle(
    LongitudinalControl *control,
    float measured_pitch_deg,
    float measured_pitch_rate_dps,
    float dt_seconds)
{
    if (control == NULL || dt_seconds <= 0.0F)
    {
        return 0.0F;
    }

    /*
     * 角度环直接使用陀螺仪角速度作为D项。目标倾角变化时不会产生
     * 微分冲击，而且这层代码表达的是实际物理关系，不再为复用几行
     * PID计算额外建立 PID_Apply/PID_UpdateWithMeasurementRate 两层函数。
     */
    PID *pid = &control->angle.pid;
    /*
     * 整车统一约定：前倾角、前倾角速度和向前电机命令都为正。
     * 因而车体比目标更前倾时必须给正输出，让轮子向前追赶倾倒方向。
     */
    const float error = measured_pitch_deg - control->target_pitch_deg;
    pid->integral_output = Clamp(
        pid->integral_output + pid->ki * error * dt_seconds,
        pid->integral_min,
        pid->integral_max);

    /* 电机死区补偿不在这里做：它属于单个轮子，由App在差分混合后叠加。 */
    const float output = pid->kp * error +
                         pid->integral_output +
                         pid->kd * measured_pitch_rate_dps;

    return Clamp(output, pid->output_min, pid->output_max);
}

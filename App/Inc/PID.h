#ifndef BALANCE_CAR_PID_H
#define BALANCE_CAR_PID_H

/** @file PID.h @brief 与HAL和具体被控对象无关的单路PID。 */

#include <stdbool.h>

/**
 * 一个PID实例同时保存参数和跨周期历史。项目只有固定三个控制环，不再把
 * Config和Controller拆成两层；蓝牙调试时可直接修改kp/ki/kd。
 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;
    /** 对I项最终输出的限制，单位与控制器输出相同。 */
    float integral_min;
    float integral_max;

    /** 已经乘过Ki的I项输出，而不是原始误差积分。 */
    float integral_output;
    float previous_error;
    bool has_previous_error;
} PID;

/** 只清除积分和微分历史，保留当前参数与限幅。 */
void PID_Reset(PID *pid);

/** 调用者显式传入dt，PID内部不依赖HAL_GetTick()。 */
float PID_Update(PID *pid,
                 float setpoint,
                 float measurement,
                 float dt_seconds);

#endif /* BALANCE_CAR_PID_H */

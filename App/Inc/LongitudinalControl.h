#ifndef LONGITUDINAL_CONTROL_H
#define LONGITUDINAL_CONTROL_H

/**
 * @file LongitudinalControl.h
 * @brief 把速度外环和角度内环封装成整车纵向控制能力。
 *
 * 速度环不直接驱动电机，而是给角度环生成目标倾角；角度环再根据车体
 * 姿态生成左右轮共同使用的电机控制量。因此这里封装的是整辆平衡车的
 * 纵向运动，不是某一个“可控速度电机”。
 */

#include "PID.h"

/** 角度内环：目标倾角 -> 公共电机控制量。 */
typedef struct
{
    PID pid;
} AngleControl;

/** 速度外环包住角度内环，组成串级纵向控制器。 */
typedef struct
{
    PID speed_pid;
    AngleControl angle;

    /** 平衡零点加速度环相对修正后的目标倾角，在两次外环更新之间保持。 */
    float target_pitch_deg;
} LongitudinalControl;

void LongitudinalControl_Init(LongitudinalControl *control);
void LongitudinalControl_Reset(LongitudinalControl *control);

/**
 * 低频速度环：根据整车速度误差生成相对平衡零点的小倾角修正，再与
 * balance_trim_deg 相加得到绝对目标倾角。
 * @return 包含人工平衡零点修正后的绝对目标倾角，单位为度。
 */
float LongitudinalControl_UpdateSpeed(
    LongitudinalControl *control,
    float target_speed_counts_per_s,
    float measured_speed_counts_per_s,
    float balance_trim_deg,
    float dt_seconds);

/**
 * 高频角度环：追踪速度环给出的目标倾角。
 * @return 两个轮子共同使用的控制量，尚未叠加转向差分量。
 */
float LongitudinalControl_UpdateAngle(
    LongitudinalControl *control,
    float measured_pitch_deg,
    float measured_pitch_rate_dps,
    float dt_seconds);

#endif /* LONGITUDINAL_CONTROL_H */

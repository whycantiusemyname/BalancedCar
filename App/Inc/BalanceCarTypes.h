#ifndef BALANCE_CAR_TYPES_H
#define BALANCE_CAR_TYPES_H

#include <stdint.h>

/**
 * @file BalanceCarTypes.h
 * @brief 应用模块之间交换的数据，不依赖 HAL 或具体硬件。
 */

/** 九个可在线调节的PID参数直接使用一个索引，不再拆成“环+增益”两级。 */
typedef enum
{
    BALANCE_CAR_PID_ANGLE_KP = 0,
    BALANCE_CAR_PID_ANGLE_KI,
    BALANCE_CAR_PID_ANGLE_KD,
    BALANCE_CAR_PID_SPEED_KP,
    BALANCE_CAR_PID_SPEED_KI,
    BALANCE_CAR_PID_SPEED_KD,
    BALANCE_CAR_PID_TURN_KP,
    BALANCE_CAR_PID_TURN_KI,
    BALANCE_CAR_PID_TURN_KD,
    BALANCE_CAR_PID_PARAMETER_COUNT
} BalanceCar_PIDParameter;

/** 蓝牙点动测试选择的电机；BOTH用于检查整车前后方向。 */
typedef enum
{
    BALANCE_CAR_MOTOR_LEFT = 0,
    BALANCE_CAR_MOTOR_RIGHT,
    BALANCE_CAR_MOTOR_BOTH
} BalanceCar_MotorSelection;

/** 来自按键、蓝牙或其他输入源的统一整车命令。 */
typedef enum
{
    BALANCE_CAR_COMMAND_NONE = 0,
    BALANCE_CAR_COMMAND_SET_FORWARD_SPEED,
    BALANCE_CAR_COMMAND_SET_TURN_SPEED,
    BALANCE_CAR_COMMAND_SET_MOTION_TARGET,
    BALANCE_CAR_COMMAND_SET_BALANCE_TRIM,
    BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_OFFSET,
    BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_BAND,
    BALANCE_CAR_COMMAND_SET_POSITION_HOLD_KP,
    BALANCE_CAR_COMMAND_START,
    BALANCE_CAR_COMMAND_STOP,
    BALANCE_CAR_COMMAND_SET_PID_GAIN,
    BALANCE_CAR_COMMAND_TEST_MOTOR
} BalanceCar_CommandType;

typedef struct
{
    BalanceCar_CommandType type;
    union
    {
        float scalar;
        struct
        {
            BalanceCar_PIDParameter parameter;
            float value;
        } pid;
        struct
        {
            BalanceCar_MotorSelection selection;
            int16_t command;
        } motor_test;
        struct
        {
            float forward_speed_counts_per_s;
            float yaw_rate_dps;
        } motion;
    } data;
} BalanceCar_Command;

/**
 * 控制器和遥测共用的一份整车状态估计。
 * MPU原始值、编码器增量和dt只在App单次采样中临时存在，不再复制一层
 * SensorFrame。编码器速度暂以counts/s表示，便于先调通。
 */
typedef struct
{
    float pitch_deg;
    float pitch_rate_dps;
    /** 绕车体竖直轴的角速度；已减去本次上电静止校准得到的零偏。 */
    float yaw_rate_dps;
    float forward_speed_counts_per_s;
    float turn_speed_counts_per_s;
} BalanceCar_StateEstimate;

/** 来自按键或遥控器的整车目标，以及可在线调节的整车级补偿参数。 */
typedef struct
{
    float forward_speed_counts_per_s;
    /** 目标偏航角速度；左转为正、右转为负，单位为度每秒。 */
    float yaw_rate_dps;
    float balance_trim_deg;
    /** 电机死区补偿幅值，混合出左右轮命令后按每轮符号叠加。 */
    float motor_deadzone_offset;
    /** 死区补偿线性过渡带宽；0表示过零直接±offset硬跳变。 */
    float motor_deadzone_band;
    /** 驻车位置环增益，(counts/s)/count；0表示关闭位置保持。 */
    float position_hold_kp;
} BalanceCar_ControlTarget;

#endif /* BALANCE_CAR_TYPES_H */

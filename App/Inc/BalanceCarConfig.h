#ifndef BALANCE_CAR_CONFIG_H
#define BALANCE_CAR_CONFIG_H

/**
 * @file BalanceCarConfig.h
 * @brief 整车级、可集中修改的默认配置。
 *
 * 这里只放“换一辆车可能需要修改”的数值。具体算法的运行状态不应
 * 以全局变量形式放在这里。
 */

#include <stdbool.h>

/* 主循环中的固定周期任务；控制周期改变时要同步复核PID的dt。 */
#define BALANCE_CAR_MPU_ADDRESS                 0x68U
#define BALANCE_CAR_KEY_DEBOUNCE_MS             20U

#define BALANCE_CAR_IMU_SAMPLE_PERIOD_MS             5U
#define BALANCE_CAR_OUTER_LOOP_PERIOD_MS         20U
#define BALANCE_CAR_KEY_TASK_PERIOD_MS           10U
#define BALANCE_CAR_BLUETOOTH_TASK_PERIOD_MS      5U

/*
 * 非0时启动阶段尝试用AT命令把HC-04切到该波特率并重配USART2;CubeMX
 * 侧保持9600作为初次协商语言。模块已在目标波特率时协商是幂等空操作。
 * 改回0即完全恢复9600行为(遥测周期同步放宽,9600带宽只够60ms)。
 */
#define BALANCE_CAR_BLUETOOTH_UPGRADE_BAUD      115200U
#if BALANCE_CAR_BLUETOOTH_UPGRADE_BAUD != 0U
#define BALANCE_CAR_TELEMETRY_PERIOD_MS          40U
#else
#define BALANCE_CAR_TELEMETRY_PERIOD_MS          60U
#endif
/* 每20ms发送OLED的一页，完整8页约160ms刷新一次。 */
#define BALANCE_CAR_DISPLAY_PAGE_PERIOD_MS       20U

/* 整车控制与安全边界，固定车型直接使用集中宏，不再复制到Config对象。 */
#define BALANCE_CAR_COMMAND_TIMEOUT_MS           500U
/* 教程摇杆输出为-100..100；本车按控制器实际单位映射速度和偏航角速度。 */
#define BALANCE_CAR_JOYSTICK_DEADBAND               5.0F
#define BALANCE_CAR_JOYSTICK_MAX_FORWARD_SPEED   2500.0F
#define BALANCE_CAR_JOYSTICK_MAX_YAW_RATE_DPS      180.0F
#define BALANCE_CAR_DEFAULT_MAX_TILT_DEG          35.0F
#define BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG  10.0F
/*
 * 7-22日志显示静态Trim误差就要吃掉约0.3~0.9度积分，1.5度上限在长时间
 * 前进或爬坡时会顶死，放宽到2.5度；绝对目标倾角仍受上面10度限制。
 */
#define BALANCE_CAR_DEFAULT_MAX_SPEED_INTEGRAL_PITCH_DEG 2.5F
#define BALANCE_CAR_DEFAULT_MOTOR_LIMIT          1000
/* 转向只占用部分电机余量，优先保证公共平衡量不被左右轮独立饱和破坏。 */
#define BALANCE_CAR_DEFAULT_MAX_TURN_OUTPUT       300.0F
#define BALANCE_CAR_DEFAULT_MAX_TURN_INTEGRAL_OUTPUT 50.0F
/*
 * 驻车位置环：纵向目标为0且车速降入门限后锁存编码器位置，位置误差乘
 * PosKp生成小速度目标。7-26实测无位置环时单次全杆原地转会带出±5~10cm
 * 位移，斜坡上也会持续溜车。PosKp为0等于关闭该功能。
 */
/* 7-26实测2.0~3.0均无位置来回蹭;3.0旋转期间圈得更住,取3.0。 */
#define BALANCE_CAR_DEFAULT_POSITION_HOLD_KP         3.0F
#define BALANCE_CAR_POSITION_HOLD_MAX_SPEED_COUNTS_PER_S 400.0F
#define BALANCE_CAR_POSITION_HOLD_ENGAGE_SPEED_COUNTS_PER_S 400.0F

/* 目标转速为0时忽略这一范围内的残余GZ零偏，并清除转向PID历史。 */
#define BALANCE_CAR_YAW_RATE_DEADBAND_DPS           1.0F
/* 只有纵向目标为0且实测速度也进入静止带，才清除直行时学到的转向积分。 */
#define BALANCE_CAR_TURN_INTEGRAL_RESET_SPEED_COUNTS_PER_S 400.0F

/* 当前实车已经验证可自主站立的一组起始参数，仍可通过蓝牙继续细调。 */
/* 7-22三段日志静止实测平衡角0.7~1.3度，取中值；旧值1.6度靠速度积分代偿。 */
#define BALANCE_CAR_DEFAULT_BALANCE_TRIM_DEG         1.2F
#define BALANCE_CAR_DEFAULT_ANGLE_KP                65.0F
#define BALANCE_CAR_DEFAULT_ANGLE_KI                 0.0F
/* 7-26扫参:死区补偿平滑后最优D上移,2.2~2.4为平台区,取中点2.3。 */
#define BALANCE_CAR_DEFAULT_ANGLE_KD                 2.3F
/*
 * 电机死区补偿在左右轮混合之后按每个轮子自己的符号叠加。补偿在
 * ±BAND内随命令线性过渡到0，而不是过零时±OFFSET硬跳变：7-22日志中
 * 该跳变让静止电机命令以约5Hz在±48间翻转（相邻60ms采样68%变号）。
 * BAND=0 恢复旧的硬阶跃行为，便于实车A/B对比；两者都可蓝牙在线调。
 */
/*
 * 实测起动死区约48，但运行中动摩擦低于静摩擦：7-26静置A/B显示补偿42
 * 比48电机跳变-30%、转向噪声-33%且机械稳定不变，48属于过补偿。
 */
#define BALANCE_CAR_DEFAULT_MOTOR_DEADZONE_OFFSET   42.0F
/*
 * 7-26实车扫参(20/30/45/60,各45s静置):30~45为甜区,相对硬阶跃
 * 电机命令平均跳变-43%、变号率59%->31%、pitch_rate sd -25%;急停回摆
 * 无差异。60时低增益区过宽,俯仰漂移反而变大;20已接近硬阶跃。
 */
#define BALANCE_CAR_DEFAULT_MOTOR_DEADZONE_BAND     40.0F
/* 7-26手柄三点对比:0.0034高速跟踪0.91~0.95且无慢摆;0.0042已明显不稳。 */
#define BALANCE_CAR_DEFAULT_SPEED_KP                 0.0034F
/* 7-26:0.00017静置最干净且压阶跃超调;0.00019电气活动开始回升。 */
#define BALANCE_CAR_DEFAULT_SPEED_KI                 0.00017F
#define BALANCE_CAR_DEFAULT_SPEED_KD                 0.0F
/* 7-26:2.6实驾验证;3.5会因GZ噪声×增益产生剧烈差分抖动,严禁大步。 */
#define BALANCE_CAR_DEFAULT_TURN_KP                  2.6F
/*
 * 目标偏航角速度前馈(输出/dps)。持续旋转实测跟踪比卡0.72且积分闭不
 * 掉,纯反馈提增益又会放大GZ噪声;前馈直接按目标给差分量,无稳定性
 * 代价。7-26在线扫参0/0.4/0.8/1.0/1.2线性响应,1.0时稳态比0.95、
 * 瞬态峰1.07,取1.0。
 */
#define BALANCE_CAR_DEFAULT_TURN_FEEDFORWARD         1.0F
#define BALANCE_CAR_DEFAULT_TURN_KI                  0.30F
#define BALANCE_CAR_DEFAULT_TURN_KD                  0.0F

/* 蓝牙点动只用于架空轮子的方向诊断，限制为15%输出并自动停止。 */
#define BALANCE_CAR_MOTOR_TEST_MAX_COMMAND        150
#define BALANCE_CAR_MOTOR_TEST_DURATION_MS        300U

/* MPU6050换算、静止校准与互补滤波的首轮调试参数。 */
#define BALANCE_CAR_MPU_ACCEL_LSB_PER_G        16384.0F
#define BALANCE_CAR_MPU_GYRO_LSB_PER_DPS         131.0F
/*
 * 加速度计相对车体的固定安装角修正，只应在机械结构变化后重新标定。
 * 先保持0度，通过多次重复测量确定本车数值后再写死；不要在每次启动时
 * 用手扶姿态重定义零点。
 */
#define BALANCE_CAR_ACCEL_PITCH_MOUNT_OFFSET_DEG   1.5F
/*
 * 7-26实测急加减速时加速度角偏差中位数3.7度、p90达12度;把时间常数
 * 从5s提到8s进一步压低机动期间对姿态估计的污染,代价是零偏收敛更慢
 * (上电校准已消除主要陀螺零偏,可接受)。
 */
#define BALANCE_CAR_COMPLEMENTARY_TIME_CONSTANT_S    8.0F
#define BALANCE_CAR_CALIBRATION_SAMPLE_COUNT      500U

/* 根据已完成的电机/编码器联动测试建立逻辑方向，实车前进方向仍需复核。 */
#define BALANCE_CAR_LEFT_MOTOR_INVERTED          false
#define BALANCE_CAR_RIGHT_MOTOR_INVERTED         true
#define BALANCE_CAR_LEFT_ENCODER_INVERTED         true
#define BALANCE_CAR_RIGHT_ENCODER_INVERTED       false

#endif /* BALANCE_CAR_CONFIG_H */

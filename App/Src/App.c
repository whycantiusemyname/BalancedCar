/**
 * @file App.c
 * @brief 单例整车状态机及其非阻塞任务调度。
 *
 * 这里是唯一知道整车全部模块的文件。main.c 只负责 CubeMX 初始化并持续
 * 调用 App_Update()；各 BSP 和算法模块不直接依赖整车状态。
 */

#include "App.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "BalanceCarConfig.h"
#include "BalanceCarTypes.h"
#include "BluetoothProtocol.h"
#include "BlueToothSerial.h"
#include "Command.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "LongitudinalControl.h"
#include "MPU6050.h"
#include "Motor.h"
#include "oled.h"
#include "PID.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

/** 一项实体按键输入，同时描述硬件位置和它产生的统一命令。 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    BalanceCar_Command command;
} AppKeyBinding;

/* 修改按键功能只需改这一张表；K4目前明确映射为“无命令”。 */
static const AppKeyBinding app_key_bindings[] =
{
    {KEY_1_GPIO_Port, KEY_1_Pin, {.type = BALANCE_CAR_COMMAND_START}},
    {KEY_2_GPIO_Port, KEY_2_Pin, {.type = BALANCE_CAR_COMMAND_STOP}},
    {KEY_3_GPIO_Port, KEY_3_Pin, {.type = BALANCE_CAR_COMMAND_START}},
    {KEY_4_GPIO_Port, KEY_4_Pin, {.type = BALANCE_CAR_COMMAND_NONE}}
};

#define APP_KEY_COUNT (sizeof(app_key_bindings) / sizeof(app_key_bindings[0]))
#define APP_RADIANS_TO_DEGREES 57.2957795F
#define APP_TELEMETRY_FLAG_SPEED_I_LIMIT       (1U << 0U)
#define APP_TELEMETRY_FLAG_TARGET_PITCH_LIMIT  (1U << 1U)
#define APP_TELEMETRY_FLAG_TURN_LIMIT          (1U << 2U)
#define APP_TELEMETRY_FLAG_MOTOR_LIMIT         (1U << 3U)
#define APP_TELEMETRY_FLAG_COMMAND_TIMEOUT     (1U << 4U)
#define APP_TELEMETRY_FLAG_TURN_I_LIMIT        (1U << 5U)

/** 只有App使用整车状态，因此状态类型也留在本文件。 */
typedef enum
{
    BALANCE_CAR_STATE_CALIBRATING = 0,
    BALANCE_CAR_STATE_BALANCING,
    BALANCE_CAR_STATE_FALLEN,
    BALANCE_CAR_STATE_FAULT
} BalanceCar_State;

/*
 * 这只是本文件唯一一份运行数据的集中存放处，不是可创建、继承或传递的
 * “App对象”。私有函数直接使用它，避免给每层函数机械传递 app 指针。
 */
static struct
{
    /* 实际底板上的硬件实例。 */
    BSP_Motor left_motor;
    BSP_Motor right_motor;
    BSP_Encoder left_encoder;
    BSP_Encoder right_encoder;
    BSP_MPU6050 imu;
    BSP_BluetoothSerial bluetooth;
    BSP_Key keys[APP_KEY_COUNT];

    /* 有独立运行状态的应用模块。互补滤波历史直接放在state_estimate中。 */
    /** 速度外环包住角度内环，统一生成左右轮公共控制量。 */
    LongitudinalControl longitudinal_control;
    /** 转向环暂时独立；它与纵向输出如何组合由App直接决定。 */
    PID turn_pid;
    BluetoothProtocol bluetooth_protocol;

    /* 一个控制周期内在采样、估计、控制和安全之间传递的数据。 */
    BalanceCar_StateEstimate state_estimate;
    BalanceCar_ControlTarget control_target;
    /** GZ偏航角速度环输出的左右轮差分控制量。 */
    float turn_output;
    float balance_output;
    float left_speed_counts_per_s;
    float right_speed_counts_per_s;
    /** 上电以来两轮平均行驶里程，仅用于驻车位置环。 */
    float position_counts;
    /** 驻车位置环锁存的目标位置。 */
    float hold_position_counts;
    bool position_hold_active;
    bool turn_output_limited;
    /** 记录上一控制周期是否存在转向命令，用于松杆时只复位一次积分。 */
    bool turn_command_was_active;

    /* CALIBRATING同时测量俯仰轴和偏航轴随温度、上电变化的零偏。 */
    /** 校准期间保存截至当前样本的在线平均，完成后就是运行时俯仰零偏。 */
    float gyro_bias_dps;
    /** 绕竖直轴GZ的运行时零偏。 */
    float gyro_yaw_bias_dps;
    /** 最近一次由重力方向得到、已加固定安装修正的加速度角。 */
    float diagnostic_accel_pitch_deg;
    /** 未减零偏的GZ角速度，用于与校正值同图核对。 */
    float diagnostic_yaw_rate_raw_dps;
    uint16_t gyro_calibration_samples;

    /* HAL_GetTick() 驱动的协作式周期调度时间戳。 */
    uint32_t last_imu_sample_ms;
    uint32_t last_outer_loop_ms;
    uint32_t last_key_task_ms;
    uint32_t last_bluetooth_task_ms;
    uint32_t last_telemetry_ms;
    uint32_t last_display_page_ms;
    uint32_t led_changed_at_ms;
    uint32_t last_motion_command_ms;
    uint32_t motor_test_started_ms;

    /** OLED显示最近收到的合法蓝牙帧数，便于确认协议是否真正生效。 */
    uint32_t bluetooth_command_count;
    uint8_t display_page;

    /* 5ms读取增量，累计到20ms速度环窗口后再换算，降低低速量化噪声。 */
    int32_t left_encoder_counts;
    int32_t right_encoder_counts;
    /** 最近一次蓝牙点动期间累计的编码器增量，结束后保留在OLED上。 */
    int32_t motor_test_left_counts;
    int32_t motor_test_right_counts;

    BalanceCar_State state;
    bool bluetooth_available;
    bool motor_test_active;
} app;

/** 无论当前状态如何，都把两路驱动立即置零。 */
static void App_StopMotors(void)
{
    (void)BSP_Motor_Stop(&app.left_motor);
    (void)BSP_Motor_Stop(&app.right_motor);
    app.balance_output = 0.0F;
    app.turn_output = 0.0F;
    app.turn_output_limited = false;
    app.turn_command_was_active = false;
}

/** App直接完成公共量/差分量混合，并在交给BSP前做整车级限幅。 */
static int16_t App_ToMotorCommand(float command)
{
    if (command > (float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT)
    {
        command = (float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT;
    }
    else if (command < -(float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT)
    {
        command = -(float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT;
    }

    return (int16_t)command;
}

/**
 * 混合后按单个轮子的命令方向补偿电机死区。band内线性放大而不是过零
 * ±offset硬跳变，避免静止时命令符号高频翻转形成的抖振极限环；band为
 * 0时退化为原始硬补偿。代价是|命令|远小于band时轮子暂时推不动，由
 * 角度/速度环慢慢加大命令，表现为缓慢小幅摇摆而不是高频抖动。
 */
static float App_CompensateMotorDeadzone(float command)
{
    const float offset = app.control_target.motor_deadzone_offset;
    const float band = app.control_target.motor_deadzone_band;

    if (command > band)
    {
        return command + offset;
    }
    if (command < -band)
    {
        return command - offset;
    }
    if (band > 0.0F)
    {
        return command * (1.0F + offset / band);
    }
    return 0.0F;
}

/**
 * Y轴是俯仰转轴时，重力投影落在X/Z轴上。本车实测原始加速度角
 * 与Gyro_Y方向相反，因此在这里取负，统一约定“车体前倾为正”。
 */
static float App_CalculateAccelPitchDeg(
    const BSP_MPU6050_RawData *data)
{
    return -atan2f((float)data->Accel_X, (float)data->Accel_Z) *
           APP_RADIANS_TO_DEGREES;
}

/** 在FALLEN状态启动一次限幅、定时的电机点动，并清零编码器测试窗口。 */
static bool App_StartMotorTest(const BalanceCar_Command *command,
                               uint32_t now_ms)
{
    if (command == NULL ||
        command->type != BALANCE_CAR_COMMAND_TEST_MOTOR ||
        app.state != BALANCE_CAR_STATE_FALLEN || app.motor_test_active)
    {
        return false;
    }

    const int16_t test_command = command->data.motor_test.command;
    if (test_command < -BALANCE_CAR_MOTOR_TEST_MAX_COMMAND ||
        test_command > BALANCE_CAR_MOTOR_TEST_MAX_COMMAND)
    {
        return false;
    }

    int16_t left_command = 0;
    int16_t right_command = 0;
    switch (command->data.motor_test.selection)
    {
        case BALANCE_CAR_MOTOR_LEFT:
            left_command = test_command;
            break;
        case BALANCE_CAR_MOTOR_RIGHT:
            right_command = test_command;
            break;
        case BALANCE_CAR_MOTOR_BOTH:
            left_command = test_command;
            right_command = test_command;
            break;
        default:
            return false;
    }

    /* 丢弃上一次读取后遗留的计数，本次结果只覆盖300ms点动窗口。 */
    (void)BSP_Encoder_ReadDelta(&app.left_encoder);
    (void)BSP_Encoder_ReadDelta(&app.right_encoder);
    app.motor_test_left_counts = 0;
    app.motor_test_right_counts = 0;

    if (BSP_Motor_SetCommand(&app.left_motor, left_command) != HAL_OK ||
        BSP_Motor_SetCommand(&app.right_motor, right_command) != HAL_OK)
    {
        App_StopMotors();
        app.state = BALANCE_CAR_STATE_FAULT;
        return false;
    }

    app.motor_test_started_ms = now_ms;
    app.motor_test_active = true;
    return true;
}

/** FALLEN仍采集MPU；若正在点动则同步累计编码器并在300ms后自动停机。 */
static void App_UpdateFallenDiagnostics(uint32_t now_ms)
{
    const uint32_t imu_elapsed_ms = now_ms - app.last_imu_sample_ms;
    if (imu_elapsed_ms >= BALANCE_CAR_IMU_SAMPLE_PERIOD_MS)
    {
        app.last_imu_sample_ms = now_ms;
        BSP_MPU6050_RawData raw;
        if (MPU6050_ReadRaw(&app.imu, &raw) != HAL_OK)
        {
            app.motor_test_active = false;
            App_StopMotors();
            app.state = BALANCE_CAR_STATE_FAULT;
            return;
        }

        app.diagnostic_accel_pitch_deg =
            App_CalculateAccelPitchDeg(&raw) +
            BALANCE_CAR_ACCEL_PITCH_MOUNT_OFFSET_DEG;
        app.state_estimate.pitch_deg = app.diagnostic_accel_pitch_deg;
        app.state_estimate.pitch_rate_dps =
            (float)raw.Gyro_Y / BALANCE_CAR_MPU_GYRO_LSB_PER_DPS -
            app.gyro_bias_dps;
        app.diagnostic_yaw_rate_raw_dps =
            (float)raw.Gyro_Z / BALANCE_CAR_MPU_GYRO_LSB_PER_DPS;
        app.state_estimate.yaw_rate_dps =
            app.diagnostic_yaw_rate_raw_dps - app.gyro_yaw_bias_dps;
    }

    if (!app.motor_test_active)
    {
        App_StopMotors();
        return;
    }

    app.motor_test_left_counts +=
        BSP_Encoder_ReadDelta(&app.left_encoder);
    app.motor_test_right_counts +=
        BSP_Encoder_ReadDelta(&app.right_encoder);

    if ((uint32_t)(now_ms - app.motor_test_started_ms) >=
        BALANCE_CAR_MOTOR_TEST_DURATION_MS)
    {
        App_StopMotors();
        app.motor_test_active = false;
    }
}

/** OLED统一用“符号+三位整数+一位小数”，避免引入printf浮点格式化。 */
static void App_OLEDPrintFixed1(uint8_t x, uint8_t y, float value)
{
    if (value != value)
    {
        value = 0.0F;
    }
    if (value > 999.9F)
    {
        value = 999.9F;
    }
    else if (value < -999.9F)
    {
        value = -999.9F;
    }

    const bool negative = value < 0.0F;
    if (negative)
    {
        value = -value;
    }
    const uint32_t scaled = (uint32_t)(value * 10.0F + 0.5F);
    char text[7] = {
        negative ? '-' : '+',
        (char)('0' + scaled / 1000U),
        (char)('0' + scaled / 100U % 10U),
        (char)('0' + scaled / 10U % 10U),
        '.',
        (char)('0' + scaled % 10U),
        '\0'
    };
    OLED_PrintASCIIString(x, y, text, &afont8x6, OLED_COLOR_NORMAL);
}

/**
 * 完成一次5ms闭环周期：采样、换算、互补滤波、控制、安全门控和电机输出。
 * 角度环和GZ转向环以200Hz运行，编码器速度外环以50Hz运行。
 */
static void App_UpdateBalancing(uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - app.last_imu_sample_ms;
    if (elapsed_ms < BALANCE_CAR_IMU_SAMPLE_PERIOD_MS)
    {
        return;
    }

    BSP_MPU6050_RawData raw;
    if (MPU6050_ReadRaw(&app.imu, &raw) != HAL_OK)
    {
        app.state = BALANCE_CAR_STATE_FAULT;
        return;
    }

    const float dt_seconds = (float)elapsed_ms * 0.001F;
    app.last_imu_sample_ms = now_ms;

    /* 陀螺仪负责短期变化；先减掉上电静止采样得到的零偏。 */
    const float gyro_pitch_dps =
        (float)raw.Gyro_Y / BALANCE_CAR_MPU_GYRO_LSB_PER_DPS -
        app.gyro_bias_dps;
    app.diagnostic_yaw_rate_raw_dps =
        (float)raw.Gyro_Z / BALANCE_CAR_MPU_GYRO_LSB_PER_DPS;
    app.state_estimate.yaw_rate_dps =
        app.diagnostic_yaw_rate_raw_dps - app.gyro_yaw_bias_dps;

    /* 加速度计负责长期绝对参考；安装角只使用固定、可重复的修正量。 */
    const float accel_pitch_deg =
        App_CalculateAccelPitchDeg(&raw) +
        BALANCE_CAR_ACCEL_PITCH_MOUNT_OFFSET_DEG;
    app.diagnostic_accel_pitch_deg = accel_pitch_deg;

    app.state_estimate.pitch_rate_dps = gyro_pitch_dps;
    const float filter_tau = BALANCE_CAR_COMPLEMENTARY_TIME_CONSTANT_S;
    const float alpha = filter_tau / (filter_tau + dt_seconds);
    app.state_estimate.pitch_deg =
        alpha * (app.state_estimate.pitch_deg +
                 gyro_pitch_dps * dt_seconds) +
        (1.0F - alpha) * accel_pitch_deg;

    /* 高频消费硬件计数，低频速度环再使用完整20ms窗口计算速度。 */
    app.left_encoder_counts += BSP_Encoder_ReadDelta(&app.left_encoder);
    app.right_encoder_counts += BSP_Encoder_ReadDelta(&app.right_encoder);

    /* 倾角越界只负责切换状态，FALLEN分支负责持续停机。 */
    if (fabsf(app.state_estimate.pitch_deg) >
        BALANCE_CAR_DEFAULT_MAX_TILT_DEG)
    {
        app.state = BALANCE_CAR_STATE_FALLEN;
        return;
    }

    const uint32_t outer_elapsed_ms = now_ms - app.last_outer_loop_ms;
    if (outer_elapsed_ms >= BALANCE_CAR_OUTER_LOOP_PERIOD_MS)
    {
        app.last_outer_loop_ms = now_ms;
        const float outer_dt_seconds =
            (float)outer_elapsed_ms * 0.001F;

        const float left_speed_counts_per_s =
            (float)app.left_encoder_counts / outer_dt_seconds;
        const float right_speed_counts_per_s =
            (float)app.right_encoder_counts / outer_dt_seconds;
        app.left_speed_counts_per_s = left_speed_counts_per_s;
        app.right_speed_counts_per_s = right_speed_counts_per_s;
        app.position_counts += 0.5F *
            (float)(app.left_encoder_counts + app.right_encoder_counts);
        app.left_encoder_counts = 0;
        app.right_encoder_counts = 0;

        app.state_estimate.forward_speed_counts_per_s =
            0.5F * (left_speed_counts_per_s + right_speed_counts_per_s);
        app.state_estimate.turn_speed_counts_per_s =
            0.5F * (right_speed_counts_per_s - left_speed_counts_per_s);

        /*
         * 驻车位置环：无纵向命令且车速已降入门限时锁存当前位置，此后
         * 将位置误差转为小速度目标。速度环只会把速度压到0，不会撤销
         * 原地转向差分不对称、松杆滑行和地面坡度已经搬走的位移。
         */
        float speed_target_counts_per_s =
            app.control_target.forward_speed_counts_per_s;
        if (speed_target_counts_per_s != 0.0F ||
            app.control_target.position_hold_kp <= 0.0F)
        {
            app.position_hold_active = false;
        }
        else if (!app.position_hold_active &&
                 fabsf(app.state_estimate.forward_speed_counts_per_s) <
                     BALANCE_CAR_POSITION_HOLD_ENGAGE_SPEED_COUNTS_PER_S)
        {
            app.position_hold_active = true;
            app.hold_position_counts = app.position_counts;
        }
        if (app.position_hold_active)
        {
            float hold_speed_counts_per_s =
                (app.hold_position_counts - app.position_counts) *
                app.control_target.position_hold_kp;
            if (hold_speed_counts_per_s >
                BALANCE_CAR_POSITION_HOLD_MAX_SPEED_COUNTS_PER_S)
            {
                hold_speed_counts_per_s =
                    BALANCE_CAR_POSITION_HOLD_MAX_SPEED_COUNTS_PER_S;
            }
            else if (hold_speed_counts_per_s <
                     -BALANCE_CAR_POSITION_HOLD_MAX_SPEED_COUNTS_PER_S)
            {
                hold_speed_counts_per_s =
                    -BALANCE_CAR_POSITION_HOLD_MAX_SPEED_COUNTS_PER_S;
            }
            speed_target_counts_per_s = hold_speed_counts_per_s;
        }

        (void)LongitudinalControl_UpdateSpeed(
            &app.longitudinal_control,
            speed_target_counts_per_s,
            app.state_estimate.forward_speed_counts_per_s,
            app.control_target.balance_trim_deg,
            outer_dt_seconds);

    }

    /*
     * 偏航角速度环直接使用已减去上电静止零偏的GZ。实测左转为正、
     * 右转为负；正Kp配合下方差分混合会产生反向修正。停止转向且GZ
     * 已进入零点噪声带时直接清零，避免零偏或I项留下持续差分命令。
     */
    app.turn_output_limited = false;
    const bool turn_command_active =
        app.control_target.yaw_rate_dps != 0.0F;
    if (!turn_command_active && app.turn_command_was_active)
    {
        /*
         * 转向目标回零意味着从“维持旋转”切换到“停止旋转”。此前为
         * 持续转向积累的I项已经失效，立即复位，避免它抵消P项刹车。
         */
        PID_Reset(&app.turn_pid);
    }
    app.turn_command_was_active = turn_command_active;
    const bool vehicle_stationary =
        app.control_target.forward_speed_counts_per_s == 0.0F &&
        fabsf(app.state_estimate.forward_speed_counts_per_s) <
            BALANCE_CAR_TURN_INTEGRAL_RESET_SPEED_COUNTS_PER_S;
    if (app.control_target.yaw_rate_dps == 0.0F &&
        vehicle_stationary &&
        fabsf(app.state_estimate.yaw_rate_dps) <
            BALANCE_CAR_YAW_RATE_DEADBAND_DPS)
    {
        PID_Reset(&app.turn_pid);
        app.turn_output = 0.0F;
    }
    else
    {
        app.turn_output = PID_Update(
            &app.turn_pid,
            app.control_target.yaw_rate_dps,
            app.state_estimate.yaw_rate_dps,
            dt_seconds);
        app.turn_output_limited =
            fabsf(app.turn_output) >=
                BALANCE_CAR_DEFAULT_MAX_TURN_OUTPUT - 0.5F;
    }

    const float balance_output = LongitudinalControl_UpdateAngle(
        &app.longitudinal_control,
        app.state_estimate.pitch_deg,
        app.state_estimate.pitch_rate_dps,
        dt_seconds);
    app.balance_output = balance_output;

    /*
     * 公共量控制前后和平衡，差分量控制转向。差分量还要受当前公共量
     * 剩余余量限制，否则某一轮先饱和会把纯转向重新混成纵向运动。
     * 余量再扣掉死区补偿，保证补偿后的最终命令仍不会被限幅截断。
     */
    float turn_headroom =
        (float)BALANCE_CAR_DEFAULT_MOTOR_LIMIT -
        app.control_target.motor_deadzone_offset -
        fabsf(balance_output);
    if (turn_headroom < 0.0F)
    {
        turn_headroom = 0.0F;
    }
    if (app.turn_output > turn_headroom)
    {
        app.turn_output = turn_headroom;
        app.turn_output_limited = true;
    }
    else if (app.turn_output < -turn_headroom)
    {
        app.turn_output = -turn_headroom;
        app.turn_output_limited = true;
    }

    /* 死区随单个轮子的实际转向翻转，因此补偿必须在差分混合之后做。 */
    const int16_t left_motor_command = App_ToMotorCommand(
        App_CompensateMotorDeadzone(balance_output - app.turn_output));
    const int16_t right_motor_command = App_ToMotorCommand(
        App_CompensateMotorDeadzone(balance_output + app.turn_output));

    if (BSP_Motor_SetCommand(&app.left_motor,
                             left_motor_command) != HAL_OK ||
        BSP_Motor_SetCommand(&app.right_motor,
                             right_motor_command) != HAL_OK)
    {
        app.state = BALANCE_CAR_STATE_FAULT;
    }

}

static void App_InitKeys(uint32_t now_ms)
{
    for (uint32_t index = 0U; index < APP_KEY_COUNT; index++)
    {
        const AppKeyBinding *binding = &app_key_bindings[index];
        BSP_Key_Init(&app.keys[index],
                     binding->port,
                     binding->pin,
                     GPIO_PIN_RESET,
                     BALANCE_CAR_KEY_DEBOUNCE_MS,
                     now_ms);
    }
}

/** 核心硬件任意一项失败都返回错误，禁止进入可驱动电机的状态。 */
static HAL_StatusTypeDef App_InitCoreHardware(void)
{
    HAL_StatusTypeDef status;

    BSP_LED_OFF();
    App_InitKeys(HAL_GetTick());

    status = BSP_Motor_Init(&app.left_motor,
                            &htim2, TIM_CHANNEL_1,
                            AIN1_GPIO_Port, AIN1_Pin,
                            AIN2_GPIO_Port, AIN2_Pin,
                            BALANCE_CAR_LEFT_MOTOR_INVERTED);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_Motor_Init(&app.right_motor,
                            &htim2, TIM_CHANNEL_2,
                            BIN1_GPIO_Port, BIN1_Pin,
                            BIN2_GPIO_Port, BIN2_Pin,
                            BALANCE_CAR_RIGHT_MOTOR_INVERTED);
    if (status != HAL_OK)
    {
        return status;
    }
    App_StopMotors();

    status = BSP_Encoder_Init(&app.left_encoder,
                              &htim3,
                              BALANCE_CAR_LEFT_ENCODER_INVERTED);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_Encoder_Init(&app.right_encoder,
                              &htim4,
                              BALANCE_CAR_RIGHT_ENCODER_INVERTED);
    if (status != HAL_OK)
    {
        return status;
    }

    return MPU6050_Init(&app.imu,
                        &hi2c2,
                        BALANCE_CAR_MPU_ADDRESS);
}

HAL_StatusTypeDef App_Init(void)
{
    memset(&app, 0, sizeof(app));
    app.led_changed_at_ms = HAL_GetTick();

    const HAL_StatusTypeDef status = App_InitCoreHardware();
    if (status != HAL_OK)
    {
        app.state = BALANCE_CAR_STATE_FAULT;
        return status;
    }

    /* OLED是调参诊断界面；启动阶段允许一次完整阻塞初始化。 */
    OLED_Init();

    /* 蓝牙是可选能力：失败只关闭调参与遥测，不阻止核心控制运行。 */
    app.bluetooth_available =
        BSP_BluetoothSerial_Init(&app.bluetooth, &huart2) == HAL_OK;

    const uint32_t now_ms = HAL_GetTick();
    LongitudinalControl_Init(&app.longitudinal_control);
    app.control_target.balance_trim_deg =
        BALANCE_CAR_DEFAULT_BALANCE_TRIM_DEG;
    app.control_target.motor_deadzone_offset =
        BALANCE_CAR_DEFAULT_MOTOR_DEADZONE_OFFSET;
    app.control_target.motor_deadzone_band =
        BALANCE_CAR_DEFAULT_MOTOR_DEADZONE_BAND;
    app.control_target.position_hold_kp =
        BALANCE_CAR_DEFAULT_POSITION_HOLD_KP;
    app.longitudinal_control.target_pitch_deg =
        app.control_target.balance_trim_deg;
    app.turn_pid = (PID){
        .kp = BALANCE_CAR_DEFAULT_TURN_KP,
        .ki = BALANCE_CAR_DEFAULT_TURN_KI,
        .kd = BALANCE_CAR_DEFAULT_TURN_KD,
        .output_min = -BALANCE_CAR_DEFAULT_MAX_TURN_OUTPUT,
        .output_max = BALANCE_CAR_DEFAULT_MAX_TURN_OUTPUT,
        .integral_min = -BALANCE_CAR_DEFAULT_MAX_TURN_INTEGRAL_OUTPUT,
        .integral_max = BALANCE_CAR_DEFAULT_MAX_TURN_INTEGRAL_OUTPUT,
    };
    BluetoothProtocol_Init(
        &app.bluetooth_protocol,
        app.bluetooth_available ? &app.bluetooth : NULL);

    app.last_outer_loop_ms = now_ms;
    app.last_key_task_ms = now_ms;
    app.last_bluetooth_task_ms = now_ms;
    app.last_telemetry_ms = now_ms;
    app.last_display_page_ms = now_ms;
    app.last_motion_command_ms = now_ms;

    app.state = BALANCE_CAR_STATE_CALIBRATING;
    app.last_imu_sample_ms = now_ms;
    return HAL_OK;
}

/** 应用命令，并由本状态机裁决启动和停止动作是否合法。 */
static bool App_ExecuteCommand(const BalanceCar_Command *command,
                               uint32_t now_ms)
{
    const CommandAction action = Command_Apply(
        command,
        &app.control_target,
        &app.longitudinal_control,
        &app.turn_pid);

    switch (action)
    {
        case COMMAND_ACTION_MOTION_UPDATED:
            app.last_motion_command_ms = now_ms;
            return true;

        case COMMAND_ACTION_APPLIED:
            return true;

        case COMMAND_ACTION_START:
            if (app.state == BALANCE_CAR_STATE_FALLEN)
            {
                app.motor_test_active = false;
                App_StopMotors();
                app.state = BALANCE_CAR_STATE_CALIBRATING;
                app.gyro_bias_dps = 0.0F;
                app.gyro_yaw_bias_dps = 0.0F;
                app.gyro_calibration_samples = 0U;
                app.last_imu_sample_ms = now_ms;
                app.state_estimate.pitch_deg = 0.0F;
                app.state_estimate.pitch_rate_dps = 0.0F;
                app.state_estimate.yaw_rate_dps = 0.0F;
                app.left_encoder_counts = 0;
                app.right_encoder_counts = 0;
                app.position_counts = 0.0F;
                app.hold_position_counts = 0.0F;
                app.position_hold_active = false;
                LongitudinalControl_Reset(&app.longitudinal_control);
                app.longitudinal_control.target_pitch_deg =
                    app.control_target.balance_trim_deg;
                PID_Reset(&app.turn_pid);
                app.turn_output = 0.0F;
                return true;
            }
            return false;

        case COMMAND_ACTION_STOP:
            if (app.motor_test_active)
            {
                app.motor_test_active = false;
                App_StopMotors();
                return true;
            }
            if (app.state == BALANCE_CAR_STATE_BALANCING)
            {
                app.state = BALANCE_CAR_STATE_FALLEN;
                return true;
            }
            return false;

        case COMMAND_ACTION_MOTOR_TEST:
            return App_StartMotorTest(command, now_ms);

        case COMMAND_ACTION_INVALID:
        default:
            return false;
    }
}

/** 只消费 DMA 已经收到的字节，不等待新数据到来。 */
static void App_UpdateBluetooth(uint32_t now_ms)
{
    if (!app.bluetooth_available ||
        (uint32_t)(now_ms - app.last_bluetooth_task_ms) <
            BALANCE_CAR_BLUETOOTH_TASK_PERIOD_MS)
    {
        return;
    }

    app.last_bluetooth_task_ms = now_ms;
    BalanceCar_Command command;
    if (BluetoothProtocol_Poll(&app.bluetooth_protocol, &command))
    {
        app.bluetooth_command_count++;
        (void)App_ExecuteCommand(&command, now_ms);
    }
}

/**
 * 一页一页刷新调参界面，避免一次发送1KB显存阻塞200Hz控制环。
 * A/S/T分别为角度、速度、转向环；O/B为输出偏移和平衡零点。
 */
static void App_UpdateDisplay(uint32_t now_ms)
{
    /*
     * OLED分页发送仍会阻塞CPU约数毫秒。平衡诊断期间完全停止刷新，避免它
     * 给5ms角度环引入周期性执行延迟；进入FALLEN后屏幕会继续更新。
     */
    if (app.state == BALANCE_CAR_STATE_BALANCING)
    {
        return;
    }

    if ((uint32_t)(now_ms - app.last_display_page_ms) <
        BALANCE_CAR_DISPLAY_PAGE_PERIOD_MS)
    {
        return;
    }
    app.last_display_page_ms = now_ms;

    if (app.display_page == 0U)
    {
        OLED_NewFrame();

        OLED_PrintASCIIChar(24, 0, 'P', &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIChar(64, 0, 'I', &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIChar(104, 0, 'D', &afont8x6, OLED_COLOR_NORMAL);

        const PID *const pids[] = {
            &app.longitudinal_control.angle.pid,
            &app.longitudinal_control.speed_pid,
            &app.turn_pid,
        };
        const char labels[] = {'A', 'S', 'T'};
        for (uint32_t index = 0U; index < 3U; index++)
        {
            if (app.state == BALANCE_CAR_STATE_FALLEN && index > 0U)
            {
                continue;
            }
            const uint8_t y = (uint8_t)((index + 1U) * 8U);
            OLED_PrintASCIIChar(0, y, labels[index],
                                &afont8x6, OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(8, y, pids[index]->kp);
            App_OLEDPrintFixed1(48, y, pids[index]->ki);
            App_OLEDPrintFixed1(88, y, pids[index]->kd);
        }

        if (app.state == BALANCE_CAR_STATE_FALLEN)
        {
            OLED_PrintASCIIChar(0, 16, 'A', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(8, 16, app.diagnostic_accel_pitch_deg);
            OLED_PrintASCIIChar(48, 16, 'G', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(56, 16, app.state_estimate.pitch_rate_dps);

            OLED_PrintASCIIChar(0, 24, 'L', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(8, 24,
                                (float)app.motor_test_left_counts);
            OLED_PrintASCIIChar(48, 24, 'R', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(56, 24,
                                (float)app.motor_test_right_counts);
        }

        OLED_PrintASCIIChar(0, 32, 'O', &afont8x6, OLED_COLOR_NORMAL);
        App_OLEDPrintFixed1(8, 32,
                            app.control_target.motor_deadzone_offset);
        OLED_PrintASCIIChar(48, 32, 'B', &afont8x6, OLED_COLOR_NORMAL);
        App_OLEDPrintFixed1(56, 32, app.control_target.balance_trim_deg);

        if (app.state == BALANCE_CAR_STATE_FALLEN)
        {
            /* Z是校正后的GZ；B是上电静止采样得到的GZ零偏。 */
            OLED_PrintASCIIChar(0, 40, 'Z', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(8, 40, app.state_estimate.yaw_rate_dps);
            OLED_PrintASCIIChar(48, 40, 'B', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(56, 40, app.gyro_yaw_bias_dps);
        }
        else
        {
            OLED_PrintASCIIChar(0, 40, 'P', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(8, 40, app.state_estimate.pitch_deg);
            OLED_PrintASCIIChar(48, 40, 'G', &afont8x6,
                                OLED_COLOR_NORMAL);
            App_OLEDPrintFixed1(56, 40,
                                app.state_estimate.pitch_rate_dps);
        }

        OLED_PrintASCIIChar(0, 48, 'L', &afont8x6, OLED_COLOR_NORMAL);
        App_OLEDPrintFixed1(8, 48,
                            (float)BSP_Motor_GetCommand(&app.left_motor) /
                            10.0F);
        OLED_PrintASCIIChar(48, 48, 'R', &afont8x6, OLED_COLOR_NORMAL);
        App_OLEDPrintFixed1(56, 48,
                            (float)BSP_Motor_GetCommand(&app.right_motor) /
                            10.0F);

        char *state_text;
        switch (app.state)
        {
            case BALANCE_CAR_STATE_CALIBRATING: state_text = "CAL"; break;
            case BALANCE_CAR_STATE_BALANCING:   state_text = "BAL"; break;
            case BALANCE_CAR_STATE_FALLEN:
                state_text = app.motor_test_active ? "TST" : "FAL";
                break;
            case BALANCE_CAR_STATE_FAULT:       state_text = "FLT"; break;
            default:                            state_text = "???"; break;
        }
        OLED_PrintASCIIString(0, 56, state_text,
                              &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(24, 56, "BT", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIChar(42, 56, app.bluetooth_available ? '1' : '0',
                            &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(54, 56, "RX", &afont8x6, OLED_COLOR_NORMAL);
        const uint32_t rx = app.bluetooth_command_count % 1000U;
        OLED_PrintASCIIChar(72, 56, (char)('0' + rx / 100U),
                            &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIChar(78, 56, (char)('0' + rx / 10U % 10U),
                            &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIChar(84, 56, (char)('0' + rx % 10U),
                            &afont8x6, OLED_COLOR_NORMAL);
    }

    OLED_ShowPage(app.display_page);
    app.display_page = (uint8_t)((app.display_page + 1U) % 8U);
}

static void App_UpdateKeys(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - app.last_key_task_ms) <
        BALANCE_CAR_KEY_TASK_PERIOD_MS)
    {
        return;
    }

    app.last_key_task_ms = now_ms;
    for (uint32_t index = 0U; index < APP_KEY_COUNT; index++)
    {
        BSP_Key_Update(&app.keys[index], now_ms);
        if (BSP_Key_TakeEvent(&app.keys[index]) != BSP_KEY_EVENT_PRESSED)
        {
            continue;
        }

        const BalanceCar_Command *command = &app_key_bindings[index].command;
        if (command->type != BALANCE_CAR_COMMAND_NONE)
        {
            (void)App_ExecuteCommand(command, now_ms);
        }
    }
}

/** 用不同闪烁周期表示状态，不使用 HAL_Delay() 阻塞主循环。 */
static void App_UpdateStatusLed(uint32_t now_ms)
{
    uint32_t toggle_period_ms = 0U;

    switch (app.state)
    {
        case BALANCE_CAR_STATE_CALIBRATING:
            toggle_period_ms = 250U;
            break;
        case BALANCE_CAR_STATE_FALLEN:
            toggle_period_ms = 150U;
            break;
        case BALANCE_CAR_STATE_FAULT:
            toggle_period_ms = 75U;
            break;
        case BALANCE_CAR_STATE_BALANCING:
            BSP_LED_ON();
            return;
        default:
            BSP_LED_OFF();
            return;
    }

    if ((uint32_t)(now_ms - app.led_changed_at_ms) >= toggle_period_ms)
    {
        app.led_changed_at_ms = now_ms;
        BSP_LED_Toggle();
    }
}

/**
 * 串口正在发送时DMA BSP会返回忙；诊断输出不会拖住控制任务。
 * 平衡时三条曲线依次为校正后GZ/10、转向差分输出/100、整车速度/1000。
 * FALLEN诊断时改为GZ原始角速度、减零偏后的GZ、GZ零偏，三者单位均为dps。
 */
static void App_UpdateDiagnostics(uint32_t now_ms)
{
    if (app.bluetooth_available &&
        (uint32_t)(now_ms - app.last_telemetry_ms) >=
            BALANCE_CAR_TELEMETRY_PERIOD_MS)
    {
        app.last_telemetry_ms = now_ms;
        uint8_t flags = 0U;
        if (fabsf(app.longitudinal_control.speed_pid.integral_output) >=
            BALANCE_CAR_DEFAULT_MAX_SPEED_INTEGRAL_PITCH_DEG - 0.001F)
        {
            flags |= APP_TELEMETRY_FLAG_SPEED_I_LIMIT;
        }
        if (fabsf(app.longitudinal_control.target_pitch_deg) >=
            BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG - 0.01F)
        {
            flags |= APP_TELEMETRY_FLAG_TARGET_PITCH_LIMIT;
        }
        if (app.turn_output_limited)
        {
            flags |= APP_TELEMETRY_FLAG_TURN_LIMIT;
        }
        if (fabsf(app.turn_pid.integral_output) >=
            BALANCE_CAR_DEFAULT_MAX_TURN_INTEGRAL_OUTPUT - 0.1F)
        {
            flags |= APP_TELEMETRY_FLAG_TURN_I_LIMIT;
        }
        const int16_t left_motor_command =
            BSP_Motor_GetCommand(&app.left_motor);
        const int16_t right_motor_command =
            BSP_Motor_GetCommand(&app.right_motor);
        if (abs(left_motor_command) >= BALANCE_CAR_DEFAULT_MOTOR_LIMIT ||
            abs(right_motor_command) >= BALANCE_CAR_DEFAULT_MOTOR_LIMIT)
        {
            flags |= APP_TELEMETRY_FLAG_MOTOR_LIMIT;
        }
        if ((uint32_t)(now_ms - app.last_motion_command_ms) >=
            BALANCE_CAR_COMMAND_TIMEOUT_MS)
        {
            flags |= APP_TELEMETRY_FLAG_COMMAND_TIMEOUT;
        }

        const BalanceCar_TelemetrySample sample = {
            .timestamp_ms = now_ms,
            .state = (uint8_t)app.state,
            .flags = flags,
            .pitch_deg = app.state_estimate.pitch_deg,
            .accel_pitch_deg = app.diagnostic_accel_pitch_deg,
            .pitch_rate_dps = app.state_estimate.pitch_rate_dps,
            .yaw_rate_dps = app.state_estimate.yaw_rate_dps,
            .yaw_bias_dps = app.gyro_yaw_bias_dps,
            .left_speed_counts_per_s = app.left_speed_counts_per_s,
            .right_speed_counts_per_s = app.right_speed_counts_per_s,
            .forward_speed_counts_per_s =
                app.state_estimate.forward_speed_counts_per_s,
            .turn_speed_counts_per_s =
                app.state_estimate.turn_speed_counts_per_s,
            .target_speed_counts_per_s =
                app.control_target.forward_speed_counts_per_s,
            .target_pitch_deg = app.longitudinal_control.target_pitch_deg,
            .target_yaw_rate_dps = app.control_target.yaw_rate_dps,
            .speed_integral_pitch_deg =
                app.longitudinal_control.speed_pid.integral_output,
            .balance_output = app.balance_output,
            .turn_output = app.turn_output,
            .left_motor_command = (float)left_motor_command,
            .right_motor_command = (float)right_motor_command,
        };
        const BalanceCar_TelemetryParameters parameters = {
            .angle_kp = app.longitudinal_control.angle.pid.kp,
            .angle_ki = app.longitudinal_control.angle.pid.ki,
            .angle_kd = app.longitudinal_control.angle.pid.kd,
            .speed_kp = app.longitudinal_control.speed_pid.kp,
            .speed_ki = app.longitudinal_control.speed_pid.ki,
            .speed_kd = app.longitudinal_control.speed_pid.kd,
            .turn_kp = app.turn_pid.kp,
            .turn_ki = app.turn_pid.ki,
            .turn_kd = app.turn_pid.kd,
            .balance_trim_deg = app.control_target.balance_trim_deg,
            .motor_deadzone_offset =
                app.control_target.motor_deadzone_offset,
            .motor_deadzone_band =
                app.control_target.motor_deadzone_band,
            .position_hold_kp =
                app.control_target.position_hold_kp,
            .speed_integral_limit_deg =
                BALANCE_CAR_DEFAULT_MAX_SPEED_INTEGRAL_PITCH_DEG,
            .target_pitch_limit_deg =
                BALANCE_CAR_DEFAULT_MAX_TARGET_PITCH_DEG,
            .motor_output_limit = BALANCE_CAR_DEFAULT_MOTOR_LIMIT,
            .turn_output_limit = BALANCE_CAR_DEFAULT_MAX_TURN_OUTPUT,
            .turn_integral_limit =
                BALANCE_CAR_DEFAULT_MAX_TURN_INTEGRAL_OUTPUT,
            .joystick_speed_limit =
                BALANCE_CAR_JOYSTICK_MAX_FORWARD_SPEED,
            .joystick_yaw_rate_limit_dps =
                BALANCE_CAR_JOYSTICK_MAX_YAW_RATE_DPS,
            .command_timeout_ms = BALANCE_CAR_COMMAND_TIMEOUT_MS,
        };
        (void)BluetoothProtocol_SendTelemetry(
            &app.bluetooth_protocol, &sample, &parameters);
    }
}

void App_Update(void)
{
    /* 一轮只读取一次毫秒时基，使本轮所有任务使用同一个时间快照。 */
    const uint32_t now_ms = HAL_GetTick();

    /* 输入任务在所有状态都运行，由命令处理处直接修改整车状态。 */
    App_UpdateBluetooth(now_ms);
    App_UpdateKeys(now_ms);
    if ((uint32_t)(now_ms - app.last_motion_command_ms) >=
        BALANCE_CAR_COMMAND_TIMEOUT_MS)
    {
        app.control_target.forward_speed_counts_per_s = 0.0F;
        app.control_target.yaw_rate_dps = 0.0F;
    }

    switch (app.state)
    {
        case BALANCE_CAR_STATE_CALIBRATING:
            /* 校准状态本身负责保持停机，完成采样后自动进入平衡。 */
            App_StopMotors();
            if (app.gyro_calibration_samples >=
                BALANCE_CAR_CALIBRATION_SAMPLE_COUNT)
            {
                /*
                 * 用最后一次绝对重力角初始化滤波器，避免从0度向真实角度
                 * 缓慢收敛。这里不再把每次手扶姿态重新定义成零点。
                 */
                app.state_estimate.pitch_deg =
                    app.diagnostic_accel_pitch_deg;
                app.state_estimate.pitch_rate_dps = 0.0F;
                app.state_estimate.yaw_rate_dps = 0.0F;
                app.last_imu_sample_ms = now_ms;
                app.last_outer_loop_ms = now_ms;
                app.longitudinal_control.target_pitch_deg =
                    app.control_target.balance_trim_deg;

                /* 丢弃校准期间积累的计数，避免首次速度被错误放大。 */
                (void)BSP_Encoder_ReadDelta(&app.left_encoder);
                (void)BSP_Encoder_ReadDelta(&app.right_encoder);
                app.left_encoder_counts = 0;
                app.right_encoder_counts = 0;
                app.position_counts = 0.0F;
                app.hold_position_counts = 0.0F;
                app.position_hold_active = false;
                app.state = BALANCE_CAR_STATE_BALANCING;
                break;
            }

            if ((uint32_t)(now_ms - app.last_imu_sample_ms) <
                BALANCE_CAR_IMU_SAMPLE_PERIOD_MS)
            {
                break;
            }

            app.last_imu_sample_ms = now_ms;
            BSP_MPU6050_RawData calibration_data;
            if (MPU6050_ReadRaw(&app.imu, &calibration_data) != HAL_OK)
            {
                app.state = BALANCE_CAR_STATE_FAULT;
                break;
            }

            const float gyro_pitch_dps = (float)calibration_data.Gyro_Y /
                BALANCE_CAR_MPU_GYRO_LSB_PER_DPS;
            const float gyro_yaw_dps = (float)calibration_data.Gyro_Z /
                BALANCE_CAR_MPU_GYRO_LSB_PER_DPS;
            app.diagnostic_accel_pitch_deg =
                App_CalculateAccelPitchDeg(&calibration_data) +
                BALANCE_CAR_ACCEL_PITCH_MOUNT_OFFSET_DEG;
            app.gyro_calibration_samples++;
            app.gyro_bias_dps +=
                (gyro_pitch_dps - app.gyro_bias_dps) /
                (float)app.gyro_calibration_samples;
            app.gyro_yaw_bias_dps +=
                (gyro_yaw_dps - app.gyro_yaw_bias_dps) /
                (float)app.gyro_calibration_samples;
            break;

        case BALANCE_CAR_STATE_BALANCING:
            App_UpdateBalancing(now_ms);
            break;

        case BALANCE_CAR_STATE_FALLEN:
            /* 默认持续停机；蓝牙点动由本分支限时执行，不绕过状态机。 */
            App_UpdateFallenDiagnostics(now_ms);
            break;

        default:
            app.state = BALANCE_CAR_STATE_FAULT;
            /* fall through：非法状态立即执行FAULT动作。 */
        case BALANCE_CAR_STATE_FAULT:
            /* FAULT的状态动作同样是持续停机，并且只允许复位恢复。 */
            app.motor_test_active = false;
            App_StopMotors();
            break;
    }

    App_UpdateDiagnostics(now_ms);
    App_UpdateDisplay(now_ms);
    App_UpdateStatusLed(now_ms);
}

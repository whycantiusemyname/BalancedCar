#include "BluetoothDiagnostic.h"

#include "BlueToothSerial.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "MPU6050.h"
#include "Motor.h"
#include "i2c.h"
#include "main.h"
#include "oled.h"
#include "tim.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RAW_BANNER_PERIOD_MS  2000U
#define RAW_LED_PERIOD_MS      250U
#define ERROR_LED_PERIOD_MS    100U
#define COMMAND_IDLE_TIMEOUT_MS 50U
#define COMMAND_BUFFER_SIZE     64U
#define MPU_SAMPLE_PERIOD_MS     20U
#define OLED_REFRESH_PERIOD_MS  250U
#define ENCODER_SAMPLE_PERIOD_MS 20U
#define MOTOR_TEST_MAGIC          0x4D545354UL
#define MOTOR_TEST_COMMAND        250
#define MOTOR_TEST_MAX_TIME_MS    250U
#define MOTOR_TEST_MAX_COUNTS     160
#define MOTOR_TEST_SETTLE_MS      250U
#define MPU_AVERAGE_SETTLE_MS  2000U
#define MPU_AVERAGE_SAMPLE_MS    10U
#define MPU_AVERAGE_SAMPLE_COUNT 1000U

#define MPU6050_ADDRESS_LOW      0x68U
#define MPU6050_ADDRESS_HIGH     0x69U
#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_CONFIG      0x1AU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_PWR_MGMT_1  0x6BU
#define MPU6050_REG_PWR_MGMT_2  0x6CU
#define MPU6050_REG_WHO_AM_I    0x75U

typedef enum {
    BLUETOOTH_DIAGNOSTIC_RAW_HAL,
    BLUETOOTH_DIAGNOSTIC_BSP_DMA,
    BLUETOOTH_DIAGNOSTIC_ERROR
} BluetoothDiagnostic_State;

typedef enum {
    MPU_AVERAGE_IDLE,
    MPU_AVERAGE_SETTLING,
    MPU_AVERAGE_COLLECTING
} MpuAverage_State;

static UART_HandleTypeDef *diagnostic_uart;
static BSP_BluetoothSerial bluetooth_serial;
static BluetoothDiagnostic_State diagnostic_state;
static MpuAverage_State mpu_average_state;
static BSP_Key diagnostic_keys[4];
static BSP_MPU6050 diagnostic_mpu;
static BSP_MPU6050_RawData diagnostic_mpu_raw;
static BSP_Encoder diagnostic_encoder_1;
static BSP_Encoder diagnostic_encoder_2;
static BSP_Motor diagnostic_motor_1;
static BSP_Motor diagnostic_motor_2;

static uint32_t last_raw_banner_ms;
static uint32_t last_led_toggle_ms;
static uint32_t last_command_byte_ms;
static uint32_t last_mpu_sample_ms;
static uint32_t last_oled_refresh_ms;
static uint32_t last_encoder_sample_ms;
static bool oled_ready;
static bool mpu_ready;
static uint8_t tested_key_mask;
static uint8_t last_pressed_key;
static uint8_t mpu_who_am_i;
static volatile HAL_StatusTypeDef mpu_bsp_init_status;
static volatile HAL_StatusTypeDef mpu_bsp_read_status;
static volatile uint32_t mpu_bsp_sample_count;
static int16_t mpu_accel_x;
static int16_t mpu_accel_y;
static int16_t mpu_accel_z;
static int16_t mpu_gyro_x;
static int16_t mpu_gyro_y;
static int16_t mpu_gyro_z;
static volatile HAL_StatusTypeDef diagnostic_encoder_1_init_status;
static volatile HAL_StatusTypeDef diagnostic_encoder_2_init_status;
static volatile int16_t diagnostic_encoder_1_delta;
static volatile int16_t diagnostic_encoder_2_delta;
static volatile int32_t diagnostic_encoder_1_total;
static volatile int32_t diagnostic_encoder_2_total;
static volatile uint16_t diagnostic_encoder_1_raw;
static volatile uint16_t diagnostic_encoder_2_raw;
static volatile uint8_t diagnostic_encoder_1_gpio_state;
static volatile uint8_t diagnostic_encoder_2_gpio_state;
static volatile uint32_t diagnostic_encoder_1_gpio_transitions;
static volatile uint32_t diagnostic_encoder_2_gpio_transitions;
static uint8_t diagnostic_encoder_1_last_gpio_state;
static uint8_t diagnostic_encoder_2_last_gpio_state;
static volatile HAL_StatusTypeDef diagnostic_motor_1_init_status;
static volatile HAL_StatusTypeDef diagnostic_motor_2_init_status;
static volatile uint32_t diagnostic_motor_test_request;
static volatile int16_t diagnostic_motor_test_results[4];
static bool show_motor_test_result;

static uint32_t mpu_average_started_ms;
static uint32_t mpu_average_next_sample_ms;
static uint16_t mpu_average_sample_count;
static uint16_t mpu_average_test_number;
static int64_t mpu_gyro_x_sum;
static int64_t mpu_gyro_y_sum;
static int64_t mpu_gyro_z_sum;
static int16_t mpu_gyro_x_min;
static int16_t mpu_gyro_y_min;
static int16_t mpu_gyro_z_min;
static int16_t mpu_gyro_x_max;
static int16_t mpu_gyro_y_max;
static int16_t mpu_gyro_z_max;
static int32_t mpu_gyro_x_average;
static int32_t mpu_gyro_y_average;
static int32_t mpu_gyro_z_average;
static int32_t mpu_gyro_x_baseline;
static int32_t mpu_gyro_y_baseline;
static int32_t mpu_gyro_z_baseline;
static uint16_t mpu_average_baseline_test_number;
static bool mpu_average_baseline_valid;
static bool show_mpu_average_result;

static char command_buffer[COMMAND_BUFFER_SIZE];
static uint16_t command_length;
static bool discard_command;

static HAL_StatusTypeDef BluetoothDiagnostic_BspWrite(const char *text);

static int16_t BluetoothDiagnostic_CounterDifference(uint16_t current,
                                                     uint16_t start)
{
    return (int16_t)(uint16_t)(current - start);
}

static int16_t BluetoothDiagnostic_RunMotorPulse(BSP_Motor *motor,
                                                 TIM_HandleTypeDef *encoder,
                                                 int16_t command)
{
    const uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(encoder);
    const uint32_t started_ms = HAL_GetTick();

    if (BSP_Motor_SetCommand(motor, command) != HAL_OK) {
        return 0;
    }

    int16_t difference = 0;
    do {
        difference = BluetoothDiagnostic_CounterDifference(
            (uint16_t)__HAL_TIM_GET_COUNTER(encoder),
            start
        );
        int32_t magnitude = difference;
        if (magnitude < 0) {
            magnitude = -magnitude;
        }
        if (magnitude >= MOTOR_TEST_MAX_COUNTS) {
            break;
        }
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - started_ms) < MOTOR_TEST_MAX_TIME_MS);

    (void)BSP_Motor_Stop(motor);
    HAL_Delay(MOTOR_TEST_SETTLE_MS);
    return difference;
}

static bool BluetoothDiagnostic_MotorPairPassed(int16_t positive,
                                                int16_t negative)
{
    return positive != 0
        && negative != 0
        && ((int32_t)positive * (int32_t)negative) < 0;
}

static void BluetoothDiagnostic_DrawMotorTestResult(void)
{
    char line[24];
    const bool motor_1_passed = BluetoothDiagnostic_MotorPairPassed(
        diagnostic_motor_test_results[0],
        diagnostic_motor_test_results[1]
    );
    const bool motor_2_passed = BluetoothDiagnostic_MotorPairPassed(
        diagnostic_motor_test_results[2],
        diagnostic_motor_test_results[3]
    );

    OLED_NewFrame();
    OLED_PrintASCIIString(1U, 0U, "MOTOR+ENCODER TEST", &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "M1 +%d  -%d",
                   diagnostic_motor_test_results[0],
                   diagnostic_motor_test_results[1]);
    OLED_PrintASCIIString(1U, 16U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "M2 +%d  -%d",
                   diagnostic_motor_test_results[2],
                   diagnostic_motor_test_results[3]);
    OLED_PrintASCIIString(1U, 30U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "RESULT %s/%s",
                   motor_1_passed ? "PASS" : "FAIL",
                   motor_2_passed ? "PASS" : "FAIL");
    OLED_PrintASCIIString(1U, 46U, line, &afont16x8, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

static void BluetoothDiagnostic_RunMotorTest(void)
{
    diagnostic_motor_test_request = 0U;
    show_motor_test_result = true;
    (void)BSP_Motor_Stop(&diagnostic_motor_1);
    (void)BSP_Motor_Stop(&diagnostic_motor_2);

    if (oled_ready) {
        OLED_NewFrame();
        OLED_PrintASCIIString(1U, 4U, "MOTOR TEST", &afont16x8, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(1U, 28U, "ONE WHEEL AT A TIME", &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(1U, 46U, "AUTO STOP ENABLED", &afont8x6, OLED_COLOR_NORMAL);
        OLED_ShowFrame();
    }
    HAL_Delay(500U);

    if (diagnostic_motor_1_init_status == HAL_OK) {
        diagnostic_motor_test_results[0] = BluetoothDiagnostic_RunMotorPulse(
            &diagnostic_motor_1, &htim3, MOTOR_TEST_COMMAND);
        diagnostic_motor_test_results[1] = BluetoothDiagnostic_RunMotorPulse(
            &diagnostic_motor_1, &htim3, -MOTOR_TEST_COMMAND);
    }
    if (diagnostic_motor_2_init_status == HAL_OK) {
        diagnostic_motor_test_results[2] = BluetoothDiagnostic_RunMotorPulse(
            &diagnostic_motor_2, &htim4, MOTOR_TEST_COMMAND);
        diagnostic_motor_test_results[3] = BluetoothDiagnostic_RunMotorPulse(
            &diagnostic_motor_2, &htim4, -MOTOR_TEST_COMMAND);
    }

    (void)BSP_Motor_Stop(&diagnostic_motor_1);
    (void)BSP_Motor_Stop(&diagnostic_motor_2);
    if (oled_ready) {
        BluetoothDiagnostic_DrawMotorTestResult();
    }
}

/*
 * The encoder boards contain about 5.1 kOhm pull-ups on A/B.  Enabling the
 * MCU's much weaker pull-downs gives us a useful no-multimeter test:
 * a powered and connected encoder still reads high when its Hall output is
 * released, while an unpowered/open cable is pulled low by the STM32.
 */
static void BluetoothDiagnostic_EnableEncoderInputPullDowns(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;

    gpio.Pin = E1A_Pin | E1B_Pin;
    HAL_GPIO_Init(E1A_GPIO_Port, &gpio);

    gpio.Pin = E2A_Pin | E2B_Pin;
    HAL_GPIO_Init(E2A_GPIO_Port, &gpio);
}

static uint8_t BluetoothDiagnostic_ReadEncoderGpioState(
    GPIO_TypeDef *port,
    uint16_t pin_a,
    uint16_t pin_b
)
{
    uint8_t state = 0U;

    if (HAL_GPIO_ReadPin(port, pin_a) == GPIO_PIN_SET) {
        state |= 0x02U;
    }
    if (HAL_GPIO_ReadPin(port, pin_b) == GPIO_PIN_SET) {
        state |= 0x01U;
    }

    return state;
}

static void BluetoothDiagnostic_UpdateEncoderGpioTransitions(void)
{
    uint8_t encoder_1_state = BluetoothDiagnostic_ReadEncoderGpioState(
        E1A_GPIO_Port,
        E1A_Pin,
        E1B_Pin
    );
    uint8_t encoder_2_state = BluetoothDiagnostic_ReadEncoderGpioState(
        E2A_GPIO_Port,
        E2A_Pin,
        E2B_Pin
    );

    diagnostic_encoder_1_gpio_state = encoder_1_state;
    diagnostic_encoder_2_gpio_state = encoder_2_state;

    if (encoder_1_state != diagnostic_encoder_1_last_gpio_state) {
        diagnostic_encoder_1_gpio_transitions++;
        diagnostic_encoder_1_last_gpio_state = encoder_1_state;
    }
    if (encoder_2_state != diagnostic_encoder_2_last_gpio_state) {
        diagnostic_encoder_2_gpio_transitions++;
        diagnostic_encoder_2_last_gpio_state = encoder_2_state;
    }
}

static void BluetoothDiagnostic_UpdateEncoders(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_encoder_sample_ms)
        < ENCODER_SAMPLE_PERIOD_MS) {
        return;
    }

    last_encoder_sample_ms = now_ms;

    if (diagnostic_encoder_1_init_status == HAL_OK) {
        diagnostic_encoder_1_delta =
            BSP_Encoder_ReadDelta(&diagnostic_encoder_1);
        diagnostic_encoder_1_total += diagnostic_encoder_1_delta;
        diagnostic_encoder_1_raw =
            (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    }

    if (diagnostic_encoder_2_init_status == HAL_OK) {
        diagnostic_encoder_2_delta =
            BSP_Encoder_ReadDelta(&diagnostic_encoder_2);
        diagnostic_encoder_2_total += diagnostic_encoder_2_delta;
        diagnostic_encoder_2_raw =
            (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    }
}

static HAL_StatusTypeDef BluetoothDiagnostic_ReadMpuSample(void)
{
    HAL_StatusTypeDef status = MPU6050_ReadRaw(&diagnostic_mpu, &diagnostic_mpu_raw);
    mpu_bsp_read_status = status;

    if (status != HAL_OK) {
        return status;
    }

    mpu_accel_x = diagnostic_mpu_raw.Accel_X;
    mpu_accel_y = diagnostic_mpu_raw.Accel_Y;
    mpu_accel_z = diagnostic_mpu_raw.Accel_Z;
    mpu_gyro_x = diagnostic_mpu_raw.Gyro_X;
    mpu_gyro_y = diagnostic_mpu_raw.Gyro_Y;
    mpu_gyro_z = diagnostic_mpu_raw.Gyro_Z;
    mpu_bsp_sample_count++;
    return HAL_OK;
}

static bool BluetoothDiagnostic_InitMpu(void)
{
    static const uint8_t candidate_addresses[] = {
        MPU6050_ADDRESS_LOW,
        MPU6050_ADDRESS_HIGH
    };

    static const struct {
        uint8_t reg;
        uint8_t expected;
    } expected_configuration[] = {
        { MPU6050_REG_PWR_MGMT_1, 0x01U },
        { MPU6050_REG_PWR_MGMT_2, 0x00U },
        { MPU6050_REG_CONFIG, 0x03U },
        { MPU6050_REG_SMPLRT_DIV, 0x09U },
        { MPU6050_REG_GYRO_CONFIG, 0x00U },
        { MPU6050_REG_ACCEL_CONFIG, 0x00U }
    };

    mpu_bsp_init_status = HAL_ERROR;
    for (uint8_t i = 0U; i < sizeof(candidate_addresses); i++) {
        mpu_bsp_init_status = MPU6050_Init(
            &diagnostic_mpu,
            &hi2c2,
            candidate_addresses[i]
        );
        if (mpu_bsp_init_status == HAL_OK) {
            break;
        }
    }

    if (mpu_bsp_init_status != HAL_OK) {
        return false;
    }

    if (MPU6050_ReadRegister(
            &diagnostic_mpu,
            MPU6050_REG_WHO_AM_I,
            &mpu_who_am_i
        ) != HAL_OK
        || (mpu_who_am_i & 0x7EU) != 0x68U) {
        return false;
    }

    /* 回读关键寄存器，确认本次测试的配置确实由 BSP 写入。 */
    for (uint8_t i = 0U;
         i < sizeof(expected_configuration) / sizeof(expected_configuration[0]);
         i++) {
        uint8_t actual = 0xFFU;
        if (MPU6050_ReadRegister(
                &diagnostic_mpu,
                expected_configuration[i].reg,
                &actual
            ) != HAL_OK
            || actual != expected_configuration[i].expected) {
            return false;
        }
    }

    return BluetoothDiagnostic_ReadMpuSample() == HAL_OK;
}

static void BluetoothDiagnostic_DrawOledTest(void)
{
    char line[24];

    OLED_NewFrame();

    (void)snprintf(
        line,
        sizeof(line),
        "P1:%u%u P2:%u%u %s/%s",
        (diagnostic_encoder_1_gpio_state >> 1U) & 1U,
        diagnostic_encoder_1_gpio_state & 1U,
        (diagnostic_encoder_2_gpio_state >> 1U) & 1U,
        diagnostic_encoder_2_gpio_state & 1U,
        diagnostic_encoder_1_init_status == HAL_OK ? "OK" : "ER",
        diagnostic_encoder_2_init_status == HAL_OK ? "OK" : "ER"
    );
    OLED_PrintASCIIString(1U, 0U, line, &afont8x6, OLED_COLOR_NORMAL);

    (void)snprintf(
        line,
        sizeof(line),
        "E1 C%5u X%7lu",
        diagnostic_encoder_1_raw,
        (unsigned long)diagnostic_encoder_1_gpio_transitions
    );
    OLED_PrintASCIIString(1U, 10U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(
        line,
        sizeof(line),
        "E1 TOTAL%+10ld",
        (long)diagnostic_encoder_1_total
    );
    OLED_PrintASCIIString(1U, 20U, line, &afont8x6, OLED_COLOR_NORMAL);

    (void)snprintf(
        line,
        sizeof(line),
        "E2 C%5u X%7lu",
        diagnostic_encoder_2_raw,
        (unsigned long)diagnostic_encoder_2_gpio_transitions
    );
    OLED_PrintASCIIString(1U, 34U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(
        line,
        sizeof(line),
        "E2 TOTAL%+10ld",
        (long)diagnostic_encoder_2_total
    );
    OLED_PrintASCIIString(1U, 44U, line, &afont8x6, OLED_COLOR_NORMAL);

    OLED_PrintASCIIString(1U, 56U, "PULLDOWN TEST: TURN", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

static void BluetoothDiagnostic_DrawMpuAverageWaiting(void)
{
    OLED_NewFrame();
    OLED_PrintASCIIString(1U, 0U, "GYRO AVERAGE TEST", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 12U, "RELEASE K1", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 24U, "KEEP BOARD STILL", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 36U, "START IN 2 SECONDS", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 52U, "N=1000 AT 100 HZ", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

static void BluetoothDiagnostic_DrawMpuAverageCollecting(void)
{
    OLED_NewFrame();
    OLED_PrintASCIIString(1U, 2U, "GYRO AVERAGE TEST", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 18U, "SAMPLING 1000 DATA", &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 34U, "DO NOT TOUCH", &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 54U, "WAIT ABOUT 10 SEC", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

static void BluetoothDiagnostic_DrawMpuAverageResult(void)
{
    char line[32];

    OLED_NewFrame();
    (void)snprintf(
        line,
        sizeof(line),
        "T%u B%u N%u",
        (unsigned int)mpu_average_test_number,
        (unsigned int)mpu_average_baseline_test_number,
        (unsigned int)mpu_average_sample_count
    );
    OLED_PrintASCIIString(1U, 0U, line, &afont8x6, OLED_COLOR_NORMAL);

    (void)snprintf(
        line,
        sizeof(line),
        "GX%6ld D%+5ld",
        (long)mpu_gyro_x_average,
        (long)(mpu_gyro_x_average - mpu_gyro_x_baseline)
    );
    OLED_PrintASCIIString(1U, 10U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(
        line,
        sizeof(line),
        "GY%6ld D%+5ld",
        (long)mpu_gyro_y_average,
        (long)(mpu_gyro_y_average - mpu_gyro_y_baseline)
    );
    OLED_PrintASCIIString(1U, 20U, line, &afont8x6, OLED_COLOR_NORMAL);
    (void)snprintf(
        line,
        sizeof(line),
        "GZ%6ld D%+5ld",
        (long)mpu_gyro_z_average,
        (long)(mpu_gyro_z_average - mpu_gyro_z_baseline)
    );
    OLED_PrintASCIIString(1U, 30U, line, &afont8x6, OLED_COLOR_NORMAL);

    (void)snprintf(
        line,
        sizeof(line),
        "R %3ld/%3ld/%3ld",
        (long)((int32_t)mpu_gyro_x_max - mpu_gyro_x_min),
        (long)((int32_t)mpu_gyro_y_max - mpu_gyro_y_min),
        (long)((int32_t)mpu_gyro_z_max - mpu_gyro_z_min)
    );
    OLED_PrintASCIIString(1U, 42U, line, &afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1U, 54U, "K1 GO K2 LIVE K3 BASE", &afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

static void BluetoothDiagnostic_StartMpuAverage(uint32_t now_ms)
{
    if (!mpu_ready || !oled_ready) {
        return;
    }

    mpu_average_test_number++;
    mpu_average_started_ms = now_ms;
    mpu_average_state = MPU_AVERAGE_SETTLING;
    show_mpu_average_result = true;
    BSP_LED_ON();
    BluetoothDiagnostic_DrawMpuAverageWaiting();
}

static bool BluetoothDiagnostic_UpdateMpuAverage(uint32_t now_ms)
{
    if (mpu_average_state == MPU_AVERAGE_IDLE) {
        return false;
    }

    if (mpu_average_state == MPU_AVERAGE_SETTLING) {
        if ((uint32_t)(now_ms - mpu_average_started_ms) < MPU_AVERAGE_SETTLE_MS) {
            return true;
        }

        mpu_average_sample_count = 0U;
        mpu_gyro_x_sum = 0;
        mpu_gyro_y_sum = 0;
        mpu_gyro_z_sum = 0;
        mpu_average_state = MPU_AVERAGE_COLLECTING;
        BluetoothDiagnostic_DrawMpuAverageCollecting();
        mpu_average_next_sample_ms = HAL_GetTick();
        return true;
    }

    if ((int32_t)(now_ms - mpu_average_next_sample_ms) < 0) {
        return true;
    }
    mpu_average_next_sample_ms += MPU_AVERAGE_SAMPLE_MS;

    if (BluetoothDiagnostic_ReadMpuSample() != HAL_OK) {
        mpu_ready = false;
        mpu_average_state = MPU_AVERAGE_IDLE;
        show_mpu_average_result = false;
        BSP_LED_OFF();
        BluetoothDiagnostic_DrawOledTest();
        return false;
    }

    if (mpu_average_sample_count == 0U) {
        mpu_gyro_x_min = mpu_gyro_x_max = mpu_gyro_x;
        mpu_gyro_y_min = mpu_gyro_y_max = mpu_gyro_y;
        mpu_gyro_z_min = mpu_gyro_z_max = mpu_gyro_z;
    } else {
        if (mpu_gyro_x < mpu_gyro_x_min) mpu_gyro_x_min = mpu_gyro_x;
        if (mpu_gyro_x > mpu_gyro_x_max) mpu_gyro_x_max = mpu_gyro_x;
        if (mpu_gyro_y < mpu_gyro_y_min) mpu_gyro_y_min = mpu_gyro_y;
        if (mpu_gyro_y > mpu_gyro_y_max) mpu_gyro_y_max = mpu_gyro_y;
        if (mpu_gyro_z < mpu_gyro_z_min) mpu_gyro_z_min = mpu_gyro_z;
        if (mpu_gyro_z > mpu_gyro_z_max) mpu_gyro_z_max = mpu_gyro_z;
    }

    mpu_gyro_x_sum += mpu_gyro_x;
    mpu_gyro_y_sum += mpu_gyro_y;
    mpu_gyro_z_sum += mpu_gyro_z;
    mpu_average_sample_count++;

    if (mpu_average_sample_count < MPU_AVERAGE_SAMPLE_COUNT) {
        return true;
    }

    mpu_gyro_x_average = (int32_t)(mpu_gyro_x_sum / MPU_AVERAGE_SAMPLE_COUNT);
    mpu_gyro_y_average = (int32_t)(mpu_gyro_y_sum / MPU_AVERAGE_SAMPLE_COUNT);
    mpu_gyro_z_average = (int32_t)(mpu_gyro_z_sum / MPU_AVERAGE_SAMPLE_COUNT);

    if (!mpu_average_baseline_valid) {
        mpu_gyro_x_baseline = mpu_gyro_x_average;
        mpu_gyro_y_baseline = mpu_gyro_y_average;
        mpu_gyro_z_baseline = mpu_gyro_z_average;
        mpu_average_baseline_test_number = mpu_average_test_number;
        mpu_average_baseline_valid = true;
    }

    mpu_average_state = MPU_AVERAGE_IDLE;
    BSP_LED_OFF();
    BluetoothDiagnostic_DrawMpuAverageResult();
    return false;
}

static bool BluetoothDiagnostic_InitOled(void)
{
    /* SSD1306 上电后需要短暂稳定；先用 HAL ACK 区分总线硬件和显示驱动问题。 */
    HAL_Delay(30U);
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDRESS, 3U, 100U) != HAL_OK) {
        return false;
    }

    OLED_Init();
    return true;
}

static void BluetoothDiagnostic_UpdateKeys(uint32_t now_ms)
{
    static const char *const key_reports[4] = {
        "[KEY] K1 PRESSED\r\n",
        "[KEY] K2 PRESSED\r\n",
        "[KEY] K3 PRESSED\r\n",
        "[KEY] K4 PRESSED\r\n"
    };

    for (uint8_t i = 0U; i < 4U; i++) {
        BSP_Key_Update(&diagnostic_keys[i], now_ms);
        if (BSP_Key_TakeEvent(&diagnostic_keys[i]) != BSP_KEY_EVENT_PRESSED) {
            continue;
        }

        tested_key_mask |= (uint8_t)(1U << i);
        last_pressed_key = (uint8_t)(i + 1U);
        BSP_LED_Toggle();

        if (i == 0U && mpu_average_state == MPU_AVERAGE_IDLE) {
            BluetoothDiagnostic_StartMpuAverage(now_ms);
        } else if (i == 1U
                   && mpu_average_state == MPU_AVERAGE_IDLE
                   && oled_ready) {
            show_mpu_average_result = false;
            BluetoothDiagnostic_DrawOledTest();
            last_oled_refresh_ms = now_ms;
        } else if (i == 2U
                   && mpu_average_state == MPU_AVERAGE_IDLE
                   && show_mpu_average_result
                   && oled_ready) {
            mpu_gyro_x_baseline = mpu_gyro_x_average;
            mpu_gyro_y_baseline = mpu_gyro_y_average;
            mpu_gyro_z_baseline = mpu_gyro_z_average;
            mpu_average_baseline_test_number = mpu_average_test_number;
            mpu_average_baseline_valid = true;
            BluetoothDiagnostic_DrawMpuAverageResult();
        }

        if (diagnostic_state == BLUETOOTH_DIAGNOSTIC_BSP_DMA
            && !BSP_BluetoothSerial_IsTxBusy(&bluetooth_serial)) {
            (void)BluetoothDiagnostic_BspWrite(key_reports[i]);
        }
    }
}

static void BluetoothDiagnostic_RawWrite(const char *text)
{
    (void)HAL_UART_Transmit(
        diagnostic_uart,
        (const uint8_t *)text,
        (uint16_t)strlen(text),
        200U
    );
}

static HAL_StatusTypeDef BluetoothDiagnostic_BspWrite(const char *text)
{
    return BSP_BluetoothSerial_Write(
        &bluetooth_serial,
        (const uint8_t *)text,
        (uint16_t)strlen(text)
    );
}

static void BluetoothDiagnostic_EnterBspStage(void)
{
    BluetoothDiagnostic_RawWrite(
        "\r\n[RAW HAL] RX OK. Switching to BSP DMA...\r\n"
    );

    HAL_StatusTypeDef status = BSP_BluetoothSerial_Init(
        &bluetooth_serial,
        diagnostic_uart
    );

    if (status != HAL_OK) {
        diagnostic_state = BLUETOOTH_DIAGNOSTIC_ERROR;
        BluetoothDiagnostic_RawWrite("[ERROR] BSP DMA init failed.\r\n");
        return;
    }

    diagnostic_state = BLUETOOTH_DIAGNOSTIC_BSP_DMA;
    command_length = 0U;
    discard_command = false;
    last_command_byte_ms = HAL_GetTick();
    BSP_LED_OFF();

    (void)BluetoothDiagnostic_BspWrite(
        "[BSP DMA] READY\r\n"
        "Commands: PING | STATUS | MPU | LED ON/OFF/TOGGLE | OLED TEST\r\n"
    );
}

static void BluetoothDiagnostic_ProcessCommand(void)
{
    command_buffer[command_length] = '\0';

    if (strcmp(command_buffer, "PING") == 0) {
        BSP_LED_Toggle();
        (void)BluetoothDiagnostic_BspWrite("PONG - BSP RX/TX DMA OK\r\n");
    } else if (strcmp(command_buffer, "STATUS") == 0) {
        char response[96];
        (void)snprintf(
            response,
            sizeof(response),
            "STATUS - DMA OK; OLED %s; MPU %s WHO=%02X\r\n",
            oled_ready ? "OK" : "FAIL",
            mpu_ready ? "OK" : "FAIL",
            mpu_who_am_i
        );
        (void)BluetoothDiagnostic_BspWrite(response);
    } else if (strcmp(command_buffer, "MPU") == 0) {
        if (mpu_ready) {
            char response[120];
            (void)snprintf(
                response,
                sizeof(response),
                "MPU AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\r\n",
                mpu_accel_x,
                mpu_accel_y,
                mpu_accel_z,
                mpu_gyro_x,
                mpu_gyro_y,
                mpu_gyro_z
            );
            (void)BluetoothDiagnostic_BspWrite(response);
        } else {
            (void)BluetoothDiagnostic_BspWrite("MPU6050 FAIL - NO VALID WHO_AM_I/DATA\r\n");
        }
    } else if (strcmp(command_buffer, "LED ON") == 0) {
        BSP_LED_ON();
        (void)BluetoothDiagnostic_BspWrite("LED ON OK\r\n");
    } else if (strcmp(command_buffer, "LED OFF") == 0) {
        BSP_LED_OFF();
        (void)BluetoothDiagnostic_BspWrite("LED OFF OK\r\n");
    } else if (strcmp(command_buffer, "LED TOGGLE") == 0) {
        BSP_LED_Toggle();
        (void)BluetoothDiagnostic_BspWrite("LED TOGGLE OK\r\n");
    } else if (strcmp(command_buffer, "OLED TEST") == 0) {
        if (oled_ready) {
            BluetoothDiagnostic_DrawOledTest();
            (void)BluetoothDiagnostic_BspWrite("OLED TEST PATTERN SENT\r\n");
        } else {
            (void)BluetoothDiagnostic_BspWrite("OLED NOT FOUND AT I2C 0x3C\r\n");
        }
    } else {
        (void)BluetoothDiagnostic_BspWrite("UNKNOWN COMMAND\r\n");
    }

    command_length = 0U;
}

static void BluetoothDiagnostic_UpdateMpu(uint32_t now_ms)
{
    if (!mpu_ready
        || (uint32_t)(now_ms - last_mpu_sample_ms) < MPU_SAMPLE_PERIOD_MS) {
        return;
    }

    last_mpu_sample_ms = now_ms;
    if (BluetoothDiagnostic_ReadMpuSample() != HAL_OK) {
        mpu_ready = false;
    }
}

static void BluetoothDiagnostic_UpdateOled(uint32_t now_ms)
{
    if (!oled_ready
        || show_mpu_average_result
        || (uint32_t)(now_ms - last_oled_refresh_ms) < OLED_REFRESH_PERIOD_MS) {
        return;
    }

    last_oled_refresh_ms = now_ms;
    BluetoothDiagnostic_DrawOledTest();
}

static void BluetoothDiagnostic_UpdateRawHal(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_raw_banner_ms) >= RAW_BANNER_PERIOD_MS) {
        last_raw_banner_ms = now_ms;
        BluetoothDiagnostic_RawWrite(oled_ready
            ? "[RAW HAL] UART ready; OLED ACK + test pattern sent. Send R for BSP DMA.\r\n"
            : "[RAW HAL] UART ready; OLED not found at 0x3C. Send R for BSP DMA.\r\n");
    }

    if ((uint32_t)(now_ms - last_led_toggle_ms) >= RAW_LED_PERIOD_MS) {
        last_led_toggle_ms = now_ms;
        BSP_LED_Toggle();
    }

    uint8_t received_byte;
    if (HAL_UART_Receive(diagnostic_uart, &received_byte, 1U, 0U) == HAL_OK) {
        if (received_byte == 'R' || received_byte == 'r') {
            BluetoothDiagnostic_EnterBspStage();
        } else if (received_byte != '\r' && received_byte != '\n') {
            BluetoothDiagnostic_RawWrite(
                "[RAW HAL] RX byte OK, but send R to continue.\r\n"
            );
        }
    }
}

static void BluetoothDiagnostic_UpdateBspDma(void)
{
    uint32_t now_ms = HAL_GetTick();

    /* 等上一次 DMA 发送完成再处理下一条命令，避免测试响应互相覆盖。 */
    if (BSP_BluetoothSerial_IsTxBusy(&bluetooth_serial)) {
        return;
    }

    if (BSP_BluetoothSerial_TakeOverflow(&bluetooth_serial)) {
        (void)BluetoothDiagnostic_BspWrite("ERROR - RX RING BUFFER OVERFLOW\r\n");
        return;
    }

    uint8_t received_byte;
    while (BSP_BluetoothSerial_ReadByte(&bluetooth_serial, &received_byte)) {
        last_command_byte_ms = now_ms;

        if (received_byte == '\r' || received_byte == '\n') {
            if (discard_command) {
                discard_command = false;
                command_length = 0U;
                (void)BluetoothDiagnostic_BspWrite("ERROR - COMMAND TOO LONG\r\n");
                return;
            }

            if (command_length > 0U) {
                BluetoothDiagnostic_ProcessCommand();
                return;
            }
            continue;
        }

        if (discard_command) {
            continue;
        }

        if (command_length < COMMAND_BUFFER_SIZE - 1U) {
            command_buffer[command_length] = (char)received_byte;
            command_length++;
        } else {
            discard_command = true;
        }
    }

    /* 部分手机蓝牙终端不发送 CR/LF；将一次短暂停顿视为一条命令的结束。 */
    if ((command_length > 0U || discard_command)
        && (uint32_t)(now_ms - last_command_byte_ms) >= COMMAND_IDLE_TIMEOUT_MS) {
        if (discard_command) {
            discard_command = false;
            command_length = 0U;
            (void)BluetoothDiagnostic_BspWrite("ERROR - COMMAND TOO LONG\r\n");
        } else {
            BluetoothDiagnostic_ProcessCommand();
        }
    }
}

static void BluetoothDiagnostic_UpdateError(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_led_toggle_ms) >= ERROR_LED_PERIOD_MS) {
        last_led_toggle_ms = now_ms;
        BSP_LED_Toggle();
    }
}

void BluetoothDiagnostic_Init(UART_HandleTypeDef *huart)
{
    diagnostic_uart = huart;
    diagnostic_state = huart == NULL
        ? BLUETOOTH_DIAGNOSTIC_ERROR
        : BLUETOOTH_DIAGNOSTIC_RAW_HAL;

    uint32_t now_ms = HAL_GetTick();
    last_raw_banner_ms = now_ms - RAW_BANNER_PERIOD_MS;
    last_led_toggle_ms = now_ms;
    last_command_byte_ms = now_ms;
    command_length = 0U;
    discard_command = false;
    tested_key_mask = 0U;
    last_pressed_key = 0U;
    mpu_who_am_i = 0U;
    mpu_bsp_init_status = HAL_ERROR;
    mpu_bsp_read_status = HAL_ERROR;
    mpu_bsp_sample_count = 0U;
    mpu_average_state = MPU_AVERAGE_IDLE;
    mpu_average_test_number = 0U;
    mpu_average_baseline_test_number = 0U;
    mpu_average_baseline_valid = false;
    show_mpu_average_result = false;
    show_motor_test_result = false;
    diagnostic_motor_test_request = 0U;
    memset((void *)diagnostic_motor_test_results, 0,
           sizeof(diagnostic_motor_test_results));
    diagnostic_encoder_1_delta = 0;
    diagnostic_encoder_2_delta = 0;
    diagnostic_encoder_1_total = 0;
    diagnostic_encoder_2_total = 0;
    diagnostic_encoder_1_raw = 0U;
    diagnostic_encoder_2_raw = 0U;
    diagnostic_encoder_1_gpio_transitions = 0U;
    diagnostic_encoder_2_gpio_transitions = 0U;
    diagnostic_encoder_1_last_gpio_state =
        BluetoothDiagnostic_ReadEncoderGpioState(
            E1A_GPIO_Port, E1A_Pin, E1B_Pin);
    diagnostic_encoder_2_last_gpio_state =
        BluetoothDiagnostic_ReadEncoderGpioState(
            E2A_GPIO_Port, E2A_Pin, E2B_Pin);
    diagnostic_encoder_1_gpio_state = diagnostic_encoder_1_last_gpio_state;
    diagnostic_encoder_2_gpio_state = diagnostic_encoder_2_last_gpio_state;
    BSP_LED_OFF();
    oled_ready = BluetoothDiagnostic_InitOled();

    BSP_Key_Init(&diagnostic_keys[0], KEY_1_GPIO_Port, KEY_1_Pin,
                 GPIO_PIN_RESET, 20U, now_ms);
    BSP_Key_Init(&diagnostic_keys[1], KEY_2_GPIO_Port, KEY_2_Pin,
                 GPIO_PIN_RESET, 20U, now_ms);
    BSP_Key_Init(&diagnostic_keys[2], KEY_3_GPIO_Port, KEY_3_Pin,
                 GPIO_PIN_RESET, 20U, now_ms);
    BSP_Key_Init(&diagnostic_keys[3], KEY_4_GPIO_Port, KEY_4_Pin,
                 GPIO_PIN_RESET, 20U, now_ms);

    diagnostic_encoder_1_init_status =
        BSP_Encoder_Init(&diagnostic_encoder_1, &htim3, false);
    diagnostic_encoder_2_init_status =
        BSP_Encoder_Init(&diagnostic_encoder_2, &htim4, false);

    diagnostic_motor_1_init_status = BSP_Motor_Init(
        &diagnostic_motor_1,
        &htim2,
        TIM_CHANNEL_1,
        AIN1_GPIO_Port,
        AIN1_Pin,
        AIN2_GPIO_Port,
        AIN2_Pin,
        false
    );
    diagnostic_motor_2_init_status = BSP_Motor_Init(
        &diagnostic_motor_2,
        &htim2,
        TIM_CHANNEL_2,
        BIN1_GPIO_Port,
        BIN1_Pin,
        BIN2_GPIO_Port,
        BIN2_Pin,
        false
    );

    BluetoothDiagnostic_EnableEncoderInputPullDowns();
    diagnostic_encoder_1_last_gpio_state =
        BluetoothDiagnostic_ReadEncoderGpioState(
            E1A_GPIO_Port, E1A_Pin, E1B_Pin);
    diagnostic_encoder_2_last_gpio_state =
        BluetoothDiagnostic_ReadEncoderGpioState(
            E2A_GPIO_Port, E2A_Pin, E2B_Pin);
    diagnostic_encoder_1_gpio_state = diagnostic_encoder_1_last_gpio_state;
    diagnostic_encoder_2_gpio_state = diagnostic_encoder_2_last_gpio_state;

    mpu_ready = BluetoothDiagnostic_InitMpu();
    now_ms = HAL_GetTick();
    last_mpu_sample_ms = now_ms;
    last_oled_refresh_ms = now_ms;
    last_encoder_sample_ms = now_ms;
    if (oled_ready) {
        BluetoothDiagnostic_DrawOledTest();
    }
}

void BluetoothDiagnostic_Update(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (diagnostic_motor_test_request == MOTOR_TEST_MAGIC) {
        BluetoothDiagnostic_RunMotorTest();
        return;
    }

    BluetoothDiagnostic_UpdateEncoderGpioTransitions();
    BluetoothDiagnostic_UpdateKeys(now_ms);
    BluetoothDiagnostic_UpdateEncoders(now_ms);

    if (BluetoothDiagnostic_UpdateMpuAverage(now_ms)) {
        return;
    }

    BluetoothDiagnostic_UpdateMpu(now_ms);
    if (!show_motor_test_result) {
        BluetoothDiagnostic_UpdateOled(now_ms);
    }

    switch (diagnostic_state) {
        case BLUETOOTH_DIAGNOSTIC_RAW_HAL:
            BluetoothDiagnostic_UpdateRawHal(now_ms);
            break;

        case BLUETOOTH_DIAGNOSTIC_BSP_DMA:
            BluetoothDiagnostic_UpdateBspDma();
            break;

        case BLUETOOTH_DIAGNOSTIC_ERROR:
        default:
            BluetoothDiagnostic_UpdateError(now_ms);
            break;
    }
}

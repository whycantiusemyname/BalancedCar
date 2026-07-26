/** @file BluetoothProtocol.c @brief 教程手机上位机的方括号调参协议。 */

#include "BluetoothProtocol.h"

#include <stddef.h>
#include <string.h>

#include "BalanceCarConfig.h"

typedef struct
{
    const char *name;
    BalanceCar_PIDParameter parameter;
} PIDSlider;

static const PIDSlider pid_sliders[] = {
    {"AngleKp", BALANCE_CAR_PID_ANGLE_KP},
    {"AngleKi", BALANCE_CAR_PID_ANGLE_KI},
    {"AngleKd", BALANCE_CAR_PID_ANGLE_KD},
    {"SpeedKp", BALANCE_CAR_PID_SPEED_KP},
    {"SpeedKi", BALANCE_CAR_PID_SPEED_KI},
    {"SpeedKd", BALANCE_CAR_PID_SPEED_KD},
    {"TurnKp", BALANCE_CAR_PID_TURN_KP},
    {"TurnKi", BALANCE_CAR_PID_TURN_KI},
    {"TurnKd", BALANCE_CAR_PID_TURN_KD},
};

#define TELEMETRY_MAGIC_0              0xA5U
#define TELEMETRY_MAGIC_1              0x5AU
#define TELEMETRY_VERSION              1U
#define TELEMETRY_SAMPLE_TYPE          1U
#define TELEMETRY_PARAMETERS_TYPE      2U
#define TELEMETRY_HEADER_SIZE          14U
#define TELEMETRY_SAMPLE_FIELD_COUNT   17U
#define TELEMETRY_PARAMETER_COUNT      22U
#define TELEMETRY_SAMPLE_FRAME_SIZE \
    (TELEMETRY_HEADER_SIZE + TELEMETRY_SAMPLE_FIELD_COUNT * 2U + 2U)
#define TELEMETRY_PARAMETER_FRAME_SIZE \
    (TELEMETRY_HEADER_SIZE + TELEMETRY_PARAMETER_COUNT * 4U + 2U)

/** 解析滑杆使用的普通十进制，不引入atof/sscanf和科学计数法。 */
static bool ParseDecimal(const char *text, float *value)
{
    if (text == NULL || value == NULL || *text == '\0')
    {
        return false;
    }

    bool negative = false;
    if (*text == '+' || *text == '-')
    {
        negative = *text++ == '-';
    }

    float number = 0.0F;
    float decimal_place = 0.1F;
    bool decimal = false;
    bool has_digit = false;

    while (*text != '\0')
    {
        if (*text >= '0' && *text <= '9')
        {
            has_digit = true;
            if (decimal)
            {
                number += (float)(*text - '0') * decimal_place;
                decimal_place *= 0.1F;
            }
            else
            {
                number = number * 10.0F + (float)(*text - '0');
            }
            if (number > 100000.0F)
            {
                return false;
            }
        }
        else if (*text == '.' && !decimal)
        {
            decimal = true;
        }
        else
        {
            return false;
        }
        text++;
    }

    if (!has_digit)
    {
        return false;
    }
    *value = negative ? -number : number;
    return true;
}

/** 原地切开下一个逗号字段；cursor为NULL表示已经取到最后一项。 */
static char *NextField(char **cursor)
{
    if (cursor == NULL || *cursor == NULL || **cursor == '\0')
    {
        return NULL;
    }

    char *field = *cursor;
    while (**cursor != '\0' && **cursor != ',')
    {
        (*cursor)++;
    }
    if (**cursor == ',')
    {
        **cursor = '\0';
        (*cursor)++;
    }
    else
    {
        *cursor = NULL;
    }
    return field;
}

static float ScaleJoystickAxis(float input, float output_max)
{
    if (input > 100.0F)
    {
        input = 100.0F;
    }
    else if (input < -100.0F)
    {
        input = -100.0F;
    }

    if (input >= -BALANCE_CAR_JOYSTICK_DEADBAND &&
        input <= BALANCE_CAR_JOYSTICK_DEADBAND)
    {
        return 0.0F;
    }
    return input * output_max / 100.0F;
}

/** 一次解析完整帧，不为每种消息再建立一层对象或解析器。 */
static bool ParseFrame(char *frame, BalanceCar_Command *command)
{
    char *cursor = frame;
    char *tag = NextField(&cursor);
    if (tag == NULL)
    {
        return false;
    }

    if (strcmp(tag, "slider") == 0)
    {
        char *name = NextField(&cursor);
        char *text = NextField(&cursor);
        float value;
        if (name == NULL || text == NULL || NextField(&cursor) != NULL ||
            !ParseDecimal(text, &value))
        {
            return false;
        }

        for (uint32_t index = 0U;
             index < sizeof(pid_sliders) / sizeof(pid_sliders[0]);
             index++)
        {
            if (strcmp(name, pid_sliders[index].name) == 0)
            {
                command->type = BALANCE_CAR_COMMAND_SET_PID_GAIN;
                command->data.pid.parameter = pid_sliders[index].parameter;
                command->data.pid.value = value;
                return true;
            }
        }

        if (strcmp(name, "Offset") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_OFFSET;
        }
        else if (strcmp(name, "OffsetBand") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_BAND;
        }
        else if (strcmp(name, "PosKp") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_SET_POSITION_HOLD_KP;
        }
        else if (strcmp(name, "TurnFF") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_SET_TURN_FEEDFORWARD;
        }
        else if (strcmp(name, "BalanceTrim") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_SET_BALANCE_TRIM;
        }
        else
        {
            return false;
        }
        command->data.scalar = value;
        return true;
    }

    if (strcmp(tag, "key") == 0)
    {
        char *name = NextField(&cursor);
        char *action = NextField(&cursor);
        if (name == NULL || action == NULL || NextField(&cursor) != NULL ||
            strcmp(action, "down") != 0)
        {
            return false;
        }

        if (strcmp(name, "Start") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_START;
        }
        else if (strcmp(name, "Stop") == 0)
        {
            command->type = BALANCE_CAR_COMMAND_STOP;
        }
        else if (strcmp(name, "Recover") == 0)
        {
            /* 恢复与启动执行同一动作，只保留一个内部命令。 */
            command->type = BALANCE_CAR_COMMAND_START;
        }
        else
        {
            return false;
        }
        return true;
    }

    if (strcmp(tag, "motor") == 0)
    {
        char *selection = NextField(&cursor);
        char *text = NextField(&cursor);
        float value;
        if (selection == NULL || text == NULL ||
            NextField(&cursor) != NULL || !ParseDecimal(text, &value) ||
            value < -(float)BALANCE_CAR_MOTOR_TEST_MAX_COMMAND ||
            value > (float)BALANCE_CAR_MOTOR_TEST_MAX_COMMAND)
        {
            return false;
        }

        if (strcmp(selection, "L") == 0)
        {
            command->data.motor_test.selection = BALANCE_CAR_MOTOR_LEFT;
        }
        else if (strcmp(selection, "R") == 0)
        {
            command->data.motor_test.selection = BALANCE_CAR_MOTOR_RIGHT;
        }
        else if (strcmp(selection, "B") == 0)
        {
            command->data.motor_test.selection = BALANCE_CAR_MOTOR_BOTH;
        }
        else
        {
            return false;
        }

        command->type = BALANCE_CAR_COMMAND_TEST_MOTOR;
        command->data.motor_test.command = (int16_t)value;
        return true;
    }

    if (strcmp(tag, "joystick") == 0)
    {
        char *lh_text = NextField(&cursor);
        char *lv_text = NextField(&cursor);
        char *rh_text = NextField(&cursor);
        char *rv_text = NextField(&cursor);
        float lh;
        float lv;
        float rh;
        float rv;
        if (lh_text == NULL || lv_text == NULL || rh_text == NULL ||
            rv_text == NULL || NextField(&cursor) != NULL ||
            !ParseDecimal(lh_text, &lh) || !ParseDecimal(lv_text, &lv) ||
            !ParseDecimal(rh_text, &rh) || !ParseDecimal(rv_text, &rv))
        {
            return false;
        }

        /* LH/RV保留给教程协议兼容；当前只使用LV前后、RH左右。 */
        (void)lh;
        (void)rv;
        command->type = BALANCE_CAR_COMMAND_SET_MOTION_TARGET;
        command->data.motion.forward_speed_counts_per_s =
            ScaleJoystickAxis(
                lv,
                BALANCE_CAR_JOYSTICK_MAX_FORWARD_SPEED);
        /* 内部约定左转为正；手机右推通常给出正RH，因此统一反号。 */
        command->data.motion.yaw_rate_dps =
            -ScaleJoystickAxis(rh, BALANCE_CAR_JOYSTICK_MAX_YAW_RATE_DPS);
        return true;
    }

    return false;
}

/** 遥测模式属于链路控制，不上升为整车命令。 */
static bool HandleTelemetryMode(BluetoothProtocol *protocol,
                                const char *frame)
{
    if (strcmp(frame, "telemetry,binary") == 0)
    {
        protocol->binary_telemetry_enabled = true;
        protocol->parameters_pending = true;
        return true;
    }
    if (strcmp(frame, "telemetry,text") == 0)
    {
        protocol->binary_telemetry_enabled = false;
        protocol->parameters_pending = false;
        return true;
    }
    return false;
}

void BluetoothProtocol_Init(BluetoothProtocol *protocol,
                            BSP_BluetoothSerial *serial)
{
    if (protocol != NULL)
    {
        memset(protocol, 0, sizeof(*protocol));
        protocol->serial = serial;
    }
}

bool BluetoothProtocol_Poll(BluetoothProtocol *protocol,
                            BalanceCar_Command *command)
{
    if (protocol == NULL || protocol->serial == NULL || command == NULL)
    {
        return false;
    }

    memset(command, 0, sizeof(*command));
    if (BSP_BluetoothSerial_TakeOverflow(protocol->serial))
    {
        protocol->frame_length = 0U;
        protocol->receiving_frame = false;
        protocol->frame_overflow = false;
        return false;
    }

    uint8_t byte;
    while (BSP_BluetoothSerial_ReadByte(protocol->serial, &byte))
    {
        if (byte == '[')
        {
            protocol->frame_length = 0U;
            protocol->receiving_frame = true;
            protocol->frame_overflow = false;
            continue;
        }
        if (!protocol->receiving_frame)
        {
            continue;
        }
        if (byte == ']')
        {
            protocol->receiving_frame = false;
            if (protocol->frame_overflow || protocol->frame_length == 0U)
            {
                protocol->frame_length = 0U;
                protocol->frame_overflow = false;
                continue;
            }

            protocol->frame_buffer[protocol->frame_length] = '\0';
            protocol->frame_length = 0U;
            if (HandleTelemetryMode(protocol, protocol->frame_buffer))
            {
                continue;
            }
            if (ParseFrame(protocol->frame_buffer, command))
            {
                if (command->type == BALANCE_CAR_COMMAND_SET_PID_GAIN ||
                    command->type == BALANCE_CAR_COMMAND_SET_BALANCE_TRIM ||
                    command->type ==
                        BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_OFFSET ||
                    command->type ==
                        BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_BAND ||
                    command->type ==
                        BALANCE_CAR_COMMAND_SET_POSITION_HOLD_KP ||
                    command->type ==
                        BALANCE_CAR_COMMAND_SET_TURN_FEEDFORWARD)
                {
                    protocol->parameters_pending = true;
                }
                return true;
            }
            continue;
        }
        if (protocol->frame_overflow)
        {
            continue;
        }
        if (protocol->frame_length >=
            BLUETOOTH_PROTOCOL_FRAME_BUFFER_SIZE - 1U)
        {
            protocol->frame_overflow = true;
            continue;
        }
        protocol->frame_buffer[protocol->frame_length++] = (char)byte;
    }
    return false;
}

static void AppendUnsigned(char *frame, uint16_t *length, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;
    do
    {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (count > 0U)
    {
        frame[(*length)++] = digits[--count];
    }
}

static void AppendFixed2(char *frame, uint16_t *length, float value)
{
    if (value != value)
    {
        value = 0.0F;
    }
    if (value > 99999.99F)
    {
        value = 99999.99F;
    }
    else if (value < -99999.99F)
    {
        value = -99999.99F;
    }
    if (value < 0.0F)
    {
        frame[(*length)++] = '-';
        value = -value;
    }

    const uint32_t scaled = (uint32_t)(value * 100.0F + 0.5F);
    AppendUnsigned(frame, length, scaled / 100U);
    frame[(*length)++] = '.';
    frame[(*length)++] = (char)('0' + scaled / 10U % 10U);
    frame[(*length)++] = (char)('0' + scaled % 10U);
}

static uint16_t TelemetryCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t index = 0U; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static void PutU16(uint8_t *frame, uint16_t *offset, uint16_t value)
{
    frame[(*offset)++] = (uint8_t)value;
    frame[(*offset)++] = (uint8_t)(value >> 8U);
}

static void PutU32(uint8_t *frame, uint16_t *offset, uint32_t value)
{
    frame[(*offset)++] = (uint8_t)value;
    frame[(*offset)++] = (uint8_t)(value >> 8U);
    frame[(*offset)++] = (uint8_t)(value >> 16U);
    frame[(*offset)++] = (uint8_t)(value >> 24U);
}

static void PutFloat(uint8_t *frame, uint16_t *offset, float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    PutU32(frame, offset, bits);
}

static int16_t EncodeFixed(float value, float scale)
{
    value *= scale;
    if (value > 32767.0F)
    {
        return 32767;
    }
    if (value < -32768.0F)
    {
        return -32768;
    }
    return (int16_t)(value >= 0.0F ? value + 0.5F : value - 0.5F);
}

static void BeginBinaryFrame(uint8_t *frame,
                             uint8_t type,
                             uint8_t length,
                             uint8_t state,
                             uint8_t flags,
                             uint16_t sequence,
                             uint32_t timestamp_ms,
                             uint16_t *offset)
{
    frame[0] = TELEMETRY_MAGIC_0;
    frame[1] = TELEMETRY_MAGIC_1;
    frame[2] = type;
    frame[3] = TELEMETRY_VERSION;
    frame[4] = length;
    frame[5] = state;
    frame[6] = flags;
    frame[7] = 0U;
    *offset = 8U;
    PutU16(frame, offset, sequence);
    PutU32(frame, offset, timestamp_ms);
}

static HAL_StatusTypeDef SendBinarySample(
    BluetoothProtocol *protocol,
    const BalanceCar_TelemetrySample *sample)
{
    _Static_assert(BSP_BLUETOOTH_TX_BUFFER_SIZE >=
                       TELEMETRY_SAMPLE_FRAME_SIZE,
                   "Bluetooth TX buffer is too small for binary telemetry");
    uint8_t frame[TELEMETRY_SAMPLE_FRAME_SIZE];
    uint16_t offset;
    BeginBinaryFrame(frame,
                     TELEMETRY_SAMPLE_TYPE,
                     (uint8_t)sizeof(frame),
                     sample->state,
                     sample->flags,
                     protocol->telemetry_sequence++,
                     sample->timestamp_ms,
                     &offset);

    const int16_t fields[TELEMETRY_SAMPLE_FIELD_COUNT] = {
        EncodeFixed(sample->pitch_deg, 100.0F),
        EncodeFixed(sample->accel_pitch_deg, 100.0F),
        EncodeFixed(sample->pitch_rate_dps, 100.0F),
        EncodeFixed(sample->yaw_rate_dps, 100.0F),
        EncodeFixed(sample->yaw_bias_dps, 100.0F),
        EncodeFixed(sample->left_speed_counts_per_s, 1.0F),
        EncodeFixed(sample->right_speed_counts_per_s, 1.0F),
        EncodeFixed(sample->forward_speed_counts_per_s, 1.0F),
        EncodeFixed(sample->turn_speed_counts_per_s, 1.0F),
        EncodeFixed(sample->target_speed_counts_per_s, 1.0F),
        EncodeFixed(sample->target_pitch_deg, 100.0F),
        EncodeFixed(sample->target_yaw_rate_dps, 100.0F),
        EncodeFixed(sample->speed_integral_pitch_deg, 1000.0F),
        EncodeFixed(sample->balance_output, 1.0F),
        EncodeFixed(sample->turn_output, 1.0F),
        EncodeFixed(sample->left_motor_command, 1.0F),
        EncodeFixed(sample->right_motor_command, 1.0F),
    };
    for (uint32_t index = 0U; index < TELEMETRY_SAMPLE_FIELD_COUNT; index++)
    {
        PutU16(frame, &offset, (uint16_t)fields[index]);
    }

    const uint16_t crc = TelemetryCrc16(frame, offset);
    PutU16(frame, &offset, crc);
    return BSP_BluetoothSerial_Write(protocol->serial, frame, offset);
}

static HAL_StatusTypeDef SendBinaryParameters(
    BluetoothProtocol *protocol,
    const BalanceCar_TelemetrySample *sample,
    const BalanceCar_TelemetryParameters *parameters)
{
    _Static_assert(BSP_BLUETOOTH_TX_BUFFER_SIZE >=
                       TELEMETRY_PARAMETER_FRAME_SIZE,
                   "Bluetooth TX buffer is too small for parameter telemetry");
    uint8_t frame[TELEMETRY_PARAMETER_FRAME_SIZE];
    uint16_t offset;
    BeginBinaryFrame(frame,
                     TELEMETRY_PARAMETERS_TYPE,
                     (uint8_t)sizeof(frame),
                     sample->state,
                     sample->flags,
                     protocol->telemetry_sequence,
                     sample->timestamp_ms,
                     &offset);

    const float values[TELEMETRY_PARAMETER_COUNT] = {
        parameters->angle_kp,
        parameters->angle_ki,
        parameters->angle_kd,
        parameters->speed_kp,
        parameters->speed_ki,
        parameters->speed_kd,
        parameters->turn_kp,
        parameters->turn_ki,
        parameters->turn_kd,
        parameters->balance_trim_deg,
        parameters->motor_deadzone_offset,
        parameters->speed_integral_limit_deg,
        parameters->target_pitch_limit_deg,
        parameters->motor_output_limit,
        parameters->turn_output_limit,
        parameters->turn_integral_limit,
        parameters->joystick_speed_limit,
        parameters->joystick_yaw_rate_limit_dps,
        parameters->command_timeout_ms,
        /* 新字段追加在末尾，上位机按位置解包时旧字段含义不变。 */
        parameters->motor_deadzone_band,
        parameters->position_hold_kp,
        parameters->turn_feedforward,
    };
    for (uint32_t index = 0U; index < TELEMETRY_PARAMETER_COUNT; index++)
    {
        PutFloat(frame, &offset, values[index]);
    }

    const uint16_t crc = TelemetryCrc16(frame, offset);
    PutU16(frame, &offset, crc);
    const HAL_StatusTypeDef status =
        BSP_BluetoothSerial_Write(protocol->serial, frame, offset);
    if (status == HAL_OK)
    {
        protocol->parameters_pending = false;
    }
    return status;
}

HAL_StatusTypeDef BluetoothProtocol_SendTelemetry(
    BluetoothProtocol *protocol,
    const BalanceCar_TelemetrySample *sample,
    const BalanceCar_TelemetryParameters *parameters)
{
    if (protocol == NULL || protocol->serial == NULL || sample == NULL ||
        parameters == NULL)
    {
        return HAL_ERROR;
    }

    if (protocol->binary_telemetry_enabled)
    {
        if (protocol->parameters_pending)
        {
            return SendBinaryParameters(protocol, sample, parameters);
        }
        return SendBinarySample(protocol, sample);
    }

    /* 三个定点数字即使达到协议限幅，完整帧也不会超过40字节。 */
    _Static_assert(BSP_BLUETOOTH_TX_BUFFER_SIZE >= 40U,
                   "Bluetooth TX buffer is too small for telemetry");
    char frame[BSP_BLUETOOTH_TX_BUFFER_SIZE];
    uint16_t length = 5U;
    memcpy(frame, "[plot", length);

    const float values[] = {
        sample->state == 2U
            ? sample->yaw_rate_dps + sample->yaw_bias_dps
            : sample->yaw_rate_dps / 10.0F,
        sample->state == 2U
            ? sample->yaw_rate_dps
            : sample->target_yaw_rate_dps / 10.0F,
        sample->state == 2U
            ? sample->yaw_bias_dps
            : sample->turn_output / 100.0F,
    };
    for (uint32_t index = 0U; index < sizeof(values) / sizeof(values[0]); index++)
    {
        frame[length++] = ',';
        AppendFixed2(frame, &length, values[index]);
    }
    frame[length++] = ']';

    return BSP_BluetoothSerial_Write(protocol->serial,
                                     (const uint8_t *)frame,
                                     length);
}

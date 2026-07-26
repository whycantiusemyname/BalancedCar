/** @file Command.c @brief 命令到控制目标、PID参数或状态动作的直接映射。 */

#include "Command.h"

#include <stddef.h>
#include <stdint.h>

CommandAction Command_Apply(const BalanceCar_Command *command,
                            BalanceCar_ControlTarget *target,
                            LongitudinalControl *longitudinal,
                            PID *turn_pid)
{
    if (command == NULL || target == NULL || longitudinal == NULL ||
        turn_pid == NULL)
    {
        return COMMAND_ACTION_INVALID;
    }

    switch (command->type)
    {
        case BALANCE_CAR_COMMAND_SET_FORWARD_SPEED:
            target->forward_speed_counts_per_s = command->data.scalar;
            return COMMAND_ACTION_MOTION_UPDATED;

        case BALANCE_CAR_COMMAND_SET_TURN_SPEED:
            target->yaw_rate_dps = command->data.scalar;
            return COMMAND_ACTION_MOTION_UPDATED;

        case BALANCE_CAR_COMMAND_SET_MOTION_TARGET:
            target->forward_speed_counts_per_s =
                command->data.motion.forward_speed_counts_per_s;
            target->yaw_rate_dps = command->data.motion.yaw_rate_dps;
            return COMMAND_ACTION_MOTION_UPDATED;

        case BALANCE_CAR_COMMAND_SET_BALANCE_TRIM:
            target->balance_trim_deg = command->data.scalar;
            return COMMAND_ACTION_APPLIED;

        case BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_OFFSET:
            target->motor_deadzone_offset =
                command->data.scalar > 0.0F ? command->data.scalar : 0.0F;
            return COMMAND_ACTION_APPLIED;

        case BALANCE_CAR_COMMAND_SET_MOTOR_DEADZONE_BAND:
            target->motor_deadzone_band =
                command->data.scalar > 0.0F ? command->data.scalar : 0.0F;
            return COMMAND_ACTION_APPLIED;

        case BALANCE_CAR_COMMAND_SET_POSITION_HOLD_KP:
            target->position_hold_kp =
                command->data.scalar > 0.0F ? command->data.scalar : 0.0F;
            return COMMAND_ACTION_APPLIED;

        case BALANCE_CAR_COMMAND_SET_TURN_FEEDFORWARD:
            target->turn_feedforward =
                command->data.scalar > 0.0F ? command->data.scalar : 0.0F;
            return COMMAND_ACTION_APPLIED;

        case BALANCE_CAR_COMMAND_SET_PID_GAIN:
        {
            PID *const pid_table[] = {
                &longitudinal->angle.pid,
                &longitudinal->speed_pid,
                turn_pid,
            };
            const uint32_t parameter = command->data.pid.parameter;
            if (parameter >= BALANCE_CAR_PID_PARAMETER_COUNT)
            {
                return COMMAND_ACTION_INVALID;
            }

            PID *pid = pid_table[parameter / 3U];
            float *const gain_table[] = {&pid->kp, &pid->ki, &pid->kd};
            *gain_table[parameter % 3U] = command->data.pid.value;
            PID_Reset(pid);
            return COMMAND_ACTION_APPLIED;
        }

        case BALANCE_CAR_COMMAND_START:
            return COMMAND_ACTION_START;

        case BALANCE_CAR_COMMAND_STOP:
            target->forward_speed_counts_per_s = 0.0F;
            target->yaw_rate_dps = 0.0F;
            return COMMAND_ACTION_STOP;

        case BALANCE_CAR_COMMAND_TEST_MOTOR:
            return COMMAND_ACTION_MOTOR_TEST;

        case BALANCE_CAR_COMMAND_NONE:
        default:
            return COMMAND_ACTION_INVALID;
    }
}

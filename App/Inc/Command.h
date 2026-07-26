#ifndef COMMAND_H
#define COMMAND_H

/**
 * @file Command.h
 * @brief 无对象、无生命周期的整车命令应用函数。
 *
 * 按键和蓝牙都生成BalanceCar_Command。本文件只负责把数值命令写到目标或
 * PID参数，并把需要整车状态机处理的动作返回给App。
 */

#include "BalanceCarTypes.h"
#include "LongitudinalControl.h"

typedef enum
{
    COMMAND_ACTION_INVALID = 0,
    COMMAND_ACTION_APPLIED,
    COMMAND_ACTION_MOTION_UPDATED,
    COMMAND_ACTION_START,
    COMMAND_ACTION_STOP,
    COMMAND_ACTION_MOTOR_TEST
} CommandAction;

/** 应用一条命令；模块不保存任何Handler对象或隐藏状态。 */
CommandAction Command_Apply(const BalanceCar_Command *command,
                            BalanceCar_ControlTarget *target,
                            LongitudinalControl *longitudinal,
                            PID *turn_pid);

#endif /* COMMAND_H */

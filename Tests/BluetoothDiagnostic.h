#ifndef BALANCECAR_BLUETOOTH_DIAGNOSTIC_H
#define BALANCECAR_BLUETOOTH_DIAGNOSTIC_H

#include "stm32f1xx_hal.h"

/**
 * @brief 初始化两阶段蓝牙诊断。
 *
 * 第一阶段直接使用 HAL 轮询收发，用来排除 BSP 驱动因素并确认硬件链路；
 * 收到字符 R 后进入第二阶段，启动新的 BSP 循环 DMA 驱动。
 */
void BluetoothDiagnostic_Init(UART_HandleTypeDef *huart);

/**
 * @brief 推进蓝牙诊断状态机，应在主循环中持续调用。
 */
void BluetoothDiagnostic_Update(void);

#endif // BALANCECAR_BLUETOOTH_DIAGNOSTIC_H

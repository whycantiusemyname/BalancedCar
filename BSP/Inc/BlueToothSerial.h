//
// Created by Nana Daiba on 2026/7/20.
//

#ifndef BALANCECAR_BLUETOOTHSERIAL_H
#define BALANCECAR_BLUETOOTHSERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/** DMA 每次连续接收使用的硬件缓冲区大小。 */
#ifndef BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE
#define BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE 64U
#endif

/**
 * @brief 应用层接收环形缓冲区大小。
 *
 * 环形缓冲区会保留一个空位置以区分“空”和“满”，因此实际容量比该值
 * 少一个字节。DMA 回调把新数据搬入这里，主循环从这里读取。
 */
#ifndef BSP_BLUETOOTH_RX_BUFFER_SIZE
#define BSP_BLUETOOTH_RX_BUFFER_SIZE 256U
#endif

/** DMA 发送暂存区大小；单次发送不能超过该值。 */
#ifndef BSP_BLUETOOTH_TX_BUFFER_SIZE
#define BSP_BLUETOOTH_TX_BUFFER_SIZE 128U
#endif

/**
 * @brief 基于 HAL UART DMA 的蓝牙透明串口对象。
 *
 * 接收使用 HAL_UARTEx_ReceiveToIdle_DMA() 配合循环 DMA。DMA 回调根据
 * dma_rx_last_pos 找出本次新增的数据，再写入应用层环形缓冲区。
 * 发送使用普通 DMA，并由内部 tx_buffer 保证异步发送期间数据仍然有效。
 * 本驱动只负责字节流收发，不负责解析调参协议。
 */
typedef struct {
    UART_HandleTypeDef *huart;                                 /**< 绑定的 HAL UART 句柄。 */

    uint8_t dma_rx_buffer[BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE];   /**< 循环 DMA 接收缓冲区。 */
    volatile uint16_t dma_rx_last_pos;                         /**< 上次已处理到的 DMA 位置。 */

    uint8_t rx_buffer[BSP_BLUETOOTH_RX_BUFFER_SIZE];           /**< 应用层接收环形缓冲区。 */
    volatile uint16_t rx_head;                                 /**< DMA 回调写入位置。 */
    volatile uint16_t rx_tail;                                 /**< 主循环读取位置。 */
    volatile bool rx_overflow;                                 /**< 接收环形缓冲区溢出标志。 */

    uint8_t tx_buffer[BSP_BLUETOOTH_TX_BUFFER_SIZE];           /**< DMA 发送期间的数据副本。 */
    volatile bool tx_busy;                                     /**< DMA 发送忙标志。 */
} BSP_BluetoothSerial;

/**
 * @brief 初始化蓝牙串口对象并启动循环 DMA 空闲线接收。
 * @param serial 待初始化的蓝牙串口对象。
 * @param huart  已由 STM32CubeMX 初始化的 UART 句柄，例如 &huart2。
 * @return HAL_UARTEx_ReceiveToIdle_DMA() 的启动结果。
 *
 * CubeMX 中 RX DMA 必须配置成 Circular，TX DMA 配置成 Normal，并开启
 * USART 和对应 DMA 通道的中断。当前实现会把一个蓝牙对象自动注册给
 * HAL 全局回调，因此同一时间只支持一个 BSP_BluetoothSerial 实例。
 */
HAL_StatusTypeDef BSP_BluetoothSerial_Init(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
);

/**
 * @brief 尝试把HC-04模块和本侧USART一起切换到目标波特率。
 * @param huart    已按CubeMX默认波特率初始化、尚未启动DMA的UART句柄。
 * @param baudrate 目标波特率;与当前值相同则直接返回HAL_OK。
 *
 * 必须在 BSP_BluetoothSerial_Init() 之前、蓝牙未被主机连接时调用。
 * 模块已在目标波特率时,旧波特率发出的AT命令只是被忽略的噪声,因此
 * 每次上电重复协商是安全的幂等操作。
 */
HAL_StatusTypeDef BSP_BluetoothSerial_NegotiateBaud(
    UART_HandleTypeDef *huart,
    uint32_t baudrate
);

/**
 * @brief 处理 UART 空闲、DMA 半满或全满产生的接收事件。
 * @param serial  蓝牙串口对象。
 * @param huart   HAL_UARTEx_RxEventCallback() 传入的 UART 句柄。
 * @param position HAL 回调给出的 DMA 当前写入位置。
 *
 * BlueToothSerial.c 已提供 HAL_UARTEx_RxEventCallback() 转发，应用层通常
 * 不需要直接调用。循环 DMA 不需要在每次事件后重新启动。
 */
void BSP_BluetoothSerial_OnRxEvent(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart,
    uint16_t position
);

/**
 * @brief 处理 UART DMA 发送完成事件。
 * @param serial 蓝牙串口对象。
 * @param huart  HAL_UART_TxCpltCallback() 传入的 UART 句柄。
 *
 * BlueToothSerial.c 已提供 HAL_UART_TxCpltCallback() 转发，用于清除 tx_busy。
 */
void BSP_BluetoothSerial_OnTxComplete(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
);

/**
 * @brief 处理 UART 接收错误并重新启动循环 DMA 接收。
 * @param serial 蓝牙串口对象。
 * @param huart  HAL_UART_ErrorCallback() 传入的 UART 句柄。
 * @return DMA 接收重新启动的结果；若不是绑定的 UART，则返回 HAL_OK。
 * BlueToothSerial.c 已提供 HAL_UART_ErrorCallback() 转发。
 */
HAL_StatusTypeDef BSP_BluetoothSerial_OnError(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
);

/**
 * @brief 查询当前可读取的字节数。
 * @param serial 蓝牙串口对象。
 * @return 接收环形缓冲区中尚未读取的字节数。
 */
uint16_t BSP_BluetoothSerial_Available(const BSP_BluetoothSerial *serial);

/**
 * @brief 从接收环形缓冲区读取一个字节。
 * @param serial 蓝牙串口对象。
 * @param data   用于保存读取结果的地址。
 * @return 成功读取返回 true；缓冲区为空返回 false。
 */
bool BSP_BluetoothSerial_ReadByte(
    BSP_BluetoothSerial *serial,
    uint8_t *data
);

/**
 * @brief 从接收环形缓冲区批量读取数据。
 * @param serial     蓝牙串口对象。
 * @param data       目标缓冲区。
 * @param max_length 本次最多读取的字节数。
 * @return 实际读取的字节数。
 */
uint16_t BSP_BluetoothSerial_Read(
    BSP_BluetoothSerial *serial,
    uint8_t *data,
    uint16_t max_length
);

/**
 * @brief 清空应用层尚未读取的数据和溢出标志。
 * @param serial 蓝牙串口对象。
 *
 * 函数不会停止循环 DMA；调用时仍在硬件 DMA 缓冲区内的数据会在后续
 * 接收事件中正常进入应用层缓冲区。
 */
void BSP_BluetoothSerial_ClearRx(BSP_BluetoothSerial *serial);

/**
 * @brief 读取并清除接收缓冲区溢出标志。
 * @param serial 蓝牙串口对象。
 * @return 自上次读取以来发生过溢出时返回 true。
 */
bool BSP_BluetoothSerial_TakeOverflow(BSP_BluetoothSerial *serial);

/**
 * @brief 复制数据并启动一次非阻塞 DMA 发送。
 * @param serial 蓝牙串口对象。
 * @param data   待发送数据。
 * @param length 数据长度，不能超过 BSP_BLUETOOTH_TX_BUFFER_SIZE。
 * @return 成功启动返回 HAL_OK；正在发送返回 HAL_BUSY；参数或长度非法返回 HAL_ERROR。
 *
 * 函数返回后调用者可以立即修改原始 data，因为数据已经复制进对象内部
 * 的 tx_buffer。发送完成前再次调用会返回 HAL_BUSY。
 */
HAL_StatusTypeDef BSP_BluetoothSerial_Write(
    BSP_BluetoothSerial *serial,
    const uint8_t *data,
    uint16_t length
);

/**
 * @brief 查询 DMA 发送是否仍在进行。
 * @return 正在发送返回 true，否则返回 false。
 */
bool BSP_BluetoothSerial_IsTxBusy(const BSP_BluetoothSerial *serial);

#endif //BALANCECAR_BLUETOOTHSERIAL_H

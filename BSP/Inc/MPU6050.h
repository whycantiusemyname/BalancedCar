#ifndef BALANCECAR_MPU6050_H
#define BALANCECAR_MPU6050_H

/**
 * @file MPU6050.h
 * @brief MPU6050 的阻塞式 STM32 HAL I2C 驱动接口。
 *
 * 当前初始化流程负责绑定 I2C、探测设备地址、校验 WHO_AM_I，
 * 并将器件配置为 100 Hz、约 42 Hz 低通、陀螺仪 ±250 dps、
 * 加速度计 ±2 g 的工作模式。
 * 本驱动返回的传感器数据均为寄存器原始值，不包含单位换算、
 * 零偏校准、温度补偿或姿态解算。
 */

#include <stdint.h>
#include "stm32f1xx_hal.h"

/**
 * @brief 一颗 MPU6050 实例及其总线信息。
 *
 * `i2c` 是非拥有型指针：它指向由 CubeMX 生成并初始化的 HAL I2C
 * 句柄，驱动不会创建或释放该句柄。`address` 保存 HAL 所要求的左移
 * 一位后的设备地址，例如 7 位地址 0x68 在这里保存为 0xD0。
 */
typedef struct
{
    I2C_HandleTypeDef *i2c; /**< 已由 MX_I2Cx_Init() 初始化的 HAL 句柄。 */
    uint16_t address;       /**< HAL 格式的设备地址，即 7 位地址左移一位。 */
} BSP_MPU6050;

/**
 * @brief 一次突发读取获得的七个 16 位原始测量值。
 *
 * 加速度、温度和角速度寄存器均采用高字节在前的有符号二进制补码。
 * 各字段尚未根据量程换算为 g、摄氏度或 dps。
 */
typedef struct
{
    int16_t Gyro_X;     /**< X 轴角速度原始值。 */
    int16_t Gyro_Y;     /**< Y 轴角速度原始值。 */
    int16_t Gyro_Z;     /**< Z 轴角速度原始值。 */
    int16_t Accel_X;    /**< X 轴加速度原始值。 */
    int16_t Accel_Y;    /**< Y 轴加速度原始值。 */
    int16_t Accel_Z;    /**< Z 轴加速度原始值。 */
    int16_t Temperature;/**< 芯片内部温度传感器原始值。 */
} BSP_MPU6050_RawData;

/**
 * @brief 绑定总线、确认器件身份并配置传感器工作模式。
 * @param imu 由调用方分配的 MPU6050 实例。
 * @param i2c 已完成初始化的 HAL I2C 句柄，例如 `&hi2c2`。
 * @param address 7 位 I2C 地址，通常为 0x68；AD0 拉高时为 0x69。
 * @return HAL_OK 表示身份检查与全部配置成功；否则返回 HAL 错误状态。
 * @note 函数包含复位与启动等待，是阻塞式初始化。
 */
HAL_StatusTypeDef MPU6050_Init(BSP_MPU6050 *imu,
                               I2C_HandleTypeDef *i2c,
                               uint16_t address);

/**
 * @brief 从 ACCEL_XOUT_H 开始突发读取连续 14 字节原始数据。
 * @param imu 已成功初始化的 MPU6050 实例。
 * @param data 接收加速度、温度和角速度原始值的结构体。
 * @return HAL_OK 表示数据有效；其他值表示 I2C 读取失败。
 * @note 函数为阻塞式调用，数据未进行零偏或比例换算。
 */
HAL_StatusTypeDef MPU6050_ReadRaw(BSP_MPU6050 *imu, BSP_MPU6050_RawData *data);

/**
 * @brief 读取一个 8 位地址、8 位数据的 MPU6050 寄存器。
 * @param imu 已绑定 I2C 总线的 MPU6050 实例。
 * @param reg 要读取的寄存器地址。
 * @param value 接收寄存器值的非空指针。
 * @return HAL I2C 操作状态。
 */
HAL_StatusTypeDef MPU6050_ReadRegister(BSP_MPU6050 *imu, uint8_t reg, uint8_t *value);

#endif /* BALANCECAR_MPU6050_H */

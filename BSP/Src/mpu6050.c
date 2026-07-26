/**
 * @file MPU6050.c
 * @brief MPU6050 基础 I2C 通信与原始数据读取实现。
 */

#include "MPU6050.h"

#define MPU6050_REG_WHO_AM_I       0x75U
#define MPU6050_WHO_AM_I_EXPECTED  0x68U
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU

#define MPU6050_READY_TRIALS        3U
#define MPU6050_I2C_TIMEOUT_MS        5U
#define MPU6050_RESET_DELAY_MS      100U
#define MPU6050_STARTUP_DELAY_MS    100U

static HAL_StatusTypeDef MPU6050_WriteRegister(
    BSP_MPU6050 *imu,
    uint8_t reg,
    uint8_t value
) {
    if (imu == NULL) return HAL_ERROR;
    if (imu->i2c == NULL) return HAL_ERROR;

    return HAL_I2C_Mem_Write(imu->i2c,
        imu->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        MPU6050_I2C_TIMEOUT_MS);

}
HAL_StatusTypeDef MPU6050_Init(BSP_MPU6050 *imu,
                               I2C_HandleTypeDef *i2c,
                               uint16_t address)
{
    /* 驱动需要由调用方提供实例和已经初始化的 HAL I2C 句柄。 */
    if (imu == NULL) return HAL_ERROR;
    if (i2c == NULL) return HAL_ERROR;

    uint8_t who_am_i = 0U;

    /* HAL 接口要求设备的 7 位地址左移一位。 */
    imu->i2c = i2c;
    imu->address = address << 1;

    /* 地址探测只确认有从机 ACK；随后还需要读取身份寄存器。 */
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
        imu->i2c,
        imu->address,
        MPU6050_READY_TRIALS,
        MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    /* WHO_AM_I 可排除同一地址上的其他器件，但不能鉴别芯片真伪。 */
    status = MPU6050_ReadRegister(imu, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (status != HAL_OK)
    {
        return status;
    }

    if (who_am_i != MPU6050_WHO_AM_I_EXPECTED)
    {
        return HAL_ERROR;
    }

    status = MPU6050_WriteRegister(imu, MPU6050_REG_PWR_MGMT_1, 0x80U);
    if (status != HAL_OK) return status;
    HAL_Delay(MPU6050_RESET_DELAY_MS);

    status = MPU6050_WriteRegister(imu, MPU6050_REG_PWR_MGMT_1, 0x01U);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteRegister(imu, MPU6050_REG_PWR_MGMT_2, 0x00U);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteRegister(imu, MPU6050_REG_CONFIG, 0x03U);
    if (status != HAL_OK) return status;
    /* DLPF开启时内部输出为1kHz；/5得到与5ms控制环一致的200Hz。 */
    status = MPU6050_WriteRegister(imu, MPU6050_REG_SMPLRT_DIV, 0x04U);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteRegister(imu, MPU6050_REG_GYRO_CONFIG, 0x00U);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteRegister(imu, MPU6050_REG_ACCEL_CONFIG, 0x00U);
    if (status != HAL_OK) return status;

    /* 等待陀螺仪 PLL 和传感器输出稳定后再允许上层读取。 */
    HAL_Delay(MPU6050_STARTUP_DELAY_MS);
    return HAL_OK;
}

/* MPU6050 连续数据寄存器按高字节、低字节顺序存放。 */
static int16_t MPU6050_CombineBytes(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

HAL_StatusTypeDef MPU6050_ReadRaw(BSP_MPU6050 *imu, BSP_MPU6050_RawData *data)
{
    if (imu == NULL) return HAL_ERROR;
    if (imu->i2c == NULL) return HAL_ERROR;
    if (data == NULL) return HAL_ERROR;

    uint8_t buffer[14];

    /*
     * MPU6050 会在每发送一个字节后递增内部寄存器指针，因此从
     * 0x3B 开始的一次事务即可取得三轴加速度、温度和三轴角速度。
     */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(imu->i2c,
        imu->address,
        MPU6050_REG_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        sizeof(buffer),
        MPU6050_I2C_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return status;
    }

    /* 每个测量值由两个连续寄存器组成，并按有符号 16 位数解释。 */
    data->Accel_X = MPU6050_CombineBytes(buffer[0], buffer[1]);
    data->Accel_Y = MPU6050_CombineBytes(buffer[2], buffer[3]);
    data->Accel_Z = MPU6050_CombineBytes(buffer[4], buffer[5]);
    data->Temperature = MPU6050_CombineBytes(buffer[6], buffer[7]);
    data->Gyro_X = MPU6050_CombineBytes(buffer[8], buffer[9]);
    data->Gyro_Y = MPU6050_CombineBytes(buffer[10], buffer[11]);
    data->Gyro_Z = MPU6050_CombineBytes(buffer[12], buffer[13]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRegister(BSP_MPU6050 *imu,
                                       uint8_t reg,
                                       uint8_t *value)
{
    if (imu == NULL) return HAL_ERROR;
    if (imu->i2c == NULL) return HAL_ERROR;
    if (value == NULL) return HAL_ERROR;

    return HAL_I2C_Mem_Read(imu->i2c,
        imu->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1U,
        MPU6050_I2C_TIMEOUT_MS);
}

//
// Created by Nana Daiba on 2026/7/20.
//

#include "BlueToothSerial.h"

#include <string.h>

_Static_assert(BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE > 0U,
               "Bluetooth DMA RX buffer must not be empty");
_Static_assert(BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE <= UINT16_MAX,
               "Bluetooth DMA RX buffer is too large for HAL UART");
_Static_assert(BSP_BLUETOOTH_RX_BUFFER_SIZE > 1U,
               "Bluetooth RX ring buffer must contain at least two bytes");
_Static_assert(BSP_BLUETOOTH_RX_BUFFER_SIZE <= UINT16_MAX,
               "Bluetooth RX ring buffer is too large");
_Static_assert(BSP_BLUETOOTH_TX_BUFFER_SIZE > 0U,
               "Bluetooth TX buffer must not be empty");
_Static_assert(BSP_BLUETOOTH_TX_BUFFER_SIZE <= UINT16_MAX,
               "Bluetooth TX buffer is too large for HAL UART");

/* 当前板子只有一个蓝牙串口；Init() 会把它注册给三个 HAL 全局回调。 */
static BSP_BluetoothSerial *registered_serial = NULL;

static uint32_t BSP_BluetoothSerial_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void BSP_BluetoothSerial_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint16_t BSP_BluetoothSerial_NextRxIndex(uint16_t index)
{
    index++;
    if (index >= BSP_BLUETOOTH_RX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

/* 由中断上下文调用。缓冲区满时丢弃最新字节，并记录溢出。 */
static void BSP_BluetoothSerial_PushRxByte(
    BSP_BluetoothSerial *serial,
    uint8_t data
)
{
    uint16_t next_head = BSP_BluetoothSerial_NextRxIndex(serial->rx_head);

    if (next_head == serial->rx_tail) {
        serial->rx_overflow = true;
        return;
    }

    serial->rx_buffer[serial->rx_head] = data;
    serial->rx_head = next_head;
}

static void BSP_BluetoothSerial_CopyDmaRange(
    BSP_BluetoothSerial *serial,
    uint16_t begin,
    uint16_t end
)
{
    for (uint16_t index = begin; index < end; index++) {
        BSP_BluetoothSerial_PushRxByte(serial, serial->dma_rx_buffer[index]);
    }
}

HAL_StatusTypeDef BSP_BluetoothSerial_NegotiateBaud(
    UART_HandleTypeDef *huart,
    uint32_t baudrate
)
{
    if (huart == NULL || baudrate == 0U)
    {
        return HAL_ERROR;
    }
    if (huart->Init.BaudRate == baudrate)
    {
        return HAL_OK;
    }
    if (baudrate != 115200U)
    {
        /* AT命令串为固定文本;目前只支持切换到115200。 */
        return HAL_ERROR;
    }

    /*
     * 汇承HC-04的AT命令使用完整数值格式;老固件不要求行结束符,新固件
     * 要求\r\n,两种都发一遍。不使用"AT+BAUD8"这类序号方言:不同厂商
     * 序号映射不一致(HM-10的8是230400),写错会永久失联。
     */
    static const char *const commands[] = {
        "AT+BAUD=115200",
        "AT+BAUD=115200\r\n",
    };
    for (uint32_t index = 0U;
         index < sizeof(commands) / sizeof(commands[0]);
         index++)
    {
        (void)HAL_UART_Transmit(huart,
                                (const uint8_t *)commands[index],
                                (uint16_t)strlen(commands[index]),
                                100U);
        /* 留出模块解析和按旧波特率回复OK的时间。 */
        HAL_Delay(150U);
    }

    if (HAL_UART_DeInit(huart) != HAL_OK)
    {
        return HAL_ERROR;
    }
    huart->Init.BaudRate = baudrate;
    return HAL_UART_Init(huart);
}

HAL_StatusTypeDef BSP_BluetoothSerial_Init(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
)
{
    if (serial == NULL || huart == NULL ||
        huart->hdmarx == NULL || huart->hdmatx == NULL) {
        return HAL_ERROR;
    }

    if (huart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        huart->hdmatx->Init.Mode != DMA_NORMAL) {
        return HAL_ERROR;
    }

    /* HAL 的全局回调只能自动路由到一个蓝牙对象。 */
    if (registered_serial != NULL) {
        return HAL_BUSY;
    }

    memset(serial, 0, sizeof(*serial));
    serial->huart = huart;
    registered_serial = serial;

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        serial->dma_rx_buffer,
        BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE
    );

    if (status != HAL_OK) {
        registered_serial = NULL;
        serial->huart = NULL;
    }

    return status;
}

void BSP_BluetoothSerial_OnRxEvent(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart,
    uint16_t position
)
{
    if (serial == NULL || huart != serial->huart ||
        position > BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE) {
        return;
    }

    uint16_t last_position = serial->dma_rx_last_pos;

    if (position == BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE) {
        /* DMA Transfer Complete：处理缓冲区尾部，随后从下标 0 继续。 */
        BSP_BluetoothSerial_CopyDmaRange(
            serial,
            last_position,
            BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE
        );
        serial->dma_rx_last_pos = 0U;
    } else if (position > last_position) {
        BSP_BluetoothSerial_CopyDmaRange(serial, last_position, position);
        serial->dma_rx_last_pos = position;
    } else if (position < last_position) {
        /* DMA 在两次事件之间发生回绕。 */
        BSP_BluetoothSerial_CopyDmaRange(
            serial,
            last_position,
            BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE
        );
        BSP_BluetoothSerial_CopyDmaRange(serial, 0U, position);
        serial->dma_rx_last_pos = position;
    }
}

void BSP_BluetoothSerial_OnTxComplete(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
)
{
    if (serial != NULL && huart == serial->huart) {
        serial->tx_busy = false;
    }
}

HAL_StatusTypeDef BSP_BluetoothSerial_OnError(
    BSP_BluetoothSerial *serial,
    UART_HandleTypeDef *huart
)
{
    if (serial == NULL || huart != serial->huart) {
        return HAL_OK;
    }

    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        serial->tx_busy = false;
    }

    /* 如果接收仍在运行，错误是可恢复的，不重复启动 DMA。 */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX) {
        return HAL_OK;
    }

    serial->dma_rx_last_pos = 0U;
    return HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        serial->dma_rx_buffer,
        BSP_BLUETOOTH_DMA_RX_BUFFER_SIZE
    );
}

uint16_t BSP_BluetoothSerial_Available(const BSP_BluetoothSerial *serial)
{
    if (serial == NULL) {
        return 0U;
    }

    uint16_t head = serial->rx_head;
    uint16_t tail = serial->rx_tail;

    if (head >= tail) {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(BSP_BLUETOOTH_RX_BUFFER_SIZE - tail + head);
}

bool BSP_BluetoothSerial_ReadByte(
    BSP_BluetoothSerial *serial,
    uint8_t *data
)
{
    if (serial == NULL || data == NULL || serial->rx_tail == serial->rx_head) {
        return false;
    }

    *data = serial->rx_buffer[serial->rx_tail];
    serial->rx_tail = BSP_BluetoothSerial_NextRxIndex(serial->rx_tail);
    return true;
}

uint16_t BSP_BluetoothSerial_Read(
    BSP_BluetoothSerial *serial,
    uint8_t *data,
    uint16_t max_length
)
{
    if (serial == NULL || data == NULL) {
        return 0U;
    }

    uint16_t length = 0U;
    while (length < max_length &&
           BSP_BluetoothSerial_ReadByte(serial, &data[length])) {
        length++;
    }

    return length;
}

void BSP_BluetoothSerial_ClearRx(BSP_BluetoothSerial *serial)
{
    if (serial == NULL) {
        return;
    }

    uint32_t primask = BSP_BluetoothSerial_EnterCritical();

    serial->rx_tail = serial->rx_head;
    serial->rx_overflow = false;

    BSP_BluetoothSerial_ExitCritical(primask);
}

bool BSP_BluetoothSerial_TakeOverflow(BSP_BluetoothSerial *serial)
{
    if (serial == NULL) {
        return false;
    }

    uint32_t primask = BSP_BluetoothSerial_EnterCritical();
    bool overflow = serial->rx_overflow;
    serial->rx_overflow = false;
    BSP_BluetoothSerial_ExitCritical(primask);

    return overflow;
}

HAL_StatusTypeDef BSP_BluetoothSerial_Write(
    BSP_BluetoothSerial *serial,
    const uint8_t *data,
    uint16_t length
)
{
    if (serial == NULL || serial->huart == NULL || data == NULL ||
        length == 0U || length > BSP_BLUETOOTH_TX_BUFFER_SIZE) {
        return HAL_ERROR;
    }

    uint32_t primask = BSP_BluetoothSerial_EnterCritical();
    if (serial->tx_busy) {
        BSP_BluetoothSerial_ExitCritical(primask);
        return HAL_BUSY;
    }
    serial->tx_busy = true;
    BSP_BluetoothSerial_ExitCritical(primask);

    memcpy(serial->tx_buffer, data, length);

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        serial->huart,
        serial->tx_buffer,
        length
    );

    if (status != HAL_OK) {
        serial->tx_busy = false;
    }

    return status;
}

bool BSP_BluetoothSerial_IsTxBusy(const BSP_BluetoothSerial *serial)
{
    return serial != NULL && serial->tx_busy;
}

/* HAL 弱回调的统一转发。初始化前 registered_serial 为 NULL，不执行操作。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (registered_serial != NULL) {
        BSP_BluetoothSerial_OnRxEvent(registered_serial, huart, size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (registered_serial != NULL) {
        BSP_BluetoothSerial_OnTxComplete(registered_serial, huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (registered_serial != NULL) {
        (void)BSP_BluetoothSerial_OnError(registered_serial, huart);
    }
}

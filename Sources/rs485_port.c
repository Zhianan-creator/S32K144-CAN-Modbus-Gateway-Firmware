  #include "rs485_port.h"

/*
 * 阅读提示：
 * RS485 使用 LPUART2 收发，PTE14 控制 DE/RE 方向。
 * Modbus RTU 只调用这里的初始化、取字节和发送帧接口。
 */

#include "Cpu.h"
#include "lpuart_driver.h"

#define RS485_DE_GPIO             PTE
#define RS485_DE_PIN              14U
#define RS485_TX_WAIT_LIMIT       10000000UL

static lpuart_state_t rs485_uart_state;

static const lpuart_user_config_t rs485_uart_config = {
    .transferType = LPUART_USING_INTERRUPTS,
    .baudRate = 115200U,
    .parityMode = LPUART_PARITY_DISABLED,
    .stopBitCount = LPUART_ONE_STOP_BIT,
    .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
    .rxDMAChannel = 0U,
    .txDMAChannel = 0U,
};

static void RS485_SetReceiveMode(void)
{
    PINS_DRV_WritePin(RS485_DE_GPIO, RS485_DE_PIN, 0U);
}

static void RS485_SetTransmitMode(void)
{
    PINS_DRV_WritePin(RS485_DE_GPIO, RS485_DE_PIN, 1U);
}

bool RS485_PortInit(void)
{
    status_t status;

    RS485_SetReceiveMode();
    status = LPUART_DRV_Init(RS485_UART_INSTANCE,
                             &rs485_uart_state,
                             &rs485_uart_config);
    return (status == STATUS_SUCCESS);
}

/* 发送时先切到 TX，等待 UART 发送完成后再切回 RX，避免占住总线。 */
bool RS485_PortSend(const uint8_t *data, uint16_t length)
{
    status_t status;
    uint32_t bytes_remaining = 0U;
    uint32_t wait_count = 0U;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    RS485_SetTransmitMode();
    status = LPUART_DRV_SendData(RS485_UART_INSTANCE, data, length);
    if (status == STATUS_SUCCESS)
    {
        do
        {
            status = LPUART_DRV_GetTransmitStatus(RS485_UART_INSTANCE,
                                                  &bytes_remaining);
            wait_count++;
        }
        while ((status == STATUS_BUSY) &&
               (wait_count < RS485_TX_WAIT_LIMIT));
    }

    /* 驱动返回后再次确认最后一个停止位已经离开发送引脚。 */
    while (((LPUART2->STAT & LPUART_STAT_TC_MASK) == 0U) &&
           (wait_count < RS485_TX_WAIT_LIMIT))
    {
        wait_count++;
    }

    if ((status != STATUS_SUCCESS) ||
        ((LPUART2->STAT & LPUART_STAT_TC_MASK) == 0U))
    {
        (void)LPUART_DRV_AbortSendingData(RS485_UART_INSTANCE);
    }

    RS485_SetReceiveMode();
    return (status == STATUS_SUCCESS) &&
           ((LPUART2->STAT & LPUART_STAT_TC_MASK) != 0U);
}

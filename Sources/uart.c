/*
 * 品智科技
 * LPUART1系统监控摘要输出
 */
#include "uart.h"

#include "lpuart1.h"

#include <stddef.h>

/*
 * 阅读提示：
 * LPUART1 只做调试监控输出：10 秒一行 SYS 摘要，再跟一行最近 CAN 原始帧。
 * 它不参与 Modbus/RS485 通信，现场调试时用来快速判断系统是否健康。
 */

#define MONITOR_TX_WAIT_LIMIT 1000000UL
#define MONITOR_LINE_MAX      256U

static char monitor_tx_buf[MONITOR_LINE_MAX];
static bool monitor_ready = false;

static void Monitor_AppendChar(uint16_t *pos, char value)
{
    if (*pos < (MONITOR_LINE_MAX - 1U))
    {
        monitor_tx_buf[*pos] = value;
        (*pos)++;
    }
}

static void Monitor_AppendText(uint16_t *pos, const char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        Monitor_AppendChar(pos, *text);
        text++;
    }
}

static void Monitor_AppendU32(uint16_t *pos, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        count++;
    }
    while ((value != 0U) && (count < sizeof(digits)));

    while (count > 0U)
    {
        count--;
        Monitor_AppendChar(pos, digits[count]);
    }
}

static void Monitor_AppendHexNibble(uint16_t *pos, uint8_t value)
{
    value &= 0x0FU;
    if (value < 10U)
    {
        Monitor_AppendChar(pos, (char)('0' + value));
    }
    else
    {
        Monitor_AppendChar(pos, (char)('A' + (value - 10U)));
    }
}

static void Monitor_AppendHex8(uint16_t *pos, uint8_t value)
{
    Monitor_AppendHexNibble(pos, (uint8_t)(value >> 4U));
    Monitor_AppendHexNibble(pos, value);
}

static void Monitor_AppendHex32NoLeadingZero(uint16_t *pos, uint32_t value)
{
    uint8_t shift;
    bool started = false;

    for (shift = 28U; shift > 0U; shift = (uint8_t)(shift - 4U))
    {
        uint8_t nibble = (uint8_t)((value >> shift) & 0x0FU);
        if ((nibble != 0U) || started)
        {
            Monitor_AppendHexNibble(pos, nibble);
            started = true;
        }
    }

    Monitor_AppendHexNibble(pos, (uint8_t)(value & 0x0FU));
}

static void Monitor_AppendFieldU32(uint16_t *pos, const char *name, uint32_t value)
{
    Monitor_AppendChar(pos, ' ');
    Monitor_AppendText(pos, name);
    Monitor_AppendChar(pos, '=');
    Monitor_AppendU32(pos, value);
}

static void Monitor_AppendCanRawLine(uint16_t *pos,
                                     const MonitorCanRawFrame_t *frame)
{
    uint8_t index;
    uint8_t bytes_to_print;

    if ((frame == NULL) || (!frame->valid))
    {
        Monitor_AppendText(pos, "CAN RAW none\r\n");
        return;
    }

    bytes_to_print = (frame->dlc > 8U) ? 8U : frame->dlc;

    Monitor_AppendText(pos, "CAN RAW ID=0x");
    Monitor_AppendHex32NoLeadingZero(pos, frame->id);
    Monitor_AppendText(pos, " DLC=");
    Monitor_AppendU32(pos, frame->dlc);
    Monitor_AppendText(pos, " DATA=");

    for (index = 0U; index < bytes_to_print; index++)
    {
        if (index > 0U)
        {
            Monitor_AppendChar(pos, ' ');
        }
        Monitor_AppendHex8(pos, frame->data[index]);
    }

    Monitor_AppendText(pos, "\r\n");
}

static bool Monitor_SendLine(uint16_t length)
{
    status_t status;
    uint32_t bytes_remaining = 0U;
    uint32_t wait_count = 0U;

    if ((!monitor_ready) || (length == 0U))
    {
        return false;
    }

    status = LPUART_DRV_SendData(INST_LPUART1,
                                 (uint8_t *)monitor_tx_buf,
                                 length);
    if (status != STATUS_SUCCESS)
    {
        return false;
    }

    do
    {
        status = LPUART_DRV_GetTransmitStatus(INST_LPUART1,
                                              &bytes_remaining);
        wait_count++;
    }
    while ((status == STATUS_BUSY) &&
           (wait_count < MONITOR_TX_WAIT_LIMIT));

    if (status != STATUS_SUCCESS)
    {
        (void)LPUART_DRV_AbortSendingData(INST_LPUART1);
        return false;
    }

    return true;
}

bool LPUART1_MonitorInit(void)
{
    status_t status;

    status = LPUART_DRV_Init(INST_LPUART1,
                             &lpuart1_State,
                             &lpuart1_InitConfig0);
    monitor_ready = (status == STATUS_SUCCESS);

    if (monitor_ready)
    {
        uint16_t pos = 0U;

        Monitor_AppendText(&pos, "LPUART1 monitor ready\r\n");
        (void)Monitor_SendLine(pos);
    }

    return monitor_ready;
}

/* 每 10 秒由 main.c 调用一次，保持输出低频，避免串口刷屏影响主循环。 */
void LPUART1_MonitorPrintSummary(const BmsCanData_t *bms,
                                 const ModbusRtuStats_t *modbus,
                                 const MonitorCanRawFrame_t *last_can_raw,
                                 uint32_t now_ms,
                                 uint32_t can_rx_count,
                                 uint32_t can_rx_overflow_count,
                                 status_t can_rx_rearm_status,
                                 uint32_t can_init_error_count,
                                 status_t can_last_init_error,
                                 bool modbus_init_ok)
{
    uint16_t pos = 0U;
    uint32_t bms_online = 0U;
    uint32_t bms_valid = 0U;
    uint32_t mb_rx = 0U;
    uint32_t mb_tx = 0U;
    uint32_t crc_err = 0U;
    uint32_t addr_miss = 0U;

    (void)can_init_error_count;
    (void)can_last_init_error;

    if (!monitor_ready)
    {
        return;
    }

    if (bms != NULL)
    {
        bms_online = bms->online ? 1U : 0U;
        bms_valid = bms->valid ? 1U : 0U;
    }

    if (modbus != NULL)
    {
        mb_rx = modbus->rx_frames;
        mb_tx = modbus->tx_frames;
        crc_err = modbus->crc_errors;
        addr_miss = modbus->address_misses;
    }

    Monitor_AppendText(&pos, "SYS");
    Monitor_AppendFieldU32(&pos, "t", now_ms / 1000U);
    Monitor_AppendChar(&pos, 's');
    Monitor_AppendFieldU32(&pos, "CAN_RX", can_rx_count);
    Monitor_AppendFieldU32(&pos, "CAN_OVF", can_rx_overflow_count);
    Monitor_AppendFieldU32(&pos, "CAN_REARM", (uint32_t)can_rx_rearm_status);
    Monitor_AppendFieldU32(&pos, "BMS_ON", bms_online);
    Monitor_AppendFieldU32(&pos, "BMS_VALID", bms_valid);
    Monitor_AppendFieldU32(&pos, "MB_RX", mb_rx);
    Monitor_AppendFieldU32(&pos, "MB_TX", mb_tx);
    Monitor_AppendFieldU32(&pos, "CRC_ERR", crc_err);
    Monitor_AppendFieldU32(&pos, "ADDR_MISS", addr_miss);
    Monitor_AppendFieldU32(&pos, "RS485_OK", modbus_init_ok ? 1U : 0U);
    Monitor_AppendText(&pos, "\r\n");
    Monitor_AppendCanRawLine(&pos, last_can_raw);

    (void)Monitor_SendLine(pos);
}

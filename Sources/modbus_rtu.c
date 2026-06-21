#include "modbus_rtu.h"

#include "Cpu.h"
#include "modbus_reg.h"
#include "rs485_port.h"

#include <stddef.h>
#include <string.h>

/*
 * 阅读提示：
 * 本文件负责标准 Modbus RTU 帧处理：收包状态机、3.5 字符间隔、CRC 校验和响应发送。
 * 具体寄存器含义不在这里，读写会转到 modbus_reg.c。
 */

#define MODBUS_FUNC_READ_HOLDING       0x03U
#define MODBUS_FUNC_WRITE_SINGLE       0x06U
#define MODBUS_FUNC_WRITE_MULTIPLE     0x10U

#define MODBUS_EX_ILLEGAL_FUNCTION     0x01U
#define MODBUS_EX_ILLEGAL_ADDRESS      0x02U
#define MODBUS_EX_ILLEGAL_VALUE        0x03U
#define MODBUS_EX_SERVER_FAILURE       0x04U

#define MODBUS_READ_MAX_QUANTITY       125U
#define MODBUS_WRITE_MAX_QUANTITY      123U
#define MODBUS_FRAME_GAP_TICKS         70000U
#define MODBUS_NO_READY_BUFFER         0xFFU

static uint8_t rx_buffers[2][MODBUS_RTU_MAX_FRAME];
static volatile uint16_t rx_lengths[2];
static volatile uint8_t active_buffer;
static volatile uint8_t ready_buffer;
static volatile uint8_t uart_rx_byte;
static ModbusRtuStats_t modbus_stats;
static uint8_t response_buffer[MODBUS_RTU_MAX_FRAME];
static uint16_t read_values[MODBUS_READ_MAX_QUANTITY];
static uint16_t write_values[MODBUS_WRITE_MAX_QUANTITY];

static uint16_t Modbus_ReadU16Be(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] << 8U) | data[1];
}

static void Modbus_WriteU16Be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)(value & 0xFFU);
}

static void Modbus_RestartGapTimer(void)
{
    LPIT0->TMR[1].TCTRL = 0U;
    LPIT0->MSR = LPIT_MSR_TIF1(1U);
    LPIT0->TMR[1].TVAL = MODBUS_FRAME_GAP_TICKS - 1U;
    LPIT0->TMR[1].TCTRL = LPIT_TMR_TCTRL_T_EN(1U);
}

static void Modbus_UartCallback(void *driver_state,
                                uart_event_t event,
                                void *user_data)
{
    uint16_t length;

    (void)driver_state;
    (void)user_data;

    if (event == UART_EVENT_RX_FULL)
    {
        length = rx_lengths[active_buffer];
        if (length < MODBUS_RTU_MAX_FRAME)
        {
            rx_buffers[active_buffer][length] = uart_rx_byte;
            rx_lengths[active_buffer] = length + 1U;
        }
        else
        {
            modbus_stats.rx_overflows++;
        }

        Modbus_RestartGapTimer();
        (void)LPUART_DRV_SetRxBuffer(RS485_UART_INSTANCE,
                                     (uint8_t *)&uart_rx_byte,
                                     1U);
    }
    else if (event == UART_EVENT_ERROR)
    {
        modbus_stats.uart_errors++;
        rx_lengths[active_buffer] = 0U;
        LPIT0->TMR[1].TCTRL = 0U;
        (void)LPUART_DRV_ReceiveData(RS485_UART_INSTANCE,
                                    (uint8_t *)&uart_rx_byte,
                                    1U);
    }
}

void LPIT0_Ch1_IRQHandler(void)
{
    uint8_t completed_buffer;

    LPIT0->MSR = LPIT_MSR_TIF1(1U);
    LPIT0->TMR[1].TCTRL = 0U;

    if (rx_lengths[active_buffer] == 0U)
    {
        return;
    }

    if (ready_buffer != MODBUS_NO_READY_BUFFER)
    {
        modbus_stats.frame_drops++;
        rx_lengths[active_buffer] = 0U;
        return;
    }

    completed_buffer = active_buffer;
    active_buffer ^= 1U;
    rx_lengths[active_buffer] = 0U;
    ready_buffer = completed_buffer;
}

uint16_t Modbus_RTU_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t Modbus_AppendCrc(uint8_t *frame, uint16_t length)
{
    uint16_t crc = Modbus_RTU_Crc16(frame, length);

    frame[length] = (uint8_t)(crc & 0xFFU);
    frame[length + 1U] = (uint8_t)(crc >> 8U);
    return length + 2U;
}

static bool Modbus_SendResponse(uint8_t *response, uint16_t length)
{
    length = Modbus_AppendCrc(response, length);
    if (!RS485_PortSend(response, length))
    {
        modbus_stats.tx_errors++;
        return false;
    }

    modbus_stats.tx_frames++;
    return true;
}

static void Modbus_SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];

    response[0] = MODBUS_RTU_SLAVE_ADDRESS;
    response[1] = function | 0x80U;
    response[2] = exception_code;
    modbus_stats.exceptions++;
    (void)Modbus_SendResponse(response, 3U);
}

/* 功能码 0x03：读取 holding register，实际取值由 Modbus_RegRead() 完成。 */
static void Modbus_HandleReadHolding(const uint8_t *request,
                                     uint16_t request_length,
                                     bool broadcast)
{
    uint16_t start;
    uint16_t quantity;
    uint16_t index;
    uint16_t response_length;

    if (broadcast)
    {
        return;
    }

    if (request_length != 8U)
    {
        Modbus_SendException(MODBUS_FUNC_READ_HOLDING,
                             MODBUS_EX_ILLEGAL_VALUE);
        return;
    }

    start = Modbus_ReadU16Be(&request[2]);
    quantity = Modbus_ReadU16Be(&request[4]);
    if ((quantity == 0U) || (quantity > MODBUS_READ_MAX_QUANTITY))
    {
        Modbus_SendException(MODBUS_FUNC_READ_HOLDING,
                             MODBUS_EX_ILLEGAL_VALUE);
        return;
    }

    if (!Modbus_RegRead(start, quantity, read_values))
    {
        Modbus_SendException(MODBUS_FUNC_READ_HOLDING,
                             MODBUS_EX_ILLEGAL_ADDRESS);
        return;
    }

    response_buffer[0] = MODBUS_RTU_SLAVE_ADDRESS;
    response_buffer[1] = MODBUS_FUNC_READ_HOLDING;
    response_buffer[2] = (uint8_t)(quantity * 2U);
    response_length = 3U;
    for (index = 0U; index < quantity; index++)
    {
        Modbus_WriteU16Be(&response_buffer[response_length],
                          read_values[index]);
        response_length += 2U;
    }

    (void)Modbus_SendResponse(response_buffer, response_length);
}

/* 功能码 0x06：写单个寄存器，先检查权限，再交给 Modbus_RegWrite()。 */
static void Modbus_HandleWriteSingle(const uint8_t *request,
                                     uint16_t request_length,
                                     bool broadcast)
{
    uint16_t address;
    uint16_t value;

    if (request_length != 8U)
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_SINGLE,
                                 MODBUS_EX_ILLEGAL_VALUE);
        }
        return;
    }

    address = Modbus_ReadU16Be(&request[2]);
    value = Modbus_ReadU16Be(&request[4]);
    if (!Modbus_RegCanWrite(address, 1U))
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_SINGLE,
                                 MODBUS_EX_ILLEGAL_ADDRESS);
        }
        return;
    }

    if (!Modbus_RegWrite(address, 1U, &value))
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_SINGLE,
                                 MODBUS_EX_SERVER_FAILURE);
        }
        return;
    }

    if (!broadcast)
    {
        (void)memcpy(response_buffer, request, 6U);
        (void)Modbus_SendResponse(response_buffer, 6U);
    }
}

/* 功能码 0x10：写多个寄存器，主要用于批量写映射配置区。 */
static void Modbus_HandleWriteMultiple(const uint8_t *request,
                                       uint16_t request_length,
                                       bool broadcast)
{
    uint16_t start;
    uint16_t quantity;
    uint16_t expected_length;
    uint16_t index;
    uint8_t byte_count;

    if (request_length < 9U)
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_MULTIPLE,
                                 MODBUS_EX_ILLEGAL_VALUE);
        }
        return;
    }

    start = Modbus_ReadU16Be(&request[2]);
    quantity = Modbus_ReadU16Be(&request[4]);
    byte_count = request[6];
    expected_length = (uint16_t)(9U + byte_count);

    if ((quantity == 0U) ||
        (quantity > MODBUS_WRITE_MAX_QUANTITY) ||
        (byte_count != (uint8_t)(quantity * 2U)) ||
        (request_length != expected_length))
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_MULTIPLE,
                                 MODBUS_EX_ILLEGAL_VALUE);
        }
        return;
    }

    if (!Modbus_RegCanWrite(start, quantity))
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_MULTIPLE,
                                 MODBUS_EX_ILLEGAL_ADDRESS);
        }
        return;
    }

    for (index = 0U; index < quantity; index++)
    {
        write_values[index] =
            Modbus_ReadU16Be(&request[7U + (index * 2U)]);
    }

    if (!Modbus_RegWrite(start, quantity, write_values))
    {
        if (!broadcast)
        {
            Modbus_SendException(MODBUS_FUNC_WRITE_MULTIPLE,
                                 MODBUS_EX_SERVER_FAILURE);
        }
        return;
    }

    if (!broadcast)
    {
        response_buffer[0] = MODBUS_RTU_SLAVE_ADDRESS;
        response_buffer[1] = MODBUS_FUNC_WRITE_MULTIPLE;
        Modbus_WriteU16Be(&response_buffer[2], start);
        Modbus_WriteU16Be(&response_buffer[4], quantity);
        (void)Modbus_SendResponse(response_buffer, 6U);
    }
}

static void Modbus_ProcessFrame(const uint8_t *request, uint16_t length)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t address;
    uint8_t function;
    bool broadcast;

    if ((request == NULL) || (length < 4U))
    {
        return;
    }

    received_crc = (uint16_t)request[length - 2U] |
                   (uint16_t)((uint16_t)request[length - 1U] << 8U);
    calculated_crc = Modbus_RTU_Crc16(request, length - 2U);
    if (received_crc != calculated_crc)
    {
        modbus_stats.crc_errors++;
        return;
    }

    address = request[0];
    if ((address != MODBUS_RTU_SLAVE_ADDRESS) && (address != 0U))
    {
        modbus_stats.address_misses++;
        return;
    }

    modbus_stats.rx_frames++;
    broadcast = (address == 0U);
    function = request[1];
    switch (function)
    {
        case MODBUS_FUNC_READ_HOLDING:
            Modbus_HandleReadHolding(request, length, broadcast);
            break;

        case MODBUS_FUNC_WRITE_SINGLE:
            Modbus_HandleWriteSingle(request, length, broadcast);
            break;

        case MODBUS_FUNC_WRITE_MULTIPLE:
            Modbus_HandleWriteMultiple(request, length, broadcast);
            break;

        default:
            if (!broadcast)
            {
                Modbus_SendException(function,
                                     MODBUS_EX_ILLEGAL_FUNCTION);
            }
            break;
    }
}

bool Modbus_RTU_Init(void)
{
    static const uint8_t crc_test_frame[] = {
        0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x0AU
    };

    (void)memset(rx_buffers, 0, sizeof(rx_buffers));
    rx_lengths[0] = 0U;
    rx_lengths[1] = 0U;
    active_buffer = 0U;
    ready_buffer = MODBUS_NO_READY_BUFFER;
    uart_rx_byte = 0U;
    (void)memset(&modbus_stats, 0, sizeof(modbus_stats));
    modbus_stats.crc_self_test_ok =
        (Modbus_RTU_Crc16(crc_test_frame,
                          (uint16_t)sizeof(crc_test_frame)) == 0xCDC5U);

    LPIT0->TMR[1].TCTRL = 0U;
    LPIT0->MSR = LPIT_MSR_TIF1(1U);
    LPIT0->TMR[1].TVAL = MODBUS_FRAME_GAP_TICKS - 1U;
    LPIT0->MIER |= LPIT_MIER_TIE1(1U);
    INT_SYS_EnableIRQ(LPIT0_Ch1_IRQn);

    LPUART_DRV_InstallRxCallback(RS485_UART_INSTANCE,
                                 Modbus_UartCallback,
                                 NULL);
    return (LPUART_DRV_ReceiveData(RS485_UART_INSTANCE,
                                   (uint8_t *)&uart_rx_byte,
                                   1U) == STATUS_SUCCESS);
}

/* 主循环周期调用：从 RS485 字节流中组帧，帧完整后进入 Modbus_ProcessFrame()。 */
void Modbus_RTU_Poll(void)
{
    uint8_t buffer_index;
    uint16_t frame_length;

    INT_SYS_DisableIRQGlobal();
    buffer_index = ready_buffer;
    if (buffer_index != MODBUS_NO_READY_BUFFER)
    {
        frame_length = rx_lengths[buffer_index];
    }
    else
    {
        frame_length = 0U;
    }
    INT_SYS_EnableIRQGlobal();

    if (buffer_index == MODBUS_NO_READY_BUFFER)
    {
        return;
    }

    Modbus_ProcessFrame(rx_buffers[buffer_index], frame_length);

    INT_SYS_DisableIRQGlobal();
    rx_lengths[buffer_index] = 0U;
    ready_buffer = MODBUS_NO_READY_BUFFER;
    INT_SYS_EnableIRQGlobal();
}

const ModbusRtuStats_t *Modbus_RTU_GetStats(void)
{
    return &modbus_stats;
}

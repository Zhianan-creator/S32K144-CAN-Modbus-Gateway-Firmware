#ifndef UART_H_
#define UART_H_

#include "bms_can_protocol.h"
#include "modbus_rtu.h"
#include "status.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * 【API】LPUART1_MonitorInit
 * 调用者：main.c 初始化阶段。
 * 作用：初始化 LPUART1 作为调试监控串口，并输出启动提示。
 * 注意：LPUART1 只做监控打印，不参与 RS485/Modbus 通信。
 */
bool LPUART1_MonitorInit(void);

typedef struct
{
    bool valid;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} MonitorCanRawFrame_t;


/*
 * 【API】LPUART1_MonitorPrintSummary
 * 调用者：main.c 每 10 秒调用一次。
 * 作用：打印一行 SYS 系统摘要，再打印一行最近 CAN 原始帧。
 * 输入 bms：提供 BMS_ON/BMS_VALID。
 * 输入 modbus：提供 MB_RX/MB_TX/CRC_ERR/ADDR_MISS。
 * 输入 last_can_raw：提供 CAN RAW ID/DLC/DATA；如果还没收到 CAN，则打印 CAN RAW none。
 * 设计目的：低频调试输出，快速判断 CAN、Modbus、RS485 是否正常，不恢复逐帧刷屏。
 */
void LPUART1_MonitorPrintSummary(const BmsCanData_t *bms,
                                 const ModbusRtuStats_t *modbus,
                                 const MonitorCanRawFrame_t *last_can_raw,
                                 uint32_t now_ms,
                                 uint32_t can_rx_count,
                                 uint32_t can_rx_overflow_count,
                                 status_t can_rx_rearm_status,
                                 uint32_t can_init_error_count,
                                 status_t can_last_init_error,
                                 bool modbus_init_ok);

#endif /* UART_H_ */

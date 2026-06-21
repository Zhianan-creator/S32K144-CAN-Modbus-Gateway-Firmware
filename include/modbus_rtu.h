#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/* Modbus RTU 从站接口：处理 LPUART2/RS485 上的 0x03、0x06、0x10 功能码。 */

#include <stdbool.h>
#include <stdint.h>

#define MODBUS_RTU_SLAVE_ADDRESS   1U
#define MODBUS_RTU_MAX_FRAME       256U

typedef struct
{
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t crc_errors;
    uint32_t address_misses;
    uint32_t rx_overflows;
    uint32_t frame_drops;
    uint32_t uart_errors;
    uint32_t tx_errors;
    uint32_t exceptions;
    bool crc_self_test_ok;
} ModbusRtuStats_t;

/* 初始化 RTU 接收、帧间隔定时器和协议状态。 */

/*
 * 【API】Modbus_RTU_Init
 * 调用者：main.c 在 RS485_PortInit 成功后调用。
 * 作用：初始化 Modbus RTU 协议状态机和 LPIT 通道1帧间隔计时。
 * 返回值：true=CRC 自检和协议初始化通过；false=初始化失败。
 */
bool Modbus_RTU_Init(void);

/* 在主循环中处理一帧已经接收完整的请求。 */

/*
 * 【API】Modbus_RTU_Poll
 * 调用者：main.c while(1) 每轮调用。
 * 作用：从 LPUART2/RS485 收字节，判断一帧是否结束，完整后解析功能码并发送响应。
 * 注意：它不定义寄存器含义；寄存器读写会转到 modbus_reg.c。
 */
void Modbus_RTU_Poll(void);

/* 计算标准 Modbus CRC16。 */

/*
 * 【API】Modbus_RTU_Crc16
 * 作用：计算标准 Modbus RTU CRC16。
 * 用途：接收时校验请求帧，发送时给响应帧追加 CRC。
 */
uint16_t Modbus_RTU_Crc16(const uint8_t *data, uint16_t length);

/* 获取协议运行统计。 */

/*
 * 【API】Modbus_RTU_GetStats
 * 作用：返回 RTU 通信统计的只读指针。
 * main.c 会把 rx_frames/tx_frames/crc_errors/address_misses 同步到系统寄存器和串口摘要。
 */
const ModbusRtuStats_t *Modbus_RTU_GetStats(void);

/* LPIT 通道1中断入口，由启动向量直接调用。 */

/*
 * 【API】LPIT0_Ch1_IRQHandler
 * 作用：Modbus RTU 帧间隔计时中断入口。
 * 调用方式：由中断向量调用，不应在业务代码中手动调用。
 */
void LPIT0_Ch1_IRQHandler(void);

#endif

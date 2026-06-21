#ifndef RS485_PORT_H
#define RS485_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define RS485_UART_INSTANCE       2U

/* 初始化方向控制脚，默认进入接收状态。 */

/*
 * 【API】RS485_PortInit
 * 调用者：main.c 初始化阶段。
 * 作用：初始化 LPUART2 和 RS485 方向控制 GPIO。
 * 默认状态：初始化后保持接收模式，等待 Modbus 主站请求。
 */
bool RS485_PortInit(void);

/* 发送一帧数据，发送完成后自动恢复接收状态。 */

/*
 * 【API】RS485_PortSend
 * 调用者：modbus_rtu.c 发送响应帧时调用。
 * 作用：切到发送方向，阻塞等待 LPUART2 发送完成，再切回接收方向。
 * 返回值：true=整帧发送完成；false=参数非法、UART 发送失败或等待超时。
 */
bool RS485_PortSend(const uint8_t *data, uint16_t length);

#endif

#ifndef CAN_PORT_H_
#define CAN_PORT_H_

/*
 * CAN0 端口封装：
 * 对外提供初始化、RX 队列弹出、0x300 控制帧发送和状态查询。
 * main.c 不再直接接触 FlexCAN 邮箱和中断细节。
 */

#include "flexcan_driver.h"
#include "status.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * 【API】CAN0_Init
 * 调用者：main.c 初始化阶段。
 * 作用：初始化 FlexCAN0，并把硬件配置成“邮箱0接收、邮箱1发送”。
 * 关键配置：波特率改为 125kbps；邮箱0不限制 CAN ID，用软件层再按映射表过滤；邮箱1用于发送 0x300 控制帧。
 * 成功标志：内部 can0_ready 置 true；失败信息通过 CAN0_GetInitErrorCount/CAN0_GetLastInitError 查询。
 * 注意：这里只负责 CAN 端口收发，不解析 0x180，也不写 Modbus 寄存器。
 */
void CAN0_Init(void);

/*
 * 【API】CAN0_RxQueuePop
 * 调用者：main.c 主循环。
 * 作用：从 CAN 软件队列取出一帧完整报文。
 * 参数 msg：输出参数，成功时填入 CAN ID、DLC 和 data[0..7]。
 * 返回值：true=取到一帧；false=队列为空或参数为空。
 * 为什么需要队列：CAN 中断只负责把硬件收到的帧复制进队列，主循环再慢慢解析，避免中断里做复杂协议处理。
 */
bool CAN0_RxQueuePop(flexcan_msgbuff_t *msg);

/*
 * 【API】CAN0_SendBmsControl
 * 调用者：modbus_reg.c 通过 Modbus_RegSetControlSender 注册后间接调用。
 * 作用：把 Modbus 控制寄存器写入的命令转换成 CAN 0x300 控制帧。
 * 帧格式：CAN ID=0x300，DLC=8，data[0]=command，data[1..7]=0。
 * 常见 command：0x01=请求状态，0x10=DSG_ON，0x11=DSG_OFF。
 * send_status：输出底层 FLEXCAN_DRV_SendBlocking 的状态码，方便记录发送失败原因。
 * 返回值：true=底层发送成功；false=CAN 未就绪或底层发送失败。
 */
bool CAN0_SendBmsControl(uint8_t command, status_t *send_status);


/*
 * 【API】CAN0_IsReady
 * 调用者：OLED 状态页、系统状态判断。
 * 作用：返回 CAN0 初始化是否完成。
 * 注意：ready 只说明 CAN 外设初始化成功，不代表总线上一定有 BMS 报文。BMS 是否在线看 bms->online。
 */
bool CAN0_IsReady(void);

/*
 * 【API】CAN0_GetRxOverflowCount
 * 作用：读取 CAN 软件队列溢出次数。
 * 含义：如果该值增加，说明 CAN 来得太快或主循环处理不及时，可能丢帧。
 * 对应监控输出：SYS 行里的 CAN_OVF，以及 Modbus 系统寄存器 204~205。
 */
uint32_t CAN0_GetRxOverflowCount(void);

/*
 * 【API】CAN0_GetRxRearmStatus
 * 作用：读取最近一次重新挂接 CAN 接收邮箱的状态。
 * 正常值：STATUS_SUCCESS，一般打印为 0。
 * 用途：如果 CAN_RX 不增长，可先看该值是否异常。
 */
status_t CAN0_GetRxRearmStatus(void);

/*
 * 【API】CAN0_GetInitErrorCount
 * 作用：读取 CAN 初始化阶段累计失败次数。
 * 用途：区分“CAN 初始化失败”和“初始化成功但没有收到 BMS 报文”。
 */
uint32_t CAN0_GetInitErrorCount(void);

/*
 * 【API】CAN0_GetLastInitError
 * 作用：读取最近一次 CAN 初始化失败的 SDK 状态码。
 * 用途：配合 CAN0_GetInitErrorCount 定位失败阶段。
 */
status_t CAN0_GetLastInitError(void);

#endif /* CAN_PORT_H_ */

#ifndef MODBUS_REG_H
#define MODBUS_REG_H

/*
 * Modbus 寄存器表接口：
 * 0~7 BMS 数据，100~115 测试/配置命令，200~230 系统统计，
 * 300~356 映射表配置，400~407 BMS 控制命令和发送结果。
 */

#include "bms_can_protocol.h"
#include "can_modbus_map.h"
#include "modbus_rtu.h"
#include "status.h"

#include <stdbool.h>
#include <stdint.h>

#define MODBUS_BMS_REG_START       0U
#define MODBUS_BMS_REG_COUNT       8U
#define MODBUS_TEST_REG_START      100U
#define MODBUS_TEST_REG_COUNT      16U
#define MODBUS_SYS_REG_START       200U
#define MODBUS_SYS_REG_COUNT       31U
#define MODBUS_MAP_CFG_REG_START   300U
#define MODBUS_MAP_CFG_ENTRY_REG_COUNT 7U
#define MODBUS_MAP_CFG_REG_COUNT   (1U + (CAN_MODBUS_MAP_MAX_ENTRIES * MODBUS_MAP_CFG_ENTRY_REG_COUNT))
#define MODBUS_CTRL_REG_START      400U
#define MODBUS_CTRL_REG_COUNT      8U

#define MODBUS_CTRL_CMD_ADDRESS          MODBUS_CTRL_REG_START
#define MODBUS_CTRL_LAST_CMD_ADDRESS     (MODBUS_CTRL_REG_START + 1U)
#define MODBUS_CTRL_LAST_RESULT_ADDRESS  (MODBUS_CTRL_REG_START + 2U)
#define MODBUS_CTRL_LAST_STATUS_ADDRESS  (MODBUS_CTRL_REG_START + 3U)
#define MODBUS_CTRL_TX_OK_LOW_ADDRESS    (MODBUS_CTRL_REG_START + 4U)
#define MODBUS_CTRL_TX_OK_HIGH_ADDRESS   (MODBUS_CTRL_REG_START + 5U)
#define MODBUS_CTRL_TX_FAIL_LOW_ADDRESS  (MODBUS_CTRL_REG_START + 6U)
#define MODBUS_CTRL_TX_FAIL_HIGH_ADDRESS (MODBUS_CTRL_REG_START + 7U)

#define MODBUS_CTRL_RESULT_IDLE          0U
#define MODBUS_CTRL_RESULT_SENT_OK       1U
#define MODBUS_CTRL_RESULT_INVALID_CMD   2U
#define MODBUS_CTRL_RESULT_SEND_FAILED   3U

typedef bool (*Modbus_ControlSender_t)(uint8_t command, status_t *send_status);

/* 初始化寄存器表。 */

/*
 * 【API】Modbus_RegInit
 * 调用者：main.c 初始化阶段。
 * 作用：清空所有 Holding Register 后端数组，包括 BMS 数据区、测试区、系统统计区、映射配置区、控制区。
 * 注意：它只清 RAM 里的寄存器表，不会清 EEPROM 中保存的映射配置。
 */
void Modbus_RegInit(void);

/* 注册 Modbus 控制寄存器触发的 CAN 控制帧发送回调。 */

/*
 * 【API】Modbus_RegSetControlSender
 * 调用者：main.c。
 * 作用：把“发送 CAN 控制帧”的函数指针注册给寄存器层。
 * 当前注册对象：CAN0_SendBmsControl。
 * 后续流程：Modbus 主站写 400 -> Modbus_RegHandleControlCommand -> control_sender -> CAN 0x300。
 */
void Modbus_RegSetControlSender(Modbus_ControlSender_t sender);

/* 将最新 BMS 工程数据同步到只读寄存器。 */

/*
 * 【API】Modbus_RegSyncBms
 * 作用：把 BmsCanData_t 整体同步到 0~7 BMS 数据寄存器。
 * 当前定位：旧式整体同步接口；当前主链路主要由 Modbus_RegWriteBmsMapped 按映射项逐字段写入。
 */
void Modbus_RegSyncBms(const BmsCanData_t *bms);

/*
 * 【API】Modbus_RegWriteBmsMapped
 * 调用者：can_modbus_map.c。
 * 作用：CAN 映射引擎内部写 BMS 数据寄存器 0~7。
 * 参数 address：目标 Modbus 地址，当前只允许 0~7。
 * 参数 value：已经按映射类型解析好的 16bit 值。
 * 返回值：true=写入成功；false=地址越界。
 * 注意：这个接口不是给外部 Modbus 主站写的，外部写权限由 Modbus_RegCanWrite 控制。
 */
bool Modbus_RegWriteBmsMapped(uint16_t address, uint16_t value);

/* 将系统运行状态同步到只读统计寄存器。 */

/*
 * 【API】Modbus_RegSyncSystemStats
 * 调用者：main.c 初始化后和主循环中持续调用。
 * 作用：把网关运行状态同步到系统统计寄存器 200~219。
 * 内容包括：运行秒数、CAN_RX、CAN_OVF、CAN_REARM、BMS_ON、BMS_VALID、BMS 帧年龄、MB_RX、MB_TX、CRC_ERR、ADDR_MISS、RS485_OK。
 * 用途：Modbus Poll 读取 200~219 可以判断网关是否健康。
 */
void Modbus_RegSyncSystemStats(const BmsCanData_t *bms,
                               const ModbusRtuStats_t *modbus,
                               uint32_t now_ms,
                               uint32_t can_rx_count,
                               uint32_t can_rx_overflow_count,
                               status_t can_rx_rearm_status,
                               uint32_t can_init_error_count,
                               status_t can_last_init_error,
                               bool modbus_init_ok);

/*
 * 【API】Modbus_RegSyncMapConfigStats
 * 作用：把映射配置/EEPROM 相关状态同步到系统寄存器 220~230。
 * 例如 EEPROM 是否存在、当前配置来源、加载/保存结果、CRC 错误、配置错误位置。
 */
void Modbus_RegSyncMapConfigStats(const CanModbusConfigStats_t *config);

/*
 * 【API】Modbus_RegSyncMapConfigTable
 * 作用：把当前生效的映射表展开到 300~356 配置寄存器。
 * 用途：Modbus 主站先读/同步 300 段，再修改其中字段，最后写命令 APPLY 或 APPLY_SAVE。
 */
void Modbus_RegSyncMapConfigTable(void);

/* 检查并读取一段连续保持寄存器。 */

/*
 * 【API】Modbus_RegRead
 * 调用者：modbus_rtu.c 处理 0x03 功能码时调用。
 * 作用：根据 start/quantity 从对应寄存器段拷贝数据给 Modbus 响应。
 * 返回值：true=地址范围合法并已填充 values；false=地址越界或 quantity 为 0。
 */
bool Modbus_RegRead(uint16_t start, uint16_t quantity, uint16_t *values);

/* 检查一段寄存器是否全部允许写入。 */

/*
 * 【API】Modbus_RegCanWrite
 * 调用者：modbus_rtu.c 写寄存器前先调用。
 * 作用：判断外部 Modbus 主站是否允许写某段 Holding Register。
 * 当前允许：100~115 测试/配置命令区，300~356 映射配置区，400 单个控制命令入口。
 * 当前禁止：0~7 BMS 实时数据区和 200~230 系统统计区，避免主站伪造状态。
 */
bool Modbus_RegCanWrite(uint16_t start, uint16_t quantity);

/* 写入单个或多个 RAM 测试寄存器。 */
bool Modbus_RegWrite(uint16_t start, uint16_t quantity, const uint16_t *values);

#endif

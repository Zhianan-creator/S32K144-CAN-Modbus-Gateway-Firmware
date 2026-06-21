#ifndef CAN_MODBUS_MAP_H
#define CAN_MODBUS_MAP_H

/*
 * CAN 到 Modbus 映射引擎接口：
 * 映射项描述“哪个 CAN ID 的哪个字节，按什么类型，写到哪个 Modbus 寄存器”。
 */

#include "bms_can_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define CAN_MODBUS_MAP_MAX_ENTRIES 8U

typedef enum
{
    CAN_MODBUS_TYPE_U8 = 0,
    CAN_MODBUS_TYPE_S8,
    CAN_MODBUS_TYPE_U16_LE,
    CAN_MODBUS_TYPE_S16_LE
} CanModbusMapDataType_t;

/* 单条映射规则：运行时配置区和 EEPROM 保存的核心结构。 */
typedef struct
{
    uint32_t can_id;
    uint8_t required_dlc;
    uint8_t byte_offset;
    CanModbusMapDataType_t data_type;
    uint16_t modbus_address;
    bool enabled;
} CanModbusMapEntry_t;

typedef struct
{
    uint32_t matched_frames;
    uint32_t mapped_values;
    uint32_t unknown_frames;
    uint32_t dlc_errors;
    uint32_t write_errors;
} CanModbusMapStats_t;

typedef struct
{
    bool eeprom_present;
    uint16_t active_source;
    uint16_t load_result;
    uint16_t save_result;
    uint16_t command_result;
    uint32_t crc_errors;
    uint32_t storage_errors;
    uint16_t config_error_index;
    uint16_t config_error_field;
} CanModbusConfigStats_t;

#define CAN_MODBUS_CFG_SOURCE_DEFAULT  0U
#define CAN_MODBUS_CFG_SOURCE_EEPROM   1U
#define CAN_MODBUS_CFG_SOURCE_RUNTIME  2U

#define CAN_MODBUS_CFG_RESULT_IDLE     0U
#define CAN_MODBUS_CFG_RESULT_OK       1U
#define CAN_MODBUS_CFG_RESULT_NO_DEV   2U
#define CAN_MODBUS_CFG_RESULT_BAD_CRC  3U
#define CAN_MODBUS_CFG_RESULT_BAD_DATA 4U
#define CAN_MODBUS_CFG_RESULT_IO_FAIL  5U
#define CAN_MODBUS_CFG_RESULT_BAD_CMD  6U

#define CAN_MODBUS_CFG_ERROR_NONE      0U
#define CAN_MODBUS_CFG_ERROR_COUNT     1U
#define CAN_MODBUS_CFG_ERROR_CAN_ID    2U
#define CAN_MODBUS_CFG_ERROR_DLC       3U
#define CAN_MODBUS_CFG_ERROR_OFFSET    4U
#define CAN_MODBUS_CFG_ERROR_TYPE      5U
#define CAN_MODBUS_CFG_ERROR_ADDRESS   6U
#define CAN_MODBUS_CFG_ERROR_ENABLED   7U

#define CAN_MODBUS_CFG_CMD_NONE        0U
#define CAN_MODBUS_CFG_CMD_DEFAULTS    1U
#define CAN_MODBUS_CFG_CMD_SAVE        2U
#define CAN_MODBUS_CFG_CMD_LOAD        3U
#define CAN_MODBUS_CFG_CMD_SYNC_REGS   4U
#define CAN_MODBUS_CFG_CMD_APPLY       5U
#define CAN_MODBUS_CFG_CMD_APPLY_SAVE  6U


/*
 * 【API】CanModbusMap_Init
 * 调用者：main.c 初始化阶段。
 * 作用：初始化 CAN->Modbus 映射引擎。
 * 启动逻辑：先加载默认映射表，再尝试从 EEPROM 读取用户保存的映射；EEPROM 无效时回退默认表。
 * 同时会初始化 bms 缓存，并把 online/valid 写入 Modbus 6/7。
 */
void CanModbusMap_Init(BmsCanData_t *bms);


/*
 * 【API】CanModbusMap_ProcessFrame
 * 调用者：main.c 从 CAN 队列取到一帧后调用。
 * 作用：执行真正的 CAN 到 Modbus 字段映射。
 * 流程：遍历当前映射表 -> 匹配 CAN ID -> 检查 DLC -> 按 byte_offset/data_type 提取数值 -> 写入目标 Modbus 寄存器。
 * 成功效果：更新 bms 缓存、Modbus 0~7 数据区、映射统计 matched_frames/mapped_values。
 * 返回值：true=至少有一个字段成功映射；false=未知 ID、DLC 不匹配或写寄存器失败。
 */
bool CanModbusMap_ProcessFrame(BmsCanData_t *bms,
                               uint32_t can_id,
                               uint8_t dlc,
                               const uint8_t data[8],
                               uint32_t now_ms);


/*
 * 【API】CanModbusMap_CheckOffline
 * 调用者：main.c 主循环。
 * 作用：周期检查 BMS 是否离线，并把 online/valid 变化同步到 Modbus 6/7。
 * 注意：它不接收 CAN 帧，只根据最近一次有效 CAN 帧时间判断超时。
 */
bool CanModbusMap_CheckOffline(BmsCanData_t *bms, uint32_t now_ms);


/*
 * 【API】CanModbusMap_GetStats
 * 作用：返回映射运行统计的只读指针。
 * 统计内容：匹配帧数、成功映射字段数、未知帧数、DLC 错误数、写寄存器错误数。
 */
const CanModbusMapStats_t *CanModbusMap_GetStats(void);

/*
 * 【API】CanModbusMap_GetConfigStats
 * 作用：返回映射配置/EEPROM 状态的只读指针。
 * main.c 会把这些状态同步到 Modbus 系统寄存器 220~230。
 */
const CanModbusConfigStats_t *CanModbusMap_GetConfigStats(void);

/*
 * 【API】CanModbusMap_GetEntryCount
 * 作用：返回当前生效的映射项数量。
 * 用途：Modbus_RegSyncMapConfigTable 用它把当前映射表同步到 300 段配置寄存器。
 */
uint16_t CanModbusMap_GetEntryCount(void);
bool CanModbusMap_GetEntry(uint16_t index, CanModbusMapEntry_t *entry);

/*
 * 【API】CanModbusMap_ApplyEntries
 * 调用者：modbus_reg.c 在执行配置命令 APPLY/APPLY_SAVE 时调用。
 * 作用：把 300 段寄存器里拼出的映射项应用为当前运行映射表。
 * 安全策略：先逐条校验 CAN ID、DLC、偏移、数据类型、目标地址、enabled，全部合法后才替换。
 * 返回值：CAN_MODBUS_CFG_RESULT_OK 或 BAD_DATA 等配置结果码。
 */
uint16_t CanModbusMap_ApplyEntries(const CanModbusMapEntry_t *entries,
                                   uint16_t entry_count);

/*
 * 【API】CanModbusMap_RecordConfigResult
 * 作用：记录最近一次配置命令结果、出错条目和出错字段。
 * 用途：让 Modbus 系统寄存器能显示“为什么应用映射失败”。
 */
void CanModbusMap_RecordConfigResult(uint16_t result,
                                     uint16_t error_index,
                                     uint16_t error_field);

/*
 * 【API】CanModbusMap_HandleConfigCommand
 * 作用：处理映射配置命令。
 * 支持命令：DEFAULTS=恢复默认表，SAVE=保存 EEPROM，LOAD=从 EEPROM 读取，SYNC_REGS=仅同步寄存器显示。
 * APPLY/APPLY_SAVE 需要读取 300 段寄存器，因此由 modbus_reg.c 先组表后再调用 ApplyEntries。
 */
uint16_t CanModbusMap_HandleConfigCommand(uint16_t command);

#endif

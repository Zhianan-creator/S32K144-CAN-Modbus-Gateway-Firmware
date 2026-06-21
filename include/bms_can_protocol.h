#ifndef BMS_CAN_PROTOCOL_H
#define BMS_CAN_PROTOCOL_H

/*
 * BMS CAN 协议定义：
 * 0x180 是 FREE-BMS 周期状态帧，0x300 是网关发给 BMS 的控制帧。
 * 当前主链路主要由 can_modbus_map.c 按映射表解析 0x180。
 */

#include <stdbool.h>
#include <stdint.h>

/* FREE-BMS 周期状态报文及在线检测参数。 */
#define BMS_CAN_CMD_ID             0x300U
#define BMS_CAN_STATUS_ID          0x180U
#define BMS_CAN_STATUS_DLC         8U
#define BMS_CAN_OFFLINE_TIMEOUT_MS 500U
#define BMS_CAN_TX_TIMEOUT_MS      10U

/* FREE-BMS 0x300 控制命令 data[0]。 */
#define BMS_CAN_CMD_STATUS_REQ     0x01U
#define BMS_CAN_CMD_DSG_ON         0x10U
#define BMS_CAN_CMD_DSG_OFF        0x11U

/* 状态字节 data[5] 的位定义。 */
#define BMS_CAN_STATE_MON_VALID    0x01U
#define BMS_CAN_STATE_CHG_ALLOWED  0x02U
#define BMS_CAN_STATE_DSG_ALLOWED  0x04U
#define BMS_CAN_STATE_CHG_ON       0x08U
#define BMS_CAN_STATE_DSG_ON       0x10U
#define BMS_CAN_STATE_BAL_ACTIVE   0x20U
#define BMS_CAN_STATE_SLEEP_REQ    0x40U

/* 故障字节 data[6] 的位定义。 */
#define BMS_CAN_FAULT_OV           0x01U
#define BMS_CAN_FAULT_UV           0x02U
#define BMS_CAN_FAULT_OCD          0x04U
#define BMS_CAN_FAULT_SCD          0x08U
#define BMS_CAN_FAULT_ANY          0x10U
#define BMS_CAN_FAULT_MON_INVALID  0x20U

/* CAN 原始报文解析后的电池工程数据。 */
typedef struct
{
    uint16_t voltage_cv;       /* 总压，单位 0.01V。 */
    int16_t current_ca;        /* 电流，单位 0.01A，负值表示放电。 */
    uint8_t soc_percent;       /* 剩余电量，单位百分比。 */
    int8_t temperature_c;      /* 第一温度通道，单位摄氏度。 */
    uint8_t state_flags;       /* BMS 状态位。 */
    uint8_t fault_flags;       /* BMS 故障位。 */
    uint32_t last_rx_ms;       /* 最近一次有效状态报文的接收时间。 */
    bool online;               /* CAN 通信在线标志。 */
    bool valid;                /* 电池测量数据有效标志。 */
} BmsCanData_t;

/* 初始化协议解析结果。 */

/*
 * 【API】BMS_CAN_ProtocolInit
 * 调用者：CanModbusMap_Init。
 * 作用：清空 BmsCanData_t 缓存，避免启动时 OLED/Modbus 显示旧值。
 * 初始化结果：online=false，valid=false，电压/电流/SOC/温度/状态/故障均清零。
 */
void BMS_CAN_ProtocolInit(BmsCanData_t *bms);

/* 检查并解析一帧 0x180 状态报文，成功时更新工程数据和接收时间。 */

/*
 * 【API】BMS_CAN_ParseStatus
 * 作用：固定解析 FREE-BMS 的 0x180 状态帧。
 * 当前定位：保留的协议解析函数；主链路更多使用 CanModbusMap_ProcessFrame 的可配置映射。
 * 输入：can_id 必须为 0x180，dlc 必须为 8，data 按固定字节布局解析。
 * 输出：成功时更新 bms 工程量、last_rx_ms、online、valid。
 * 返回值：true=这是一帧合法状态帧并解析成功；false=ID/DLC/参数不符合。
 */
bool BMS_CAN_ParseStatus(BmsCanData_t *bms,
                         uint32_t can_id,
                         uint8_t dlc,
                         const uint8_t data[8],
                         uint32_t now_ms);

/* 检查接收超时；刚从在线变为离线时返回 true。 */

/*
 * 【API】BMS_CAN_CheckOffline
 * 调用者：CanModbusMap_CheckOffline。
 * 作用：根据 now_ms 与 bms->last_rx_ms 的差值判断 BMS 是否超时离线。
 * 超时阈值：BMS_CAN_OFFLINE_TIMEOUT_MS，目前为 500ms。
 * 返回值：true=本次调用导致 online/valid 状态发生变化；false=状态没有变化。
 */
bool BMS_CAN_CheckOffline(BmsCanData_t *bms, uint32_t now_ms);

#endif

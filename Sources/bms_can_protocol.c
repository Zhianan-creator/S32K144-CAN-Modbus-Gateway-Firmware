#include "bms_can_protocol.h"

#include <stddef.h>
#include <string.h>

/*
 * 阅读提示：
 * 这里保留 BMS 状态帧的固定解析方式和在线超时判断。
 * 当前项目的可配置映射主链路在 CanModbusMap_ProcessFrame() 中。
 */

/* 清空全部工程数据，系统启动时默认处于离线状态。 */
void BMS_CAN_ProtocolInit(BmsCanData_t *bms)
{
    if (bms != NULL)
    {
        (void)memset(bms, 0, sizeof(*bms));
    }
}

/* 固定协议解析函数：按 0x180 的原始字节填充 BmsCanData_t。 */
bool BMS_CAN_ParseStatus(BmsCanData_t *bms,
                         uint32_t can_id,
                         uint8_t dlc,
                         const uint8_t data[8],
                         uint32_t now_ms)
{
    uint16_t current_raw;

    /* 只接受 FREE-BMS 定义的标准状态报文。 */
    if ((bms == NULL) || (data == NULL) ||
        (can_id != BMS_CAN_STATUS_ID) || (dlc != BMS_CAN_STATUS_DLC))
    {
        return false;
    }

    /* 总压和电流均为低字节在前的小端格式。 */
    bms->voltage_cv = (uint16_t)data[0] |
                      (uint16_t)((uint16_t)data[1] << 8U);

    current_raw = (uint16_t)data[2] |
                  (uint16_t)((uint16_t)data[3] << 8U);
    bms->current_ca = (int16_t)current_raw;

    bms->soc_percent = (data[4] > 100U) ? 100U : data[4];
    bms->state_flags = data[5];
    bms->fault_flags = data[6];
    bms->temperature_c = (int8_t)data[7];
    bms->last_rx_ms = now_ms;
    bms->online = true;
    bms->valid = ((bms->state_flags & BMS_CAN_STATE_MON_VALID) != 0U) &&
                 ((bms->fault_flags & BMS_CAN_FAULT_MON_INVALID) == 0U);

    return true;
}

/* 使用无符号减法，可正确处理毫秒计数器回绕。 */
/* 在线检测：超过 BMS_CAN_OFFLINE_TIMEOUT_MS 未收到状态帧，就清 online/valid。 */
bool BMS_CAN_CheckOffline(BmsCanData_t *bms, uint32_t now_ms)
{
    if ((bms != NULL) && bms->online &&
        ((uint32_t)(now_ms - bms->last_rx_ms) > BMS_CAN_OFFLINE_TIMEOUT_MS))
    {
        bms->online = false;
        bms->valid = false;
        return true;
    }

    return false;
}

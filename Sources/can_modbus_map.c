#include "can_modbus_map.h"

#include "eeprom_24c64.h"
#include "modbus_reg.h"

#include <stddef.h>
#include <string.h>

#define CAN_MODBUS_REG_VOLTAGE    0U
#define CAN_MODBUS_REG_CURRENT    1U
#define CAN_MODBUS_REG_SOC        2U
#define CAN_MODBUS_REG_TEMP       3U
#define CAN_MODBUS_REG_STATE      4U
#define CAN_MODBUS_REG_FAULT      5U
#define CAN_MODBUS_REG_ONLINE     6U
#define CAN_MODBUS_REG_VALID      7U

/*
 * 阅读提示：
 * 本文件是网关最核心的 CAN -> Modbus 转换层。
 * 默认表先给出 0x180 状态帧到寄存器 0~5 的映射，6/7 用于 online/valid。
 */


#define CAN_MODBUS_CONFIG_EEPROM_ADDR    0x0000U
#define CAN_MODBUS_CONFIG_MAGIC          0x434D4150UL
#define CAN_MODBUS_CONFIG_VERSION        1U
#define CAN_MODBUS_CONFIG_HEADER_SIZE    8U
#define CAN_MODBUS_CONFIG_ENTRY_SIZE     10U
#define CAN_MODBUS_CONFIG_CRC_SIZE       2U
#define CAN_MODBUS_CONFIG_BUFFER_SIZE    (CAN_MODBUS_CONFIG_HEADER_SIZE + \
                                          (CAN_MODBUS_MAP_MAX_ENTRIES * CAN_MODBUS_CONFIG_ENTRY_SIZE) + \
                                          CAN_MODBUS_CONFIG_CRC_SIZE)

/* 默认映射表：没有 EEPROM 配置或恢复默认时使用。 */
static const CanModbusMapEntry_t can_modbus_default_map_table[] = {
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 0U, CAN_MODBUS_TYPE_U16_LE, CAN_MODBUS_REG_VOLTAGE, true },
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 2U, CAN_MODBUS_TYPE_S16_LE, CAN_MODBUS_REG_CURRENT, true },
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 4U, CAN_MODBUS_TYPE_U8,     CAN_MODBUS_REG_SOC,     true },
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 7U, CAN_MODBUS_TYPE_S8,     CAN_MODBUS_REG_TEMP,    true },
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 5U, CAN_MODBUS_TYPE_U8,     CAN_MODBUS_REG_STATE,   true },
    { BMS_CAN_STATUS_ID, BMS_CAN_STATUS_DLC, 6U, CAN_MODBUS_TYPE_U8,     CAN_MODBUS_REG_FAULT,   true }
};

static CanModbusMapEntry_t can_modbus_map_table[CAN_MODBUS_MAP_MAX_ENTRIES];
static uint16_t can_modbus_map_count;
static CanModbusMapStats_t map_stats;
static CanModbusConfigStats_t config_stats;

static uint16_t CanModbusMap_Crc16(const uint8_t *data, uint16_t length)
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

static void CanModbusMap_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t CanModbusMap_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8U);
}

static void CanModbusMap_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t CanModbusMap_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           (uint32_t)((uint32_t)data[1] << 8U) |
           (uint32_t)((uint32_t)data[2] << 16U) |
           (uint32_t)((uint32_t)data[3] << 24U);
}

static void CanModbusMap_LoadDefaults(void)
{
    can_modbus_map_count =
        (uint16_t)(sizeof(can_modbus_default_map_table) /
                   sizeof(can_modbus_default_map_table[0]));
    (void)memcpy(can_modbus_map_table,
                 can_modbus_default_map_table,
                 sizeof(can_modbus_default_map_table));
    config_stats.active_source = CAN_MODBUS_CFG_SOURCE_DEFAULT;
}

static bool CanModbusMap_ValidateEntry(const CanModbusMapEntry_t *entry,
                                       uint16_t *error_field)
{
    uint16_t offset;

    if (error_field != NULL)
    {
        *error_field = CAN_MODBUS_CFG_ERROR_NONE;
    }

    if (entry == NULL)
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_COUNT;
        }
        return false;
    }

    if (entry->can_id > 0x7FFU)
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_CAN_ID;
        }
        return false;
    }

    if ((entry->required_dlc == 0U) || (entry->required_dlc > 8U))
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_DLC;
        }
        return false;
    }

    if (entry->data_type > CAN_MODBUS_TYPE_S16_LE)
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_TYPE;
        }
        return false;
    }

    if (entry->modbus_address >= CAN_MODBUS_REG_ONLINE)
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_ADDRESS;
        }
        return false;
    }

    if ((entry->enabled != false) && (entry->enabled != true))
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_ENABLED;
        }
        return false;
    }

    offset = entry->byte_offset;
    if (offset >= entry->required_dlc)
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_OFFSET;
        }
        return false;
    }

    if (((entry->data_type == CAN_MODBUS_TYPE_U16_LE) ||
         (entry->data_type == CAN_MODBUS_TYPE_S16_LE)) &&
        ((offset + 1U) >= entry->required_dlc))
    {
        if (error_field != NULL)
        {
            *error_field = CAN_MODBUS_CFG_ERROR_OFFSET;
        }
        return false;
    }

    return true;
}

static bool CanModbusMap_IsEntryValid(const CanModbusMapEntry_t *entry)
{
    return CanModbusMap_ValidateEntry(entry, NULL);
}

/* 从 24C64 EEPROM 读取映射表，校验 magic/version/count/CRC 后才生效。 */
static uint16_t CanModbusMap_LoadFromEeprom(void)
{
    uint8_t buffer[CAN_MODBUS_CONFIG_BUFFER_SIZE];
    CanModbusMapEntry_t loaded[CAN_MODBUS_MAP_MAX_ENTRIES];
    uint32_t magic;
    uint16_t entry_count;
    uint16_t payload_len;
    uint16_t stored_crc;
    uint16_t calc_crc;
    uint16_t index;
    uint16_t pos;

    if (!config_stats.eeprom_present)
    {
        config_stats.load_result = CAN_MODBUS_CFG_RESULT_NO_DEV;
        return config_stats.load_result;
    }

    if (!EEPROM24C64_Read(CAN_MODBUS_CONFIG_EEPROM_ADDR,
                          buffer,
                          sizeof(buffer)))
    {
        config_stats.storage_errors++;
        config_stats.load_result = CAN_MODBUS_CFG_RESULT_IO_FAIL;
        return config_stats.load_result;
    }

    magic = CanModbusMap_ReadU32Le(&buffer[0]);
    entry_count = CanModbusMap_ReadU16Le(&buffer[6]);
    if ((magic != CAN_MODBUS_CONFIG_MAGIC) ||
        (buffer[4] != CAN_MODBUS_CONFIG_VERSION) ||
        (entry_count == 0U) ||
        (entry_count > CAN_MODBUS_MAP_MAX_ENTRIES))
    {
        config_stats.load_result = CAN_MODBUS_CFG_RESULT_BAD_DATA;
        return config_stats.load_result;
    }

    payload_len = (uint16_t)(CAN_MODBUS_CONFIG_HEADER_SIZE +
                  (entry_count * CAN_MODBUS_CONFIG_ENTRY_SIZE));
    stored_crc = CanModbusMap_ReadU16Le(&buffer[payload_len]);
    calc_crc = CanModbusMap_Crc16(buffer, payload_len);
    if (stored_crc != calc_crc)
    {
        config_stats.crc_errors++;
        config_stats.load_result = CAN_MODBUS_CFG_RESULT_BAD_CRC;
        return config_stats.load_result;
    }

    pos = CAN_MODBUS_CONFIG_HEADER_SIZE;
    for (index = 0U; index < entry_count; index++)
    {
        loaded[index].can_id = CanModbusMap_ReadU32Le(&buffer[pos]);
        loaded[index].required_dlc = buffer[pos + 4U];
        loaded[index].byte_offset = buffer[pos + 5U];
        loaded[index].data_type = (CanModbusMapDataType_t)buffer[pos + 6U];
        loaded[index].modbus_address = CanModbusMap_ReadU16Le(&buffer[pos + 7U]);
        loaded[index].enabled = (buffer[pos + 9U] != 0U);
        if (!CanModbusMap_IsEntryValid(&loaded[index]))
        {
            config_stats.load_result = CAN_MODBUS_CFG_RESULT_BAD_DATA;
            return config_stats.load_result;
        }
        pos = (uint16_t)(pos + CAN_MODBUS_CONFIG_ENTRY_SIZE);
    }

    (void)memcpy(can_modbus_map_table,
                 loaded,
                 (uint32_t)entry_count * sizeof(CanModbusMapEntry_t));
    can_modbus_map_count = entry_count;
    config_stats.active_source = CAN_MODBUS_CFG_SOURCE_EEPROM;
    config_stats.load_result = CAN_MODBUS_CFG_RESULT_OK;
    return config_stats.load_result;
}

/* 将当前运行中的映射表打包并写入 EEPROM，实现掉电保持。 */
static uint16_t CanModbusMap_SaveToEeprom(void)
{
    uint8_t buffer[CAN_MODBUS_CONFIG_BUFFER_SIZE];
    uint16_t payload_len;
    uint16_t crc;
    uint16_t index;
    uint16_t pos;

    if (!config_stats.eeprom_present)
    {
        config_stats.save_result = CAN_MODBUS_CFG_RESULT_NO_DEV;
        return config_stats.save_result;
    }

    (void)memset(buffer, 0xFF, sizeof(buffer));
    CanModbusMap_WriteU32Le(&buffer[0], CAN_MODBUS_CONFIG_MAGIC);
    buffer[4] = CAN_MODBUS_CONFIG_VERSION;
    buffer[5] = 0U;
    CanModbusMap_WriteU16Le(&buffer[6], can_modbus_map_count);

    pos = CAN_MODBUS_CONFIG_HEADER_SIZE;
    for (index = 0U; index < can_modbus_map_count; index++)
    {
        const CanModbusMapEntry_t *entry = &can_modbus_map_table[index];

        if (!CanModbusMap_IsEntryValid(entry))
        {
            config_stats.save_result = CAN_MODBUS_CFG_RESULT_BAD_DATA;
            return config_stats.save_result;
        }

        CanModbusMap_WriteU32Le(&buffer[pos], entry->can_id);
        buffer[pos + 4U] = entry->required_dlc;
        buffer[pos + 5U] = entry->byte_offset;
        buffer[pos + 6U] = (uint8_t)entry->data_type;
        CanModbusMap_WriteU16Le(&buffer[pos + 7U], entry->modbus_address);
        buffer[pos + 9U] = entry->enabled ? 1U : 0U;
        pos = (uint16_t)(pos + CAN_MODBUS_CONFIG_ENTRY_SIZE);
    }

    payload_len = pos;
    crc = CanModbusMap_Crc16(buffer, payload_len);
    CanModbusMap_WriteU16Le(&buffer[payload_len], crc);

    if (!EEPROM24C64_Write(CAN_MODBUS_CONFIG_EEPROM_ADDR,
                           buffer,
                           (uint16_t)(payload_len + CAN_MODBUS_CONFIG_CRC_SIZE)))
    {
        config_stats.storage_errors++;
        config_stats.save_result = CAN_MODBUS_CFG_RESULT_IO_FAIL;
        return config_stats.save_result;
    }

    config_stats.save_result = CAN_MODBUS_CFG_RESULT_OK;
    return config_stats.save_result;
}

/* 按映射项的数据类型和字节偏移，从 CAN data[] 中提取 16bit Modbus 值。 */
static bool CanModbusMap_ExtractValue(const CanModbusMapEntry_t *entry,
                                      const uint8_t *data,
                                      uint8_t dlc,
                                      uint16_t *value)
{
    uint8_t offset;
    uint16_t raw_u16;

    if ((entry == NULL) || (data == NULL) || (value == NULL))
    {
        return false;
    }

    offset = entry->byte_offset;
    switch (entry->data_type)
    {
        case CAN_MODBUS_TYPE_U8:
            if (offset >= dlc)
            {
                return false;
            }
            *value = data[offset];
            return true;

        case CAN_MODBUS_TYPE_S8:
            if (offset >= dlc)
            {
                return false;
            }
            *value = (uint16_t)((int16_t)((int8_t)data[offset]));
            return true;

        case CAN_MODBUS_TYPE_U16_LE:
            if (((uint16_t)offset + 1U) >= dlc)
            {
                return false;
            }
            *value = (uint16_t)data[offset] |
                     (uint16_t)((uint16_t)data[offset + 1U] << 8U);
            return true;

        case CAN_MODBUS_TYPE_S16_LE:
            if (((uint16_t)offset + 1U) >= dlc)
            {
                return false;
            }
            raw_u16 = (uint16_t)data[offset] |
                      (uint16_t)((uint16_t)data[offset + 1U] << 8U);
            *value = (uint16_t)((int16_t)raw_u16);
            return true;

        default:
            return false;
    }
}

static void CanModbusMap_UpdateBmsCache(BmsCanData_t *bms,
                                        uint16_t address,
                                        uint16_t value)
{
    if (bms == NULL)
    {
        return;
    }

    switch (address)
    {
        case CAN_MODBUS_REG_VOLTAGE:
            bms->voltage_cv = value;
            break;

        case CAN_MODBUS_REG_CURRENT:
            bms->current_ca = (int16_t)value;
            break;

        case CAN_MODBUS_REG_SOC:
            bms->soc_percent = (value > 100U) ? 100U : (uint8_t)value;
            break;

        case CAN_MODBUS_REG_TEMP:
            bms->temperature_c = (int8_t)((int16_t)value);
            break;

        case CAN_MODBUS_REG_STATE:
            bms->state_flags = (uint8_t)value;
            break;

        case CAN_MODBUS_REG_FAULT:
            bms->fault_flags = (uint8_t)value;
            break;

        default:
            break;
    }
}

static void CanModbusMap_UpdateStatusRegs(const BmsCanData_t *bms)
{
    bool write_ok;
    uint16_t online = 0U;
    uint16_t valid = 0U;

    if (bms != NULL)
    {
        online = bms->online ? 1U : 0U;
        valid = bms->valid ? 1U : 0U;
    }

    write_ok = Modbus_RegWriteBmsMapped(CAN_MODBUS_REG_ONLINE, online);
    write_ok = Modbus_RegWriteBmsMapped(CAN_MODBUS_REG_VALID, valid) && write_ok;
    if (!write_ok)
    {
        map_stats.write_errors++;
    }
}

void CanModbusMap_Init(BmsCanData_t *bms)
{
    (void)memset(&map_stats, 0, sizeof(map_stats));
    (void)memset(&config_stats, 0, sizeof(config_stats));
    CanModbusMap_LoadDefaults();
    config_stats.eeprom_present = EEPROM24C64_Init();
    if (CanModbusMap_LoadFromEeprom() != CAN_MODBUS_CFG_RESULT_OK)
    {
        CanModbusMap_LoadDefaults();
    }
    BMS_CAN_ProtocolInit(bms);
    CanModbusMap_UpdateStatusRegs(bms);
}

/* 主转换入口：匹配 CAN ID/DLC，提取字段，写入 Modbus BMS 寄存器。 */
bool CanModbusMap_ProcessFrame(BmsCanData_t *bms,
                               uint32_t can_id,
                               uint8_t dlc,
                               const uint8_t data[8],
                               uint32_t now_ms)
{
    uint16_t index;
    uint16_t value;
    bool saw_id = false;
    bool mapped_any = false;
    bool write_ok;

    if ((bms == NULL) || (data == NULL))
    {
        return false;
    }

    for (index = 0U; index < can_modbus_map_count; index++)
    {
        const CanModbusMapEntry_t *entry = &can_modbus_map_table[index];

        if ((!entry->enabled) || (entry->can_id != can_id))
        {
            continue;
        }

        saw_id = true;
        if (dlc != entry->required_dlc)
        {
            continue;
        }

        if (!CanModbusMap_ExtractValue(entry, data, dlc, &value))
        {
            map_stats.dlc_errors++;
            continue;
        }

        write_ok = Modbus_RegWriteBmsMapped(entry->modbus_address, value);
        if (!write_ok)
        {
            map_stats.write_errors++;
            continue;
        }

        CanModbusMap_UpdateBmsCache(bms, entry->modbus_address, value);
        map_stats.mapped_values++;
        mapped_any = true;
    }

    if (!saw_id)
    {
        map_stats.unknown_frames++;
        return false;
    }

    if (!mapped_any)
    {
        map_stats.dlc_errors++;
        return false;
    }

    bms->last_rx_ms = now_ms;
    bms->online = true;
    bms->valid = ((bms->state_flags & BMS_CAN_STATE_MON_VALID) != 0U) &&
                 ((bms->fault_flags & BMS_CAN_FAULT_MON_INVALID) == 0U);
    CanModbusMap_UpdateStatusRegs(bms);
    map_stats.matched_frames++;

    return true;
}

bool CanModbusMap_CheckOffline(BmsCanData_t *bms, uint32_t now_ms)
{
    bool changed = BMS_CAN_CheckOffline(bms, now_ms);

    if (changed)
    {
        CanModbusMap_UpdateStatusRegs(bms);
    }

    return changed;
}

const CanModbusMapStats_t *CanModbusMap_GetStats(void)
{
    return &map_stats;
}

const CanModbusConfigStats_t *CanModbusMap_GetConfigStats(void)
{
    return &config_stats;
}

uint16_t CanModbusMap_GetEntryCount(void)
{
    return can_modbus_map_count;
}

bool CanModbusMap_GetEntry(uint16_t index, CanModbusMapEntry_t *entry)
{
    if ((entry == NULL) || (index >= can_modbus_map_count))
    {
        return false;
    }

    *entry = can_modbus_map_table[index];
    return true;
}

void CanModbusMap_RecordConfigResult(uint16_t result,
                                     uint16_t error_index,
                                     uint16_t error_field)
{
    config_stats.command_result = result;
    config_stats.config_error_index = error_index;
    config_stats.config_error_field = error_field;
}

/* 应用运行时配置：先完整校验所有映射项，全部合法后再替换当前表。 */
uint16_t CanModbusMap_ApplyEntries(const CanModbusMapEntry_t *entries,
                                   uint16_t entry_count)
{
    CanModbusMapEntry_t loaded[CAN_MODBUS_MAP_MAX_ENTRIES];
    uint16_t index;
    uint16_t error_field;

    if ((entries == NULL) ||
        (entry_count == 0U) ||
        (entry_count > CAN_MODBUS_MAP_MAX_ENTRIES))
    {
        CanModbusMap_RecordConfigResult(CAN_MODBUS_CFG_RESULT_BAD_DATA,
                                        0U,
                                        CAN_MODBUS_CFG_ERROR_COUNT);
        return CAN_MODBUS_CFG_RESULT_BAD_DATA;
    }

    for (index = 0U; index < entry_count; index++)
    {
        loaded[index] = entries[index];
        if (!CanModbusMap_ValidateEntry(&loaded[index], &error_field))
        {
            CanModbusMap_RecordConfigResult(CAN_MODBUS_CFG_RESULT_BAD_DATA,
                                            index,
                                            error_field);
            return CAN_MODBUS_CFG_RESULT_BAD_DATA;
        }
    }

    (void)memcpy(can_modbus_map_table,
                 loaded,
                 (uint32_t)entry_count * sizeof(CanModbusMapEntry_t));
    can_modbus_map_count = entry_count;
    config_stats.active_source = CAN_MODBUS_CFG_SOURCE_RUNTIME;
    CanModbusMap_RecordConfigResult(CAN_MODBUS_CFG_RESULT_OK,
                                    0U,
                                    CAN_MODBUS_CFG_ERROR_NONE);
    return CAN_MODBUS_CFG_RESULT_OK;
}

uint16_t CanModbusMap_HandleConfigCommand(uint16_t command)
{
    uint16_t result;

    switch (command)
    {
        case CAN_MODBUS_CFG_CMD_NONE:
            result = CAN_MODBUS_CFG_RESULT_IDLE;
            break;

        case CAN_MODBUS_CFG_CMD_DEFAULTS:
            CanModbusMap_LoadDefaults();
            result = CAN_MODBUS_CFG_RESULT_OK;
            break;

        case CAN_MODBUS_CFG_CMD_SAVE:
            result = CanModbusMap_SaveToEeprom();
            break;

        case CAN_MODBUS_CFG_CMD_LOAD:
            result = CanModbusMap_LoadFromEeprom();
            if (result != CAN_MODBUS_CFG_RESULT_OK)
            {
                CanModbusMap_LoadDefaults();
            }
            break;

        case CAN_MODBUS_CFG_CMD_SYNC_REGS:
            result = CAN_MODBUS_CFG_RESULT_OK;
            break;

        default:
            result = CAN_MODBUS_CFG_RESULT_BAD_CMD;
            break;
    }

    CanModbusMap_RecordConfigResult(result, 0U, CAN_MODBUS_CFG_ERROR_NONE);
    return result;
}

#include "modbus_reg.h"

#include "can_modbus_map.h"

#include <stddef.h>
#include <string.h>

/*
 * 阅读提示：
 * 本文件是 Modbus Holding Register 的统一入口，负责地址分段、读写权限和命令触发。
 * 外部 Modbus 主站不能直接写 0~7，CAN 映射模块通过 Modbus_RegWriteBmsMapped() 内部写入。
 */

static uint16_t bms_registers[MODBUS_BMS_REG_COUNT];
static uint16_t test_registers[MODBUS_TEST_REG_COUNT];
static uint16_t system_registers[MODBUS_SYS_REG_COUNT];
static uint16_t map_config_registers[MODBUS_MAP_CFG_REG_COUNT];
static uint16_t control_registers[MODBUS_CTRL_REG_COUNT];
static Modbus_ControlSender_t control_sender;

#define MODBUS_CFG_CMD_ADDRESS      MODBUS_TEST_REG_START
#define MODBUS_CFG_RESULT_ADDRESS   (MODBUS_TEST_REG_START + 1U)

#define MODBUS_MAP_CFG_COUNT_INDEX       0U
#define MODBUS_MAP_CFG_ENTRY_START_INDEX 1U
#define MODBUS_MAP_CFG_CAN_ID_LOW        0U
#define MODBUS_MAP_CFG_CAN_ID_HIGH       1U
#define MODBUS_MAP_CFG_DLC               2U
#define MODBUS_MAP_CFG_BYTE_OFFSET       3U
#define MODBUS_MAP_CFG_DATA_TYPE         4U
#define MODBUS_MAP_CFG_ADDRESS           5U
#define MODBUS_MAP_CFG_ENABLED           6U

#define MODBUS_CTRL_CMD_INDEX            0U
#define MODBUS_CTRL_LAST_CMD_INDEX       1U
#define MODBUS_CTRL_LAST_RESULT_INDEX    2U
#define MODBUS_CTRL_LAST_STATUS_INDEX    3U
#define MODBUS_CTRL_TX_OK_LOW_INDEX      4U
#define MODBUS_CTRL_TX_OK_HIGH_INDEX     5U
#define MODBUS_CTRL_TX_FAIL_LOW_INDEX    6U
#define MODBUS_CTRL_TX_FAIL_HIGH_INDEX   7U

static uint32_t control_tx_ok_count;
static uint32_t control_tx_fail_count;

void Modbus_RegInit(void)
{
    (void)memset(bms_registers, 0, sizeof(bms_registers));
    (void)memset(test_registers, 0, sizeof(test_registers));
    (void)memset(system_registers, 0, sizeof(system_registers));
    (void)memset(map_config_registers, 0, sizeof(map_config_registers));
    (void)memset(control_registers, 0, sizeof(control_registers));
    control_tx_ok_count = 0U;
    control_tx_fail_count = 0U;
}

void Modbus_RegSetControlSender(Modbus_ControlSender_t sender)
{
    control_sender = sender;
}

void Modbus_RegSyncBms(const BmsCanData_t *bms)
{
    if (bms == NULL)
    {
        return;
    }

    bms_registers[0] = bms->voltage_cv;
    bms_registers[1] = (uint16_t)bms->current_ca;
    bms_registers[2] = bms->soc_percent;
    bms_registers[3] = (uint16_t)((int16_t)bms->temperature_c);
    bms_registers[4] = bms->state_flags;
    bms_registers[5] = bms->fault_flags;
    bms_registers[6] = bms->online ? 1U : 0U;
    bms_registers[7] = bms->valid ? 1U : 0U;
}

/* CAN -> Modbus 的内部写入口，只允许写 BMS 数据区 0~7。 */
bool Modbus_RegWriteBmsMapped(uint16_t address, uint16_t value)
{
    uint16_t index;

    if (address >= (MODBUS_BMS_REG_START + MODBUS_BMS_REG_COUNT))
    {
        return false;
    }

    index = (uint16_t)(address - MODBUS_BMS_REG_START);
    bms_registers[index] = value;
    return true;
}

/* 把运行状态同步到 200~230，便于 Modbus Poll 观察网关健康状态。 */
void Modbus_RegSyncSystemStats(const BmsCanData_t *bms,
                               const ModbusRtuStats_t *modbus,
                               uint32_t now_ms,
                               uint32_t can_rx_count,
                               uint32_t can_rx_overflow_count,
                               status_t can_rx_rearm_status,
                               uint32_t can_init_error_count,
                               status_t can_last_init_error,
                               bool modbus_init_ok)
{
    uint32_t run_seconds = now_ms / 1000U;
    uint32_t bms_rx_age_ms = 0U;

    if (bms != NULL)
    {
        bms_rx_age_ms = now_ms - bms->last_rx_ms;
    }

    system_registers[0] = (uint16_t)(run_seconds & 0xFFFFU);
    system_registers[1] = (uint16_t)(run_seconds >> 16U);
    system_registers[2] = (uint16_t)(can_rx_count & 0xFFFFU);
    system_registers[3] = (uint16_t)(can_rx_count >> 16U);
    system_registers[4] = (uint16_t)(can_rx_overflow_count & 0xFFFFU);
    system_registers[5] = (uint16_t)(can_rx_overflow_count >> 16U);
    system_registers[6] = (uint16_t)can_rx_rearm_status;
    system_registers[7] = (uint16_t)(can_init_error_count & 0xFFFFU);
    system_registers[8] = (uint16_t)can_last_init_error;
    system_registers[9] = ((bms != NULL) && bms->online) ? 1U : 0U;
    system_registers[10] = ((bms != NULL) && bms->valid) ? 1U : 0U;
    system_registers[11] = (uint16_t)(bms_rx_age_ms & 0xFFFFU);
    system_registers[12] = (uint16_t)(bms_rx_age_ms >> 16U);
    system_registers[13] = (modbus != NULL) ? (uint16_t)(modbus->rx_frames & 0xFFFFU) : 0U;
    system_registers[14] = (modbus != NULL) ? (uint16_t)(modbus->rx_frames >> 16U) : 0U;
    system_registers[15] = (modbus != NULL) ? (uint16_t)(modbus->tx_frames & 0xFFFFU) : 0U;
    system_registers[16] = (modbus != NULL) ? (uint16_t)(modbus->tx_frames >> 16U) : 0U;
    system_registers[17] = (modbus != NULL) ? (uint16_t)(modbus->crc_errors & 0xFFFFU) : 0U;
    system_registers[18] = (modbus != NULL) ? (uint16_t)(modbus->address_misses & 0xFFFFU) : 0U;
    system_registers[19] = modbus_init_ok ? 1U : 0U;
}

void Modbus_RegSyncMapConfigStats(const CanModbusConfigStats_t *config)
{
    if (config == NULL)
    {
        return;
    }

    system_registers[20] = config->eeprom_present ? 1U : 0U;
    system_registers[21] = config->active_source;
    system_registers[22] = config->load_result;
    system_registers[23] = config->save_result;
    system_registers[24] = config->command_result;
    system_registers[25] = (uint16_t)(config->crc_errors & 0xFFFFU);
    system_registers[26] = (uint16_t)(config->crc_errors >> 16U);
    system_registers[27] = (uint16_t)(config->storage_errors & 0xFFFFU);
    system_registers[28] = (uint16_t)(config->storage_errors >> 16U);
    system_registers[29] = config->config_error_index;
    system_registers[30] = config->config_error_field;
}

void Modbus_RegSyncMapConfigTable(void)
{
    uint16_t index;
    uint16_t base;
    uint16_t count = CanModbusMap_GetEntryCount();
    CanModbusMapEntry_t entry;

    (void)memset(map_config_registers, 0, sizeof(map_config_registers));
    map_config_registers[MODBUS_MAP_CFG_COUNT_INDEX] = count;

    for (index = 0U; index < CAN_MODBUS_MAP_MAX_ENTRIES; index++)
    {
        base = (uint16_t)(MODBUS_MAP_CFG_ENTRY_START_INDEX +
                         (index * MODBUS_MAP_CFG_ENTRY_REG_COUNT));
        if ((index < count) && CanModbusMap_GetEntry(index, &entry))
        {
            map_config_registers[base + MODBUS_MAP_CFG_CAN_ID_LOW] =
                (uint16_t)(entry.can_id & 0xFFFFU);
            map_config_registers[base + MODBUS_MAP_CFG_CAN_ID_HIGH] =
                (uint16_t)(entry.can_id >> 16U);
            map_config_registers[base + MODBUS_MAP_CFG_DLC] = entry.required_dlc;
            map_config_registers[base + MODBUS_MAP_CFG_BYTE_OFFSET] = entry.byte_offset;
            map_config_registers[base + MODBUS_MAP_CFG_DATA_TYPE] = (uint16_t)entry.data_type;
            map_config_registers[base + MODBUS_MAP_CFG_ADDRESS] = entry.modbus_address;
            map_config_registers[base + MODBUS_MAP_CFG_ENABLED] = entry.enabled ? 1U : 0U;
        }
    }
}

/* 将 300~356 配置区转换成映射表；save_after_apply 为 true 时再写 EEPROM。 */
static uint16_t Modbus_RegApplyMapConfig(bool save_after_apply)
{
    CanModbusMapEntry_t entries[CAN_MODBUS_MAP_MAX_ENTRIES];
    uint16_t index;
    uint16_t base;
    uint16_t result;
    uint16_t count = map_config_registers[MODBUS_MAP_CFG_COUNT_INDEX];

    for (index = 0U; index < CAN_MODBUS_MAP_MAX_ENTRIES; index++)
    {
        base = (uint16_t)(MODBUS_MAP_CFG_ENTRY_START_INDEX +
                         (index * MODBUS_MAP_CFG_ENTRY_REG_COUNT));
        entries[index].can_id =
            (uint32_t)map_config_registers[base + MODBUS_MAP_CFG_CAN_ID_LOW] |
            ((uint32_t)map_config_registers[base + MODBUS_MAP_CFG_CAN_ID_HIGH] << 16U);
        entries[index].required_dlc =
            (uint8_t)map_config_registers[base + MODBUS_MAP_CFG_DLC];
        entries[index].byte_offset =
            (uint8_t)map_config_registers[base + MODBUS_MAP_CFG_BYTE_OFFSET];
        entries[index].data_type =
            (CanModbusMapDataType_t)map_config_registers[base + MODBUS_MAP_CFG_DATA_TYPE];
        entries[index].modbus_address =
            map_config_registers[base + MODBUS_MAP_CFG_ADDRESS];
        if (map_config_registers[base + MODBUS_MAP_CFG_ENABLED] > 1U)
        {
            CanModbusMap_RecordConfigResult(CAN_MODBUS_CFG_RESULT_BAD_DATA,
                                            index,
                                            CAN_MODBUS_CFG_ERROR_ENABLED);
            return CAN_MODBUS_CFG_RESULT_BAD_DATA;
        }
        entries[index].enabled =
            (map_config_registers[base + MODBUS_MAP_CFG_ENABLED] != 0U);
    }

    result = CanModbusMap_ApplyEntries(entries, count);
    if (result != CAN_MODBUS_CFG_RESULT_OK)
    {
        return result;
    }

    Modbus_RegSyncMapConfigTable();
    if (save_after_apply)
    {
        result = CanModbusMap_HandleConfigCommand(CAN_MODBUS_CFG_CMD_SAVE);
    }

    return result;
}

static uint16_t Modbus_RegHandleConfigCommand(uint16_t command)
{
    uint16_t result;

    switch (command)
    {
        case CAN_MODBUS_CFG_CMD_DEFAULTS:
            result = CanModbusMap_HandleConfigCommand(command);
            Modbus_RegSyncMapConfigTable();
            break;

        case CAN_MODBUS_CFG_CMD_LOAD:
            result = CanModbusMap_HandleConfigCommand(command);
            Modbus_RegSyncMapConfigTable();
            break;

        case CAN_MODBUS_CFG_CMD_SYNC_REGS:
            Modbus_RegSyncMapConfigTable();
            CanModbusMap_RecordConfigResult(CAN_MODBUS_CFG_RESULT_OK,
                                            0U,
                                            CAN_MODBUS_CFG_ERROR_NONE);
            result = CAN_MODBUS_CFG_RESULT_OK;
            break;

        case CAN_MODBUS_CFG_CMD_APPLY:
            result = Modbus_RegApplyMapConfig(false);
            break;

        case CAN_MODBUS_CFG_CMD_APPLY_SAVE:
            result = Modbus_RegApplyMapConfig(true);
            break;

        default:
            result = CanModbusMap_HandleConfigCommand(command);
            break;
    }

    return result;
}

static bool Modbus_RegIsControlCommandValid(uint16_t command)
{
    return (command == BMS_CAN_CMD_STATUS_REQ) ||
           (command == BMS_CAN_CMD_DSG_ON) ||
           (command == BMS_CAN_CMD_DSG_OFF);
}

static void Modbus_RegSyncControlCounters(void)
{
    control_registers[MODBUS_CTRL_TX_OK_LOW_INDEX] =
        (uint16_t)(control_tx_ok_count & 0xFFFFU);
    control_registers[MODBUS_CTRL_TX_OK_HIGH_INDEX] =
        (uint16_t)(control_tx_ok_count >> 16U);
    control_registers[MODBUS_CTRL_TX_FAIL_LOW_INDEX] =
        (uint16_t)(control_tx_fail_count & 0xFFFFU);
    control_registers[MODBUS_CTRL_TX_FAIL_HIGH_INDEX] =
        (uint16_t)(control_tx_fail_count >> 16U);
}

/* 写 400 触发 CAN 控制帧发送，并把发送结果记录到 401~407。 */
static bool Modbus_RegHandleControlCommand(uint16_t command)
{
    status_t send_status = STATUS_SUCCESS;
    bool send_ok;

    control_registers[MODBUS_CTRL_CMD_INDEX] = command;
    if (command == 0U)
    {
        control_registers[MODBUS_CTRL_LAST_RESULT_INDEX] =
            MODBUS_CTRL_RESULT_IDLE;
        control_registers[MODBUS_CTRL_LAST_STATUS_INDEX] =
            (uint16_t)STATUS_SUCCESS;
        return true;
    }

    control_registers[MODBUS_CTRL_LAST_CMD_INDEX] = command;
    if (!Modbus_RegIsControlCommandValid(command))
    {
        control_registers[MODBUS_CTRL_LAST_RESULT_INDEX] =
            MODBUS_CTRL_RESULT_INVALID_CMD;
        control_registers[MODBUS_CTRL_LAST_STATUS_INDEX] =
            (uint16_t)STATUS_ERROR;
        return false;
    }

    if (control_sender == NULL)
    {
        send_ok = false;
        send_status = STATUS_ERROR;
    }
    else
    {
        send_ok = control_sender((uint8_t)command, &send_status);
    }

    control_registers[MODBUS_CTRL_LAST_STATUS_INDEX] =
        (uint16_t)send_status;
    if (send_ok)
    {
        control_registers[MODBUS_CTRL_LAST_RESULT_INDEX] =
            MODBUS_CTRL_RESULT_SENT_OK;
        control_tx_ok_count++;
        Modbus_RegSyncControlCounters();
        return true;
    }

    control_registers[MODBUS_CTRL_LAST_RESULT_INDEX] =
        MODBUS_CTRL_RESULT_SEND_FAILED;
    control_tx_fail_count++;
    Modbus_RegSyncControlCounters();
    return false;
}

bool Modbus_RegRead(uint16_t start, uint16_t quantity, uint16_t *values)
{
    uint32_t end;
    uint16_t index;

    if ((values == NULL) || (quantity == 0U))
    {
        return false;
    }

    end = (uint32_t)start + quantity;
    if (end <= (MODBUS_BMS_REG_START + MODBUS_BMS_REG_COUNT))
    {
        for (index = 0U; index < quantity; index++)
        {
            values[index] = bms_registers[start + index];
        }
        return true;
    }

    if ((start >= MODBUS_TEST_REG_START) &&
        (end <= (MODBUS_TEST_REG_START + MODBUS_TEST_REG_COUNT)))
    {
        for (index = 0U; index < quantity; index++)
        {
            values[index] = test_registers[(start - MODBUS_TEST_REG_START) + index];
        }
        return true;
    }

    if ((start >= MODBUS_SYS_REG_START) &&
        (end <= (MODBUS_SYS_REG_START + MODBUS_SYS_REG_COUNT)))
    {
        for (index = 0U; index < quantity; index++)
        {
            values[index] = system_registers[(start - MODBUS_SYS_REG_START) + index];
        }
        return true;
    }

    if ((start >= MODBUS_MAP_CFG_REG_START) &&
        (end <= (MODBUS_MAP_CFG_REG_START + MODBUS_MAP_CFG_REG_COUNT)))
    {
        for (index = 0U; index < quantity; index++)
        {
            values[index] = map_config_registers[(start - MODBUS_MAP_CFG_REG_START) + index];
        }
        return true;
    }

    if ((start >= MODBUS_CTRL_REG_START) &&
        (end <= (MODBUS_CTRL_REG_START + MODBUS_CTRL_REG_COUNT)))
    {
        for (index = 0U; index < quantity; index++)
        {
            values[index] = control_registers[(start - MODBUS_CTRL_REG_START) + index];
        }
        return true;
    }

    return false;
}

/* Modbus 写权限边界：只开放测试区、映射配置区和控制命令入口。 */
bool Modbus_RegCanWrite(uint16_t start, uint16_t quantity)
{
    uint32_t end;

    if (quantity == 0U)
    {
        return false;
    }

    end = (uint32_t)start + quantity;
    if ((start >= MODBUS_TEST_REG_START) &&
        (end <= (MODBUS_TEST_REG_START + MODBUS_TEST_REG_COUNT)))
    {
        return true;
    }

    if ((start == MODBUS_CTRL_CMD_ADDRESS) && (quantity == 1U))
    {
        return true;
    }

    return (start >= MODBUS_MAP_CFG_REG_START) &&
           (end <= (MODBUS_MAP_CFG_REG_START + MODBUS_MAP_CFG_REG_COUNT));
}

bool Modbus_RegWrite(uint16_t start, uint16_t quantity, const uint16_t *values)
{
    uint16_t index;
    uint16_t address;
    uint16_t command_result;

    if ((values == NULL) || !Modbus_RegCanWrite(start, quantity))
    {
        return false;
    }

    if ((start >= MODBUS_MAP_CFG_REG_START) &&
        (((uint32_t)start + quantity) <=
         (MODBUS_MAP_CFG_REG_START + MODBUS_MAP_CFG_REG_COUNT)))
    {
        for (index = 0U; index < quantity; index++)
        {
            address = (uint16_t)(start + index);
            map_config_registers[(address - MODBUS_MAP_CFG_REG_START)] = values[index];
        }
        return true;
    }

    if ((start == MODBUS_CTRL_CMD_ADDRESS) && (quantity == 1U))
    {
        return Modbus_RegHandleControlCommand(values[0]);
    }

    for (index = 0U; index < quantity; index++)
    {
        address = (uint16_t)(start + index);
        test_registers[(address - MODBUS_TEST_REG_START)] = values[index];

        if ((address == MODBUS_CFG_CMD_ADDRESS) &&
            (values[index] != CAN_MODBUS_CFG_CMD_NONE))
        {
            command_result = Modbus_RegHandleConfigCommand(values[index]);
            test_registers[MODBUS_CFG_RESULT_ADDRESS - MODBUS_TEST_REG_START] =
                command_result;
            test_registers[MODBUS_CFG_CMD_ADDRESS - MODBUS_TEST_REG_START] =
                CAN_MODBUS_CFG_CMD_NONE;
        }
    }

    return true;
}

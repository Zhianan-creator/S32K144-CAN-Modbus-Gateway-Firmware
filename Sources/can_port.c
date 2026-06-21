#include "can_port.h"

#include "Cpu.h"
#include "bms_can_protocol.h"
#include "canCom1.h"

/*
 * 阅读提示：
 * 本文件对应 FlexCAN0：邮箱 0 接收 BMS 报文，邮箱 1 发送 0x300 控制命令。
 * 中断回调只做入队和重新挂接接收，协议解析放在 main.c 主循环之后执行。
 */

#define CAN0_RX_QUEUE_SIZE 8U

static const flexcan_data_info_t can0_tx_info = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = BMS_CAN_STATUS_DLC,
    .is_remote = false
};

static flexcan_msgbuff_t can0_rx_driver_msg;
static flexcan_msgbuff_t can0_rx_queue[CAN0_RX_QUEUE_SIZE];
static volatile uint8_t can0_rx_queue_head = 0U;
static volatile uint8_t can0_rx_queue_tail = 0U;
static volatile uint32_t can0_rx_overflow_count = 0U;
static volatile status_t can0_rx_rearm_status = STATUS_SUCCESS;
static volatile uint32_t can0_init_error_count = 0U;
static volatile status_t can0_last_init_error = STATUS_SUCCESS;
static volatile bool can0_ready = false;

/* 主循环调用：从 CAN 软件队列取一帧，取不到就返回 false。 */
bool CAN0_RxQueuePop(flexcan_msgbuff_t *msg)
{
    uint8_t tail = can0_rx_queue_tail;

    if ((msg == NULL) || (tail == can0_rx_queue_head))
    {
        return false;
    }

    *msg = can0_rx_queue[tail];
    can0_rx_queue_tail = (uint8_t)((tail + 1U) % CAN0_RX_QUEUE_SIZE);
    return true;
}

/* Modbus 控制寄存器最终会调用这里，发送 ID=0x300、data[0]=命令字。 */
bool CAN0_SendBmsControl(uint8_t command, status_t *send_status)
{
    uint8_t data[BMS_CAN_STATUS_DLC] = { 0U };
    status_t status = STATUS_ERROR;

    if (can0_ready)
    {
        data[0] = command;
        status = FLEXCAN_DRV_SendBlocking(INST_CANCOM1,
                                          1U,
                                          &can0_tx_info,
                                          BMS_CAN_CMD_ID,
                                          data,
                                          BMS_CAN_TX_TIMEOUT_MS);
    }

    if (send_status != NULL)
    {
        *send_status = status;
    }

    return status == STATUS_SUCCESS;
}

/* FlexCAN RX 完成回调：保持 ISR 很短，只复制帧到队列，不在中断里解析协议。 */
static void CAN0_Callback(uint8_t instance,
                          flexcan_event_type_t eventType,
                          uint32_t buffIdx,
                          flexcan_state_t *flexcanState)
{
    (void)flexcanState;
    (void)instance;
    (void)buffIdx;

    if (eventType == FLEXCAN_EVENT_RX_COMPLETE)
    {
        uint8_t head = can0_rx_queue_head;
        uint8_t next = (uint8_t)((head + 1U) % CAN0_RX_QUEUE_SIZE);

        if (next != can0_rx_queue_tail)
        {
            can0_rx_queue[head] = can0_rx_driver_msg;
            can0_rx_queue_head = next;
        }
        else
        {
            can0_rx_overflow_count++;
        }

        can0_rx_rearm_status = FLEXCAN_DRV_Receive(INST_CANCOM1, 0U, &can0_rx_driver_msg);
    }
}

/* 初始化 CAN0：125kbps，邮箱0接收，邮箱1发送，最后挂起第一次异步接收。 */
void CAN0_Init(void)
{
    status_t status;
    flexcan_user_config_t can0_cfg = canCom1_InitConfig0;
    flexcan_data_info_t rx_info = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8U,
        .is_remote = false
    };

    can0_ready = false;
    can0_cfg.bitrate.preDivider = 3;
    can0_cfg.bitrate_cbt.preDivider = 3;

    status = FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &can0_cfg);
    if (status != STATUS_SUCCESS)
    {
        can0_last_init_error = status;
        can0_init_error_count++;
        return;
    }

    FLEXCAN_DRV_SetRxMaskType(INST_CANCOM1, FLEXCAN_RX_MASK_INDIVIDUAL);
    status = FLEXCAN_DRV_SetRxIndividualMask(INST_CANCOM1,
                                             FLEXCAN_MSG_ID_STD,
                                             0U,
                                             0x00000000U);
    if (status != STATUS_SUCCESS)
    {
        can0_last_init_error = status;
        can0_init_error_count++;
        return;
    }

    status = FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, 0U, &rx_info, 0x000);
    if (status != STATUS_SUCCESS)
    {
        can0_last_init_error = status;
        can0_init_error_count++;
        return;
    }

    status = FLEXCAN_DRV_ConfigTxMb(INST_CANCOM1, 1U, &can0_tx_info, BMS_CAN_CMD_ID);
    if (status != STATUS_SUCCESS)
    {
        can0_last_init_error = status;
        can0_init_error_count++;
        return;
    }

    FLEXCAN_DRV_InstallEventCallback(INST_CANCOM1, CAN0_Callback, NULL);

    can0_rx_rearm_status = FLEXCAN_DRV_Receive(INST_CANCOM1, 0U, &can0_rx_driver_msg);
    if (can0_rx_rearm_status != STATUS_SUCCESS)
    {
        can0_last_init_error = can0_rx_rearm_status;
        can0_init_error_count++;
        return;
    }

    can0_ready = true;
}

bool CAN0_IsReady(void)
{
    return can0_ready;
}

uint32_t CAN0_GetRxOverflowCount(void)
{
    return can0_rx_overflow_count;
}

status_t CAN0_GetRxRearmStatus(void)
{
    return can0_rx_rearm_status;
}

uint32_t CAN0_GetInitErrorCount(void)
{
    return can0_init_error_count;
}

status_t CAN0_GetLastInitError(void)
{
    return can0_last_init_error;
}

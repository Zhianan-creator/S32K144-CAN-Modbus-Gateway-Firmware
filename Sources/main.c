/* 板载 Modbus 接口使用 LPUART2，PTE14 控制 RS485 的 DE+/RE。 */
#include "Cpu.h"
#include "delay.h"
#include"key.h"
#include"oled.h"
#include "can_port.h"
#include "can_modbus_map.h"
#include "bms_can_protocol.h"
#include "modbus_reg.h"
#include "modbus_rtu.h"
#include "rs485_port.h"
#include "uart.h"

  volatile int exit_code = 0;

#define LED1(x)  PINS_DRV_WritePin(PTD,16,!x);
#define LED3(x)  PINS_DRV_WritePin(PTD,1,!x);
#define LED4(x)  PINS_DRV_WritePin(PTD,0,!x);

#define LPIT0_PERIOD_MS 100U
#define LPUART1_SUMMARY_PERIOD_MS 10000U
#define OLED_STATUS_PERIOD_MS 500U

/*
 * 阅读提示：
 * main.c 只保留板级初始化、主循环调度和 100ms 系统节拍。
 * CAN 收发看 can_port.c，CAN 到 Modbus 映射看 can_modbus_map.c，寄存器表看 modbus_reg.c。
 */

static volatile uint32_t system_time_ms = 0U;
static volatile bool modbus_init_ok = false;

static void LPIT0_Init(void)
{
	PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(2U) | PCC_PCCn_CGC(1U);
	LPIT0->MCR = LPIT_MCR_M_CEN(1U);
	LPIT0->TMR[0].TVAL = 4000000U - 1U;
	LPIT0->MSR = LPIT_MSR_TIF0(1U);
	LPIT0->MIER = LPIT_MIER_TIE0(1U);
	INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);
	LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_T_EN(1U);
}

void LPIT0_Ch0_IRQHandler(void)
{
	LPIT0->MSR = LPIT_MSR_TIF0(1U);
	system_time_ms += LPIT0_PERIOD_MS;
	PINS_DRV_TogglePins(PTD, 1UL << 0);
}

/*
 * 主入口阅读顺序：先看初始化，再看 while(1) 中的周期任务。
 * 重点关注 CAN 队列处理、Modbus 状态同步、OLED 500ms 刷新、LPUART1 10s 摘要。
 */
int main(void)
{
  /* Write your local variable definition here */
	uint8_t key_now;
	int MCU_Freq;
	flexcan_msgbuff_t rx_msg;
	MonitorCanRawFrame_t last_can_raw_frame = { 0 };
	BmsCanData_t bms_data;
	uint32_t can_rx_count = 0U;
	uint32_t last_summary_ms = 0U;
	uint32_t last_oled_ms = 0U;
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

	CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
	MCU_Freq = delay_init();
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	LED4(0);
	LPIT0_Init(); // 100ms系统节拍
	I2C_MasterInit(&i2c1_instance, &i2c1_MasterConfig0); // OLED使用的I2C1
	(void)LPUART1_MonitorInit(); // 调试串口
	CAN0_Init(); // CAN收发
	Modbus_RegInit(); // 初始化Modbus寄存器表
	Modbus_RegSetControlSender(CAN0_SendBmsControl); // Modbus控制寄存器写入后转CAN控制帧
	CanModbusMap_Init(&bms_data); // 加载CAN到Modbus映射表
	modbus_init_ok = RS485_PortInit(); // RS485端口
	if (modbus_init_ok)
	{
		modbus_init_ok = Modbus_RTU_Init(); // Modbus RTU协议层
	}
	Modbus_RegSyncSystemStats(&bms_data, // 首次同步系统状态寄存器
	                          Modbus_RTU_GetStats(),
	                          system_time_ms,
	                          can_rx_count,
	                          CAN0_GetRxOverflowCount(),
	                          CAN0_GetRxRearmStatus(),
	                          CAN0_GetInitErrorCount(),
	                          CAN0_GetLastInitError(),
	                          modbus_init_ok);
	Modbus_RegSyncMapConfigStats(CanModbusMap_GetConfigStats());
	Modbus_RegSyncMapConfigTable(); // 同步映射表配置寄存器

	oled_init(); // OLED初始化
	OLED_TITLE((uint8_t*)"S32K144",(uint8_t*)"01_BASE");
	(void)MCU_Freq;
    while(1)
    {
    	key_now = BTN1;
    	LED1(key_now);
    	LED3(key_now);

    	/* CAN中断只入队，协议解析放在主循环。 */
    	while (CAN0_RxQueuePop(&rx_msg))
    	{
    		uint32_t now_ms = system_time_ms;

    		uint8_t raw_index;

    		can_rx_count++;
    		last_can_raw_frame.valid = true;
    		last_can_raw_frame.id = rx_msg.msgId;
    		last_can_raw_frame.dlc = rx_msg.dataLen;
    		for (raw_index = 0U; raw_index < 8U; raw_index++)
    		{
    			last_can_raw_frame.data[raw_index] = rx_msg.data[raw_index];
    		}
     		(void)CanModbusMap_ProcessFrame(&bms_data, // CAN帧写入Modbus寄存器
     				                       rx_msg.msgId,
     				                       rx_msg.dataLen,
     				                       rx_msg.data,
     				                       now_ms);
    	}

    	(void)CanModbusMap_CheckOffline(&bms_data, system_time_ms); // 检查CAN超时
    	Modbus_RegSyncSystemStats(&bms_data, // 刷新系统状态寄存器
    	                          Modbus_RTU_GetStats(),
    	                          system_time_ms,
    	                          can_rx_count,
    	                          CAN0_GetRxOverflowCount(),
    	                          CAN0_GetRxRearmStatus(),
    	                          CAN0_GetInitErrorCount(),
    	                          CAN0_GetLastInitError(),
    	                          modbus_init_ok);
    	Modbus_RegSyncMapConfigStats(CanModbusMap_GetConfigStats());
    	if ((uint32_t)(system_time_ms - last_oled_ms) >= OLED_STATUS_PERIOD_MS)
    	{
    		last_oled_ms = system_time_ms;
    		OLED_StatusUpdate(&bms_data, // 500ms刷新OLED
    		                  Modbus_RTU_GetStats(),
    		                  can_rx_count,
    		                  CAN0_IsReady(),
    		                  modbus_init_ok);
    	}
    	if ((uint32_t)(system_time_ms - last_summary_ms) >= LPUART1_SUMMARY_PERIOD_MS)
    	{
    		last_summary_ms = system_time_ms;
    		LPUART1_MonitorPrintSummary(&bms_data, // 10s打印SYS和CAN RAW
    		                            Modbus_RTU_GetStats(),
    		                            &last_can_raw_frame,
    		                            system_time_ms,
    		                            can_rx_count,
    		                            CAN0_GetRxOverflowCount(),
    		                            CAN0_GetRxRearmStatus(),
    		                            CAN0_GetInitErrorCount(),
    		                            CAN0_GetLastInitError(),
    		                            modbus_init_ok);
    	}
    	Modbus_RTU_Poll(); // 处理Modbus主站请求
    }
  /*** Don't write any code pass th5is line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP S32K series of microcontrollers.
**
** ###################################################################
*/

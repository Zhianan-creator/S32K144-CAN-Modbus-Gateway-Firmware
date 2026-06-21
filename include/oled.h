/*
 * oled.h
 *
 *  Created on: 2021年3月5日
 *      Author: Administrator
 */

#ifndef OLED_H_
#define OLED_H_

#include "bms_can_protocol.h"
#include "modbus_rtu.h"

#include <stdbool.h>
#include <stdint.h>

#define XLevelL  	0x00
#define XLevelH  	0x10
#define XLevel          ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column 	128
#define Max_Row  	64
#define Brightness 	0xCF
#define X_WIDTH 	128
#define Y_WIDTH 	64

 uint8_t I2c_Send_Byte(uint8_t Dev_reg,uint8_t data);
 uint8_t I2c_Read_Byte(uint8_t Dev_reg);
 void oled_init(void);
 void OLED_WrDat(unsigned char IIC_Data);
 void OLED_WrCmd(unsigned char IIC_Command);
 void OLED_Set_Pos(unsigned char x, unsigned char y);
 void OLED_Fill(unsigned char bmp_dat)	;
 void OLED_CLS(void);
 void OLED_Init(void);
 void LCD_clear_L(unsigned char x,unsigned char y);
   void LCD_clear_L_POS(unsigned char x1,unsigned char x2,unsigned char y);
 void OLED_Clear(void);
 void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[])	;
 void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ucData);
 void OLED_write_number(unsigned char x,unsigned char y, float number,uint8_t fontsize,uint8_t mode);
 void OLED_Clear(void)  ;
 void OLED_task (void * pvParameters);
 void OLED_process(void);
 void OLED_show(int count,int page,int refresh_flag);
 void Draw_Logo1(void);
 void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t mode);
 void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size,uint8_t MODE);
 void OLED_ShowStringCHN(uint8_t x,uint8_t y,uint8_t *chr,uint8_t MODE);
 void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t mode);
 void OLED_TITLE(uint8_t * type,uint8_t * title);
 
/*
 * 【API】OLED_StatusUpdate
 * 调用者：main.c 每 500ms 调用一次。
 * 作用：刷新 OLED page 2~7 的网关状态页。
 * 显示内容：CAN/Modbus 初始化状态、BMS 在线状态、CAN_RX、总压、SOC、电流、温度、状态位、故障位、BMS_VALID、Modbus RX。
 * 注意：这里只负责显示，不改变 CAN/Modbus/BMS 数据。
 */
void OLED_StatusUpdate(const BmsCanData_t *bms,
                        const ModbusRtuStats_t *modbus,
                        uint32_t can_rx_count,
                        bool can_ready,
                        bool modbus_init_ok);
  void OLED_TITLE2(uint8_t * type,uint8_t * title);

#endif /* OLED_H_ */

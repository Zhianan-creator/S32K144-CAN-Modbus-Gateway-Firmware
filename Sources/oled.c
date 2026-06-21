/*
 * 用户函数列表
 *   OLED_ShowString 	显示8或者16大小英文字符串
 *   OLED_ShowStringCHN 显示16大小中文字符串
 *   OLED_write_number  显示数字
 *   OLED_CLS			清屏
 *   LCD_clear_L		清行
 *
 *   拼接字符串用法示例
 *   uint8_t oled_demo[6];
 *   sprintf((char*)oled_demo,"string + %d",3.14f);
 *   OLED_ShowString(0,0,oled_demo,0,);
 */
#include"cpu.h"
#include"oled.h"
#include"font.h"
#include"delay.h"

/*
 * 阅读提示：
 * 前半部分是 OLED 底层 I2C/显示函数，文件末尾的 OLED_StatusUpdate() 是网关状态页。
 * main.c 每 500ms 调用一次状态页刷新，显示 CAN/Modbus/BMS 的核心状态。
 */

#define OLED_I2C_ADDRESS 0x3CU





  uint8_t I2c_Send_Byte(uint8_t Dev_reg,uint8_t data)
  {
	uint8_t I2c_Tx_Buff[2];
	(void)I2C_MasterSetSlaveAddress(&i2c1_instance, OLED_I2C_ADDRESS, false);
	I2c_Tx_Buff[0] = Dev_reg;
	I2c_Tx_Buff[1] = data;
	I2C_MasterSendDataBlocking(&i2c1_instance, I2c_Tx_Buff, sizeof(I2c_Tx_Buff), true, 100);
	return 0;
  }


  uint8_t I2c_Read_Byte(uint8_t Dev_reg)
  {
	   uint8_t Rx_Buff;
	   (void)I2C_MasterSetSlaveAddress(&i2c1_instance, OLED_I2C_ADDRESS, false);
	   I2C_MasterSendDataBlocking(&i2c1_instance, &Dev_reg, 1, true, 100);
	   I2C_MasterReceiveDataBlocking(&i2c1_instance,&Rx_Buff,1,true,100);
	   return Rx_Buff;
  }

 /*OLED写数据*/
  void OLED_WrDat(unsigned char IIC_Data)
  {
  	 I2c_Send_Byte(0x40,IIC_Data);
  }

 /*OLED写命令*/
  void OLED_WrCmd(unsigned char IIC_Command)
  {
   I2c_Send_Byte(0x00,IIC_Command);
  }


  void OLED_Set_Pos(unsigned char x, unsigned char y)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(((x&0xf0)>>4)|0x10);
    OLED_WrCmd((x&0x0f)|0x01);
  }
  /*OLED全屏*/
  void OLED_Fill(unsigned char bmp_dat)
  {
    unsigned char y,x;
    for(y=0;y<8;y++)
    {
      OLED_WrCmd(0xb0+y);
      OLED_WrCmd(0x01);
      OLED_WrCmd(0x10);
      for(x=0;x<X_WIDTH;x++)
        OLED_WrDat(bmp_dat);
    }
  }







  /*OLED清屏*/
  void OLED_Clear(void)
  {
  	uint8_t i,n;
  	for(i=0;i<8;i++)
  	{
  		OLED_WrCmd (0xb0+i);    //设置页地址（0~7）
  		OLED_WrCmd (0x00);      //设置显示位置—列低地址
  		OLED_WrCmd (0x10);      //设置显示位置—列高地址
  		for(n=0;n<128;n++)OLED_WrDat(0);
  	} //更新显示
  }





  /*被调用函数*/
  void LCD_P6x8Char(unsigned char x,unsigned char  y,unsigned char ucData)
  {
    unsigned char i, ucDataTmp;
    ucDataTmp = ucData-32;
    if(x > 126)
    {
      x= 0;
      y++;
    }
    OLED_Set_Pos(x, y);
    for(i = 0; i < 6; i++)
    {
      OLED_WrDat(F6x8[ucDataTmp][i]);
    }

  }






  /*被调用函数*/
  void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t mode)
  {
	  uint8_t t,adder=0;
  	OLED_Set_Pos(x,y);
      for(t=0;t<16;t++)
  		{
  			if(mode == 0)//反白
  				{
  					OLED_WrDat(Hzk[2*no][t]);
  				}
  			else if(mode == 1)
  				{
  					OLED_WrDat(~Hzk[2*no][t]);
  				}

  				adder+=1;
       }
  		OLED_Set_Pos(x,y+1);
      for(t=0;t<16;t++)
  			{
  		if(mode == 0)//反白
  				{
  				OLED_WrDat(Hzk[2*no+1][t]);
  				}
  		else if(mode == 1)
  				{
  					OLED_WrDat(~Hzk[2*no+1][t]);
  				}
  				adder+=1;
        }
  }


  /*
   * 被调用函数
   * void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[])
   * 显示6X8一组标准的ASCII字符串，显示坐标为（x，y）-x为行,y为列
   * */
  void LCD_P6x8Str(unsigned char x,unsigned char  y,unsigned char ch[])
  {
    unsigned char c=0,i=0,j=0;
    while (ch[j]!='\0')
    {
      c =ch[j]-32;
      if(x>126){x=0;y++;}
      OLED_Set_Pos(x,y);
      for(i=0;i<6;i++)
        OLED_WrDat(F6x8[c][i]);

      x+=6;
      j++;
    }
  }


  /*被调用函数*/
  void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t mode)
  {
  	  unsigned char c=0,i=0;
  		c=chr-' ';//得到偏移后的值
  		if(x>Max_Column-1){x=0;y=y+2;}
  		if(Char_Size ==16)
  			{
  			OLED_Set_Pos(x,y);
  			for(i=0;i<8;i++)
  				if(mode == 0)  //反白
  				{
  					OLED_WrDat(F8X16[c*16+i]);
  				}
  				else if(mode == 1)
  				{
  					OLED_WrDat(~F8X16[c*16+i]);
  				}
  			OLED_Set_Pos(x,y+1);
  			for(i=0;i<8;i++)
  				if(mode == 0)//反白
  				{
  					OLED_WrDat(F8X16[c*16+i+8]);
  				}
  				else if(mode == 1)
  				{
  					OLED_WrDat(~F8X16[c*16+i+8]);
  				}
  			}
  			else {
  				OLED_Set_Pos(x,y);
  				for(i=0;i<6;i++)

  					if(mode == 0)//反白
  				{
  					OLED_WrDat(F6x8[c][i]);
  				}
  				else if(mode == 1)
  				{
  					OLED_WrDat(~F6x8[c][i]);
  				}

  			}
  }


  /*  用户函数
   *  OLED显示 8 或 16 大小英文字符串
   *  void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size,uint8_t MODE)
   *  X		       	列
   *  Y		       	行
   *  chr      		字符串  uint8_t string[] = "test string";
   *  Char_Size 	字符大小 支持 8 或者16 8= 8*6大小 16= 16*8大小
   *  mode     		反白显示 1=反白 0=不反白
   *
  */
  void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size,uint8_t MODE)
  {
  	unsigned char j=0;
  	while (chr[j]!='\0')
  	{	 OLED_ShowChar(x,y,chr[j],Char_Size,MODE);
  			x+=8;
  		if(x>120){x=0;y+=2;}
  			j++;
  	}
  }


  /*  用户函数
   *  OLED显示 16*16大小汉字
   *  void OLED_ShowStringCHN(uint8_t x,uint8_t y,uint8_t *chr,uint8_t char_num,uint8_t Char_Size,uint8_t MODE,uint8_t languege)
   *  X		       列
   *  Y		       行
   *  chr      字库数组顺序  uint8_t hanzi[] ={1,2,3}
   *  mode     反白显示 1=反白 0=不反白
   *
  */
  void OLED_ShowStringCHN(uint8_t x,uint8_t y,uint8_t *chr,uint8_t MODE)
  	{
  		unsigned char i=0;
  		unsigned char char_num = sizeof(chr);
		while(i<char_num)
		{
			OLED_ShowCHinese(x,y*2,chr[i],MODE); x+=16;
			i++;
		}
  	}


  /*
     *  OLED显示数字
     *  void OLED_write_number(unsigned char x,unsigned char y, float number,uint8_t fontsize,uint8_t mode)
     *  X		       列
     *  Y		       行
     *  number   数字
     *  fontsize 字体大小,可选 8 or 16
     *  mode     反白显示 1=反白 0=不反白
     *
    */
    void OLED_write_number(unsigned char x,unsigned char y, float number,uint8_t fontsize,uint8_t mode)
    {
      unsigned char i=0;
      unsigned char temp[16];
      unsigned char *point=temp;
      float decimal;
      int data;
      if(number<0)
      {
        temp[0]='-';
        LCD_P6x8Char(x,y,temp[0]);
        x+=6;
        number=-number;
      }
      data=(int)number;
      decimal=number-data;					//小数部分

      if(data>=1000000000)           //是否能被10^9整除
      {
        temp[i]=48+data/1000000000;
        data=data%1000000000;
        i++;
      }
      if(data>=100000000)           //是否能被10^8整除
      {
        temp[i]=48+data/100000000;
        data=data%100000000;
        i++;
      }
      else
        if(data<100000000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=10000000)           //是否能被10^7整除
      {
        temp[i]=48+data/10000000;
        data=data%10000000;
        i++;
      }
      else
        if(data<10000000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=1000000)           //是否能被10^6整除
      {
        temp[i]=48+data/1000000;
        data=data%1000000;
        i++;
      }
      else
        if(data<1000000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=100000)           //是否能被100000整除
      {
        temp[i]=48+data/100000;
        data=data%100000;
        i++;
      }
      else
        if(data<100000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=10000)           //是否能被10000整除
      {
        temp[i]=48+data/10000;
        data=data%10000;
        i++;
      }
      else
        if(data<10000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=1000)           //是否能被1000整除
      {
        temp[i]=48+data/1000;
        data=data%1000;
        i++;
      }
      else
        if(data<1000&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=100)              //是否能被100整除
      {
        temp[i]=48+data/100;
        data=data%100;
        i++;
      }
      else
        if(data<100&&i!=0)
        {
          temp[i]=0+48;
          i++;
        }
      if(data>=10)                  //是否能被10整除
      {
        temp[i]=48+data/10;
        data=data%10;
        i++;
      }
      else
        if(data<10&&i!=0)
        {
          temp[i]=48;
          i++;
        }
      temp[i]=48+data;
      if(decimal>=0.0001)           //判断是否有小数部分
      {
        i++;
        temp[i]='.';                //显示小数点
        i++;
        data=(int)(decimal*1000);
        temp[i]=48+data/100;
        data=data%100;
        i++;
        if(data>0)
        {
          temp[i]=48+data/10;
          data=data%10;
        }
        if(data>=0)
        {
          i++;
          temp[i]=data+48;
        }
      }
      i++;
      temp[i]='\0';
    	if(fontsize==16) //如果字体大于8就调用大字体,小于8就小字体
    	{
    		OLED_ShowString(x,y,point,fontsize,mode);
    	}
     else if(fontsize==8)
     {
    	 LCD_P6x8Str(x,y,point);
     }
    }




  /*清屏函数*/
  void OLED_CLS(void)
  {
    unsigned char y,x;
    for(y=0;y<8;y++)
    {
      OLED_WrCmd(0xb0+y);
      OLED_WrCmd(0x01);
      OLED_WrCmd(0x10);
      for(x=0;x<X_WIDTH;x++)
        OLED_WrDat(0);
    }
  }


  /*OLED清行*/
  void LCD_clear_L(unsigned char x,unsigned char y)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    OLED_Set_Pos(x,y);
    for(;x<X_WIDTH;x++)
    {
      OLED_WrDat(0);
    }
  }


 /* 指定位置清行x1起始行,x2结束行*/
  void LCD_clear_L_POS(unsigned char x1,unsigned char x2,unsigned char y)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    OLED_Set_Pos(x1,y);
    for(;x1<x2;x1++)
    {
      OLED_WrDat(0);
    }
  }



  void OLED_TITLE(uint8_t * type,uint8_t * title)
  {
  	uint8_t logo1[] ={10,10,11,11}; //品智科技
  	//uint8_t logo2[] ={4,5,6,7}; //字库编码--极客电子
  	OLED_ShowStringCHN(0,0,logo1,0);
  	OLED_ShowString(64,0,type,8,0);
  	OLED_ShowString(64,1,title,8,0);
  }
  
  
    void OLED_TITLE2(uint8_t * type,uint8_t * title)
  {
  	uint8_t logo1[] ={10,10,11,11}; //品智科技
  	//uint8_t logo2[] ={4,5,6,7}; //字库编码--极客电子
  	OLED_ShowStringCHN(0,0,logo1,0);
  	OLED_ShowString(64,0,type,16,0);
  	OLED_ShowString(0,2,title,8,0);
  }





void oled_init(void)
{
	  delay_ms(50);
	  OLED_WrCmd(0xae);//--turn off oled panel
	  OLED_WrCmd(0x00);//---set low column address
	  OLED_WrCmd(0x10);//---set high column address
	  OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	  OLED_WrCmd(0x81);//--set contrast control registerjn
	  OLED_WrCmd(Brightness); // Set SEG Output Current Brightness
	  OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	  OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	  OLED_WrCmd(0xa6);//--set normal display
	  OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
	  OLED_WrCmd(0x3f);//--1/64 duty
	  OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	  OLED_WrCmd(0x00);//-not offset
	  OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
	  OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	  OLED_WrCmd(0xd9);//--set pre-charge period
	  OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	  OLED_WrCmd(0xda);//--set com pins hardware configuration
	  OLED_WrCmd(0x12);
	  OLED_WrCmd(0xdb);//--set vcomh
	  OLED_WrCmd(0x40);//Set VCOM Deselect Level
	  OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	  OLED_WrCmd(0x02);//
	  OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
	  OLED_WrCmd(0x14);//--set(0x10) disable
	  OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
	  OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
	  OLED_WrCmd(0xaf);//--turn on oled panel
	  OLED_Fill(0x00); //初始清屏
	  OLED_Set_Pos(0,0);
	  delay_ms(50);
}

static void OLED_LineClearAndShow(uint8_t page, const char *text)
{
    LCD_clear_L(0U, page);
    OLED_ShowString(0U, page, (uint8_t *)text, 8U, 0U);
}

static void OLED_LineAppendChar(char *line, uint8_t *pos, char value)
{
    if (*pos < 16U)
    {
        line[*pos] = value;
        (*pos)++;
        line[*pos] = '\0';
    }
}

static void OLED_LineAppendText(char *line, uint8_t *pos, const char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        OLED_LineAppendChar(line, pos, *text);
        text++;
    }
}

static void OLED_LineAppendU32(char *line, uint8_t *pos, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        count++;
    }
    while ((value != 0U) && (count < sizeof(digits)));

    while (count > 0U)
    {
        count--;
        OLED_LineAppendChar(line, pos, digits[count]);
    }
}

static void OLED_LineAppendHex8(char *line, uint8_t *pos, uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    OLED_LineAppendChar(line, pos, hex[(value >> 4U) & 0x0FU]);
    OLED_LineAppendChar(line, pos, hex[value & 0x0FU]);
}

static void OLED_LineAppendCentValue(char *line, uint8_t *pos, int32_t value)
{
    uint32_t abs_value;
    uint32_t integer;
    uint32_t decimal;

    if (value < 0)
    {
        OLED_LineAppendChar(line, pos, '-');
        abs_value = (uint32_t)(-value);
    }
    else
    {
        abs_value = (uint32_t)value;
    }

    integer = abs_value / 100U;
    decimal = abs_value % 100U;
    OLED_LineAppendU32(line, pos, integer);
    OLED_LineAppendChar(line, pos, '.');
    OLED_LineAppendChar(line, pos, (char)('0' + (decimal / 10U)));
    OLED_LineAppendChar(line, pos, (char)('0' + (decimal % 10U)));
}

/* 500ms 状态页刷新入口：只负责拼 6 行短文本并写到 OLED page 2~7。 */
void OLED_StatusUpdate(const BmsCanData_t *bms,
                       const ModbusRtuStats_t *modbus,
                       uint32_t can_rx_count,
                       bool can_ready,
                       bool modbus_init_ok)
{
    char line[17];
    uint8_t pos;
    uint32_t modbus_rx = 0U;

    if (modbus != NULL)
    {
        modbus_rx = modbus->rx_frames;
    }

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "CAN:");
    OLED_LineAppendText(line, &pos, can_ready ? "OK" : "ER");
    OLED_LineAppendText(line, &pos, " MB:");
    OLED_LineAppendText(line, &pos, modbus_init_ok ? "OK" : "ER");
    OLED_LineClearAndShow(2U, line);

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "BMS:");
    OLED_LineAppendText(line, &pos, ((bms != NULL) && bms->online) ? "ON" : "OFF");
    OLED_LineAppendText(line, &pos, " RX:");
    OLED_LineAppendU32(line, &pos, can_rx_count);
    OLED_LineClearAndShow(3U, line);

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "V:");
    OLED_LineAppendCentValue(line, &pos, (bms != NULL) ? bms->voltage_cv : 0);
    OLED_LineAppendText(line, &pos, " SOC:");
    OLED_LineAppendU32(line, &pos, (bms != NULL) ? bms->soc_percent : 0U);
    OLED_LineClearAndShow(4U, line);

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "I:");
    OLED_LineAppendCentValue(line, &pos, (bms != NULL) ? bms->current_ca : 0);
    OLED_LineAppendText(line, &pos, " T:");
    if ((bms != NULL) && (bms->temperature_c < 0))
    {
        OLED_LineAppendChar(line, &pos, '-');
        OLED_LineAppendU32(line, &pos, (uint32_t)(-bms->temperature_c));
    }
    else
    {
        OLED_LineAppendU32(line, &pos, (bms != NULL) ? (uint32_t)bms->temperature_c : 0U);
    }
    OLED_LineClearAndShow(5U, line);

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "S:");
    OLED_LineAppendHex8(line, &pos, (bms != NULL) ? bms->state_flags : 0U);
    OLED_LineAppendText(line, &pos, " F:");
    OLED_LineAppendHex8(line, &pos, (bms != NULL) ? bms->fault_flags : 0U);
    OLED_LineAppendText(line, &pos, " V:");
    OLED_LineAppendText(line, &pos, ((bms != NULL) && bms->valid) ? "1" : "0");
    OLED_LineClearAndShow(6U, line);

    pos = 0U;
    line[0] = '\0';
    OLED_LineAppendText(line, &pos, "MBRX:");
    OLED_LineAppendU32(line, &pos, modbus_rx);
    OLED_LineClearAndShow(7U, line);
}

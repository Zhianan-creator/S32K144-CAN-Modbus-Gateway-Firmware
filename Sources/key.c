/*
 * 品智科技
 * 按键函数
 */
#include"cpu.h"
#include"key.h"
#include"delay.h"

/*0为单次模式 1为连续模式*/
unsigned char  KEY_Proc (unsigned char mode)
{
    static unsigned char key_up=1;     //按键松开标志
    if(mode==1)key_up=1;    //支持连按
   // u3_printf("keyup:%d BTN0:%d BTN1:%d\r\n",key_up,BTN0,BTN1);
    if(key_up&&(BTN1==1||BTN2==1||BTN3==1))
    {
    	//u3_printf("in key proc\r\n");
        delay_ms(10);
        key_up=0;
        if(BTN1==1)       return BTN1_PRES;
        else if(BTN2==1)  return BTN2_PRES;
        else if(BTN3==1)  return BTN3_PRES;

    }else if(BTN1==0&&BTN2==0&&BTN3==0)key_up=1;
    return 0;   //无按键按下
}

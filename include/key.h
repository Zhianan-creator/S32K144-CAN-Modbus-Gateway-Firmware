/*
 * key.h
 *
 *  Created on: 2020Äê11ÔÂ18ÈÕ
 *      Author: Administrator
 */

#ifndef KEY_H_
#define KEY_H_

#include "Cpu.h"
#include <stdio.h>
#include <stdint.h>

#ifndef KEY_INPUT_H_
#define KEY_INPUT_H_



#define BTN1 ((PINS_DRV_ReadPins(PTC)>>12)&0x01)
#define BTN2 ((PINS_DRV_ReadPins(PTC)>>13)&0x01)
#define BTN3 ((PINS_DRV_ReadPins(PTB)>>2)&0x01)

#define BTN1_PRES 	1
#define BTN2_PRES	2
#define BTN3_PRES	3
unsigned char  KEY_Proc (unsigned char mode);



#endif /* KEY_INPUT_H_ */


#endif /* KEY_H_ */

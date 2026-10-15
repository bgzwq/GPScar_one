#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

/*按键位定义（位图中的位）*/
#define KEY_1		0x01	//PB12
#define KEY_2		0x02	//PB13
#define KEY_3		0x04	//PB14
#define KEY_4		0x08	//PB15

void Key_Init(void);
void Key_Scan(void);
uint8_t Key_GetState(void);
uint8_t Key_GetEvent(void);

#endif

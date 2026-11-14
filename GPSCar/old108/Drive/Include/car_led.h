#ifndef __CAR_LED_H
#define __CAR_LED_H

#include "stm32f4xx.h"

/*---------------------- 车灯LED配置宏 ------------------------*/
/*
 * 4个LED: PG0 PG1 PG2 PG3, 低电平点亮(共阳接法)
 * 若实际硬件为高电平点亮, 将 CAR_LED_ACTIVE_LOW 改为 0
 */
#define CAR_LED_ACTIVE_LOW	1

#define CAR_LED_NUM			4

#define CAR_LED1	0		// PG0
#define CAR_LED2	1		// PG1
#define CAR_LED3	2		// PG2
#define CAR_LED4	3		// PG3

/*---------------------- 函数声明 ----------------------------*/

void 	CarLED_Init(void);						// LED初始化(默认全灭)
void 	CarLED_On(uint8_t id);					// 点亮
void 	CarLED_Off(uint8_t id);					// 熄灭
void 	CarLED_Toggle(uint8_t id);				// 翻转
void 	CarLED_AllOff(void);					// 全灭
void 	CarLED_AllOn(void);						// 全亮
uint8_t CarLED_Get(uint8_t id);					// 读取当前状态: 1=点亮 0=熄灭

#endif //__CAR_LED_H

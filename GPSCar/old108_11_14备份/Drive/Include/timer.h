#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

/*---------------------- 微秒时基配置宏 ------------------------*/
/*
 * 使用 TIM2 (32位定时器) 作为 1MHz 自由运行计数器,
 * 为超声波测距、软件I2C等提供微秒级延时与时间戳。
 * TIM2 挂在 APB1(42MHz) 上, 定时器时钟倍频后为 84MHz。
 */

/*---------------------- 函数声明 ----------------------------*/

void 		Timer_Init(void);					// 微秒时基初始化
uint32_t 	Get_Us(void);						// 获取系统运行的微秒数
uint32_t 	Get_Ms(void);						// 获取系统运行的毫秒数
void 		Delay_us(uint32_t nUs);				// 微秒延时

#endif //__TIMER_H

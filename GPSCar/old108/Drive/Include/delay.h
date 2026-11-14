#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void Delay_Init(void);				//延时函数初始化
void Delay_ms(uint32_t nTime);	//毫秒延时函数

/*---------------- 非阻塞延时 ----------------------------*/
/*
 * 基于TIM2微秒时基(timer.h), 使用前需先调用Timer_Init()
 * 每个需要定时的任务定义一个静态 NB_DelayTypeDef 句柄
 */
typedef struct
{
	uint32_t start;		// 起始时间戳
	uint8_t  running;	// 1=计时中
} NB_DelayTypeDef;

void	NB_DelayReset(NB_DelayTypeDef *nb);					// 复位句柄, 重新计时
uint8_t NB_DelayMs(NB_DelayTypeDef *nb, uint32_t nMs);	// 非阻塞毫秒延时, 到期返回1
uint8_t NB_DelayUs(NB_DelayTypeDef *nb, uint32_t nUs);	// 非阻塞微秒延时, 到期返回1

#endif //__DELAY_H

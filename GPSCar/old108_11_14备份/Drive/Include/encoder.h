#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

/*---------------------- 编码器编号 ----------------------------*/
#define ENCODER_1	0		// 电机1编码器  TIM5: PA0/PA1
#define ENCODER_2	1		// 电机2编码器  TIM3: PA6/PA7
#define ENCODER_3	2		// 电机3编码器  TIM4: PB6/PB7
#define ENCODER_4	3		// 电机4编码器  TIM8: PC6/PC7
#define ENCODER_NUM	4

/* 若编码器读数方向与电机正转方向相反, 把对应宏改为 1 */
#define ENCODER1_DIR_FLIP	0
#define ENCODER2_DIR_FLIP	0
#define ENCODER3_DIR_FLIP	0
#define ENCODER4_DIR_FLIP	0

/*---------------------- 函数声明 ----------------------------*/

void 	Encoder_Init(void);							// 编码器初始化
int16_t Encoder_ReadDelta(uint8_t id);				// 读取距上次读取的脉冲增量(带方向)
int32_t Encoder_GetTotal(uint8_t id);				// 读取累计脉冲数
void 	Encoder_ClearTotal(uint8_t id);				// 清零累计脉冲数

#endif //__ENCODER_H

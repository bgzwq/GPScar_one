#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"

/*---------------------- 舵机配置宏 ------------------------*/
/*
 * 舵机PWM: TIM12_CH1 = PB14, 50Hz
 * TIM12 挂在 APB1, 定时器时钟 84MHz
 * 84MHz / 84 = 1MHz -> 计数单位1us, 周期20000us = 50Hz
 */
#define SERVO_TIM			TIM12
#define SERVO_TIM_CLK		RCC_APB1Periph_TIM12
#define SERVO_PIN			GPIO_Pin_14
#define SERVO_PORT			GPIOB
#define SERVO_PinSource		GPIO_PinSource14
#define SERVO_GPIO_CLK		RCC_AHB1Periph_GPIOB
#define SERVO_AF			GPIO_AF_TIM12

#define SERVO_PERIOD_US		20000	// 50Hz
#define SERVO_PULSE_MIN		500		// 0度脉宽(us)
#define SERVO_PULSE_MAX		2500	// 180度脉宽(us)

/*---------------------- 函数声明 ----------------------------*/

void 	Servo_Init(void);						// 舵机初始化, 默认转到90度
void 	Servo_SetAngle(float angle);			// 设置角度 0~180
void 	Servo_SetPulse(uint16_t pulseUs);		// 直接设置脉宽 500~2500us

#endif //__SERVO_H

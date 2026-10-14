#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f4xx.h"

/*---------------------- 蜂鸣器配置宏 ------------------------*/
/*
 * 无源蜂鸣器: TIM11_CH1 = PF6, PWM驱动, 可变音调
 * TIM11 挂在 APB2, 定时器时钟 168MHz
 * 预分频 168 -> 1MHz 计数, 通过改变ARR改变频率
 */
#define BUZZER_TIM			TIM11
#define BUZZER_TIM_CLK		RCC_APB2Periph_TIM11
#define BUZZER_PIN			GPIO_Pin_6
#define BUZZER_PORT			GPIOF
#define BUZZER_PinSource	GPIO_PinSource6
#define BUZZER_GPIO_CLK		RCC_AHB1Periph_GPIOF
#define BUZZER_AF			GPIO_AF_TIM11

/*---------------------- 函数声明 ----------------------------*/

void 	Buzzer_Init(void);							// 蜂鸣器初始化(默认静音)
void 	Buzzer_On(uint16_t freq);					// 以指定频率鸣叫(Hz)
void 	Buzzer_Off(void);							// 静音
void 	Buzzer_Beep(uint16_t freq, uint16_t ms);	// 鸣叫指定时长(阻塞, 只建议开机时用)

/*
 * 非阻塞蜂鸣: Buzzer_BeepAsync() 只是登记"响 freq 共 ms", 立即返回;
 * Buzzer_Task() 需要在主循环里反复调用, 到期自动静音。
 * 原来的 Buzzer_Beep 用 Delay_ms 阻塞, 按键切模式/避障时会把
 * GPS解析、K230组帧、遥控指令一起卡住60~100ms, 周期性提示音请改用这一对。
 */
void 	Buzzer_BeepAsync(uint16_t freq, uint16_t ms);
void 	Buzzer_Task(void);

#endif //__BUZZER_H

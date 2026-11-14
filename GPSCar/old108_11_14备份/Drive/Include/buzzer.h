#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f4xx.h"

/*---------------------- 蜂鸣器配置宏 ------------------------*/
/*
 * 无源蜂鸣器: PF10 推挽输出
 * 注意: PF10 没有定时器复用功能(它是 ADC3_IN8), 不能硬件PWM,
 * 这里用 TIM11 的更新中断按 2倍音调频率翻转 PF10 产生方波(占空比50%)。
 * TIM11 挂在 APB2, 定时器时钟 168MHz, 预分频168 -> 1MHz 计数。
 * 若使用的是有源蜂鸣器, 调用 Buzzer_On(任意频率) 也会响, 只是音调不可变。
 */
#define BUZZER_TIM			TIM11
#define BUZZER_TIM_CLK		RCC_APB2Periph_TIM11
#define BUZZER_IRQn			TIM1_TRG_COM_TIM11_IRQn
#define BUZZER_PIN			GPIO_Pin_10
#define BUZZER_PORT			GPIOF
#define BUZZER_GPIO_CLK		RCC_AHB1Periph_GPIOF

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

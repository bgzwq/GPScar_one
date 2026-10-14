#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

/*---------------------- 电机编号 ----------------------------*/
#define MOTOR_1		0		// 电机1 左前
#define MOTOR_2		1		// 电机2 左后
#define MOTOR_3		2		// 电机3 右前
#define MOTOR_4		3		// 电机4 右后
#define MOTOR_NUM	4

/*---------------------- 电机引脚配置宏 ------------------------*/
/*
 * 4路PWM: TIM1 CH1~CH4 -> PE9 / PE11 / PE13 / PE14  (TIM1在APB2, 定时器时钟168MHz)
 * 方向脚: 普通推挽输出
 *
 *  M1: IN1=PE7  IN2=PE8   PWM=PE9  (TIM1_CH1)
 *  M2: IN1=PE10 IN2=PE12  PWM=PE11 (TIM1_CH2)
 *  M3: IN1=PG6  IN2=PG8   PWM=PE13 (TIM1_CH3)
 *  M4: IN1=PB13 IN2=PB12  PWM=PE14 (TIM1_CH4)
 */
#define MOTOR_PWM_TIM			TIM1
#define MOTOR_PWM_CLK			RCC_APB2Periph_TIM1
#define MOTOR_PWM_AF			GPIO_AF_TIM1
#define MOTOR_PWM_GPIO_CLK		RCC_AHB1Periph_GPIOE
#define MOTOR_PWM_PORT			GPIOE
#define MOTOR_PWM_PINS			(GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14)

/* PWM频率: 168MHz / 16800 = 10kHz */
#define MOTOR_PWM_PERIOD		(16800 - 1)
#define MOTOR_SPEED_MAX			1000	// 速度范围 -1000 ~ +1000 (千分比)

/* 方向引脚 */
#define M1_IN1_PORT		GPIOE
#define M1_IN1_PIN		GPIO_Pin_7
#define M1_IN2_PORT		GPIOE
#define M1_IN2_PIN		GPIO_Pin_8

#define M2_IN1_PORT		GPIOE
#define M2_IN1_PIN		GPIO_Pin_10
#define M2_IN2_PORT		GPIOE
#define M2_IN2_PIN		GPIO_Pin_12

#define M3_IN1_PORT		GPIOG
#define M3_IN1_PIN		GPIO_Pin_6
#define M3_IN2_PORT		GPIOG
#define M3_IN2_PIN		GPIO_Pin_8

#define M4_IN1_PORT		GPIOB
#define M4_IN1_PIN		GPIO_Pin_13
#define M4_IN2_PORT		GPIOB
#define M4_IN2_PIN		GPIO_Pin_12

/* 若某个电机实际转向与预期相反, 把对应宏改为 1 即可软件翻转 */
#define MOTOR1_DIR_FLIP		0
#define MOTOR2_DIR_FLIP		0
#define MOTOR3_DIR_FLIP		0
#define MOTOR4_DIR_FLIP		0

/*---------------------- 函数声明 ----------------------------*/

void 	Motor_Init(void);								// 电机初始化
void 	Motor_SetSpeed(uint8_t id, int16_t speed);		// 设置电机速度 -1000~1000
void 	Motor_Stop(uint8_t id);							// 单个电机制动停止
void 	Motor_StopAll(void);							// 全部电机制动停止
int16_t Motor_GetPwm(uint8_t id);						// 读取当前PWM占空比(千分比 0~1000)

#endif //__MOTOR_H

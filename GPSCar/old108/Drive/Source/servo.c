/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   舵机驱动 (TIM12_CH1 = PB14, 50Hz)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 与超声波组成扫描雷达, 舵机带动超声波左右摆动测距
	*  2. 脉宽 500~2500us 对应 0~180 度, 不同舵机可能需微调 SERVO_PULSE_MIN/MAX
	*
	*  【舵机已整体禁用 —— 供电功率不足】
	*  Servo_Init/Servo_SetPulse/Servo_SetAngle 函数体已注释: 引脚不初始化、
	*  TIM12不开时钟, PB14保持复位态零驱动, 舵机不通电。
	*  上层调用(main.c避障扫描/雷达摆头等)原样保留, 变为空操作。
	*  注意: 禁用后 RADAR 雷达模式和 AVOID 避障的左右扫描没有实际动作,
	*  恢复方法: 取消下面三个函数体内的注释即可, 调用处无需改动。
	*
	************************************************************************************************
***/

#include "servo.h"

/*************************************************************************************************
*	函 数 名:	Servo_Init
*
*	函数功能:	初始化 TIM12_CH1 输出50Hz PWM, 舵机归中90度
*
*************************************************************************************************/
void Servo_Init(void)
{
	/* 已禁用(省电): 不初始化PB14与TIM12, 引脚保持复位态零驱动
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(SERVO_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = SERVO_PIN;
	GPIO_Init(SERVO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(SERVO_PORT, SERVO_PinSource, SERVO_AF);

	TIM_TimeBaseStructure.TIM_Prescaler     = 84 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period        = SERVO_PERIOD_US - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = 1500;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC1Init(SERVO_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);
	TIM_Cmd(SERVO_TIM, ENABLE);
	*/
}

/*************************************************************************************************
*	函 数 名:	Servo_SetPulse
*
*	入口参数:	pulseUs - 脉宽 500~2500us
*
*************************************************************************************************/
void Servo_SetPulse(uint16_t pulseUs)
{
	(void)pulseUs;
	/* 已禁用(省电): 空操作
	if (pulseUs < SERVO_PULSE_MIN) pulseUs = SERVO_PULSE_MIN;
	if (pulseUs > SERVO_PULSE_MAX) pulseUs = SERVO_PULSE_MAX;
	TIM_SetCompare1(SERVO_TIM, pulseUs);
	*/
}

/*************************************************************************************************
*	函 数 名:	Servo_SetAngle
*
*	入口参数:	angle - 角度 0~180
*
*************************************************************************************************/
void Servo_SetAngle(float angle)
{
	(void)angle;
	/* 已禁用(省电): 空操作
	uint16_t pulse;

	if (angle < 0)   angle = 0;
	if (angle > 180) angle = 180;

	pulse = SERVO_PULSE_MIN +
			(uint16_t)(angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180.0f);
	Servo_SetPulse(pulse);
	*/
}

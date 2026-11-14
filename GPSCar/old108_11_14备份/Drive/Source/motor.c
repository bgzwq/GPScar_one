/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   4路编码电机驱动 (TB6612等双极性驱动板)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. TIM1 输出 4 路 10kHz PWM: CH1=PE9, CH2=PE11, CH3=PE13, CH4=PE14
	*  2. 每个电机 2 个方向脚, 正转: IN1=1 IN2=0, 反转: IN1=0 IN2=1, 制动: IN1=1 IN2=1
	*  3. 速度范围 -1000 ~ +1000, 正负代表方向
	*
	************************************************************************************************
***/

#include "motor.h"

/* 方向引脚表, 方便按编号索引 */
typedef struct
{
	GPIO_TypeDef *in1Port;
	uint16_t      in1Pin;
	GPIO_TypeDef *in2Port;
	uint16_t      in2Pin;
	uint8_t       flip;		// 方向翻转标志
} MotorDir_TypeDef;

static const MotorDir_TypeDef s_motorDir[MOTOR_NUM] =
{
	{ M1_IN1_PORT, M1_IN1_PIN, M1_IN2_PORT, M1_IN2_PIN, MOTOR1_DIR_FLIP },
	{ M2_IN1_PORT, M2_IN1_PIN, M2_IN2_PORT, M2_IN2_PIN, MOTOR2_DIR_FLIP },
	{ M3_IN1_PORT, M3_IN1_PIN, M3_IN2_PORT, M3_IN2_PIN, MOTOR3_DIR_FLIP },
	{ M4_IN1_PORT, M4_IN1_PIN, M4_IN2_PORT, M4_IN2_PIN, MOTOR4_DIR_FLIP },
};

/*************************************************************************************************
*	函 数 名:	Motor_GPIO_Init
*
*	函数功能:	初始化全部方向脚与PWM脚
*
*************************************************************************************************/
static void Motor_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE |
						   RCC_AHB1Periph_GPIOG |
						   RCC_AHB1Periph_GPIOB, ENABLE);

	/* 方向脚: 推挽输出 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = M1_IN1_PIN | M1_IN2_PIN | M2_IN1_PIN | M2_IN2_PIN |
								  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = M3_IN1_PIN | M3_IN2_PIN;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = M4_IN1_PIN | M4_IN2_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PWM脚: 复用推挽, AF1(TIM1) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin  = MOTOR_PWM_PINS;
	GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource9,  MOTOR_PWM_AF);
	GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource11, MOTOR_PWM_AF);
	GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource13, MOTOR_PWM_AF);
	GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource14, MOTOR_PWM_AF);
}

/*************************************************************************************************
*	函 数 名:	Motor_Init
*
*	函数功能:	初始化 TIM1 四路PWM与方向脚, 初始状态全部制动
*
*************************************************************************************************/
void Motor_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	Motor_GPIO_Init();

	RCC_APB2PeriphClockCmd(MOTOR_PWM_CLK, ENABLE);

	/* 168MHz / (0+1) / 16800 = 10kHz */
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period        = MOTOR_PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTOR_PWM_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse       = 0;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState= TIM_OCNIdleState_Reset;

	TIM_OC1Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
	TIM_OC4Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(MOTOR_PWM_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR_PWM_TIM, ENABLE);	// 高级定时器必须开启主输出
	TIM_Cmd(MOTOR_PWM_TIM, ENABLE);

	Motor_StopAll();
}

/*************************************************************************************************
*	函 数 名:	Motor_SetSpeed
*
*	入口参数:	id    - 电机编号 MOTOR_1 ~ MOTOR_4
*				speed - 速度 -1000 ~ +1000, 正负代表方向, 0 为滑行
*
*	函数功能:	设置单个电机的转向与PWM占空比
*
*************************************************************************************************/
void Motor_SetSpeed(uint8_t id, int16_t speed)
{
	uint16_t pulse;
	int16_t  dir;

	if (id >= MOTOR_NUM) return;

	if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
	if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

	dir = (speed >= 0) ? 1 : -1;
	if (s_motorDir[id].flip) dir = -dir;	// 软件方向翻转

	if (speed == 0)
	{
		/* 滑行: 两个方向脚都拉低 */
		GPIO_ResetBits(s_motorDir[id].in1Port, s_motorDir[id].in1Pin);
		GPIO_ResetBits(s_motorDir[id].in2Port, s_motorDir[id].in2Pin);
	}
	else if (dir > 0)
	{
		GPIO_SetBits  (s_motorDir[id].in1Port, s_motorDir[id].in1Pin);
		GPIO_ResetBits(s_motorDir[id].in2Port, s_motorDir[id].in2Pin);
	}
	else
	{
		GPIO_ResetBits(s_motorDir[id].in1Port, s_motorDir[id].in1Pin);
		GPIO_SetBits  (s_motorDir[id].in2Port, s_motorDir[id].in2Pin);
	}

	/* 占空比 = |speed|/1000 * 周期 */
	pulse = (uint16_t)((uint32_t)((speed >= 0) ? speed : -speed) * (MOTOR_PWM_PERIOD + 1) / MOTOR_SPEED_MAX);

	switch (id)
	{
		case MOTOR_1: TIM_SetCompare1(MOTOR_PWM_TIM, pulse); break;
		case MOTOR_2: TIM_SetCompare2(MOTOR_PWM_TIM, pulse); break;
		case MOTOR_3: TIM_SetCompare3(MOTOR_PWM_TIM, pulse); break;
		case MOTOR_4: TIM_SetCompare4(MOTOR_PWM_TIM, pulse); break;
		default: break;
	}
}

/*************************************************************************************************
*	函 数 名:	Motor_Stop
*
*	入口参数:	id - 电机编号
*
*	函数功能:	制动停止 (IN1=IN2=1, PWM=0)
*
*************************************************************************************************/
void Motor_Stop(uint8_t id)
{
	if (id >= MOTOR_NUM) return;

	GPIO_SetBits(s_motorDir[id].in1Port, s_motorDir[id].in1Pin);
	GPIO_SetBits(s_motorDir[id].in2Port, s_motorDir[id].in2Pin);

	switch (id)
	{
		case MOTOR_1: TIM_SetCompare1(MOTOR_PWM_TIM, 0); break;
		case MOTOR_2: TIM_SetCompare2(MOTOR_PWM_TIM, 0); break;
		case MOTOR_3: TIM_SetCompare3(MOTOR_PWM_TIM, 0); break;
		case MOTOR_4: TIM_SetCompare4(MOTOR_PWM_TIM, 0); break;
		default: break;
	}
}

/*************************************************************************************************
*	函 数 名:	Motor_StopAll
*
*	函数功能:	全部电机制动停止
*
*************************************************************************************************/
void Motor_StopAll(void)
{
	uint8_t i;
	for (i = 0; i < MOTOR_NUM; i++)
	{
		Motor_Stop(i);
	}
}

/*************************************************************************************************
*	函 数 名:	Motor_GetPwm
*
*	入口参数:	id - 电机编号
*
*	返 回 值:	当前PWM占空比(千分比 0~1000), 不含方向
*
*	函数功能:	回读TIM1比较寄存器, 供菜单显示/诊断
*
*************************************************************************************************/
int16_t Motor_GetPwm(uint8_t id)
{
	uint16_t pulse;

	switch (id)
	{
		case MOTOR_1: pulse = TIM_GetCapture1(MOTOR_PWM_TIM); break;
		case MOTOR_2: pulse = TIM_GetCapture2(MOTOR_PWM_TIM); break;
		case MOTOR_3: pulse = TIM_GetCapture3(MOTOR_PWM_TIM); break;
		case MOTOR_4: pulse = TIM_GetCapture4(MOTOR_PWM_TIM); break;
		default: return 0;
	}

	return (int16_t)((uint32_t)pulse * MOTOR_SPEED_MAX / (MOTOR_PWM_PERIOD + 1));
}

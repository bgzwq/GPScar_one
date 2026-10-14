	/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   4路编码器接口 (定时器编码器模式)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 编码器A/B相接入同一定时器的 CH1/CH2, 使用 TI12 正交解码(4倍频)
	*     ENC1: TIM5 CH1/CH2 = PA0/PA1
	*     ENC2: TIM3 CH1/CH2 = PA6/PA7
	*     ENC3: TIM4 CH1/CH2 = PB6/PB7
	*     ENC4: TIM8 CH1/CH2 = PC6/PC7
	*  2. 计数周期统一设为 0xFFFF, 增量按 int16 回绕差值计算
	*  3. Encoder_ReadDelta() 需周期性调用(如每10~20ms), 间隔不能超过计数器回绕时间
	*
	************************************************************************************************
***/

#include "encoder.h"

typedef struct
{
	TIM_TypeDef *tim;
	uint32_t     timClk;
	GPIO_TypeDef *port;
	uint16_t     pinA;
	uint16_t     pinB;
	uint8_t      pinSrcA;
	uint8_t      pinSrcB;
	uint8_t      af;
	uint32_t     gpioClk;
	uint8_t      flip;
	uint8_t      isApb2;	// 定时器是否挂在APB2
} EncoderHW_TypeDef;

static const EncoderHW_TypeDef s_encHW[ENCODER_NUM] =
{
	/* ENC1: TIM5 PA0/PA1, AF2, APB1 */
	{ TIM5, RCC_APB1Periph_TIM5, GPIOA, GPIO_Pin_0, GPIO_Pin_1,
	  GPIO_PinSource0, GPIO_PinSource1, GPIO_AF_TIM5, RCC_AHB1Periph_GPIOA, ENCODER1_DIR_FLIP, 0 },
	/* ENC2: TIM3 PA6/PA7, AF2, APB1 */
	{ TIM3, RCC_APB1Periph_TIM3, GPIOA, GPIO_Pin_6, GPIO_Pin_7,
	  GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_TIM3, RCC_AHB1Periph_GPIOA, ENCODER2_DIR_FLIP, 0 },
	/* ENC3: TIM4 PB6/PB7, AF2, APB1 */
	{ TIM4, RCC_APB1Periph_TIM4, GPIOB, GPIO_Pin_6, GPIO_Pin_7,
	  GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_TIM4, RCC_AHB1Periph_GPIOB, ENCODER3_DIR_FLIP, 0 },
	/* ENC4: TIM8 PC6/PC7, AF3, APB2 */
	{ TIM8, RCC_APB2Periph_TIM8, GPIOC, GPIO_Pin_6, GPIO_Pin_7,
	  GPIO_PinSource6, GPIO_PinSource7, GPIO_AF_TIM8, RCC_AHB1Periph_GPIOC, ENCODER4_DIR_FLIP, 1 },
};

static int32_t s_total[ENCODER_NUM];	// 累计脉冲数

/*************************************************************************************************
*	函 数 名:	Encoder_Init
*
*	函数功能:	初始化 4 个定时器为正交编码器模式
*
*************************************************************************************************/
void Encoder_Init(void)
{
	uint8_t i;
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;

	for (i = 0; i < ENCODER_NUM; i++)
	{
		/* GPIO时钟与定时器时钟 */
		RCC_AHB1PeriphClockCmd(s_encHW[i].gpioClk, ENABLE);
		if (s_encHW[i].isApb2)
			RCC_APB2PeriphClockCmd(s_encHW[i].timClk, ENABLE);
		else
			RCC_APB1PeriphClockCmd(s_encHW[i].timClk, ENABLE);

		/* A/B相: 复用输入, 上拉 */
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Pin   = s_encHW[i].pinA | s_encHW[i].pinB;
		GPIO_Init(s_encHW[i].port, &GPIO_InitStructure);

		GPIO_PinAFConfig(s_encHW[i].port, s_encHW[i].pinSrcA, s_encHW[i].af);
		GPIO_PinAFConfig(s_encHW[i].port, s_encHW[i].pinSrcB, s_encHW[i].af);

		/* 时基: 向上计数, 周期0xFFFF */
		TIM_TimeBaseStructure.TIM_Prescaler     = 0;
		TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(s_encHW[i].tim, &TIM_TimeBaseStructure);

		/* 编码器模式: TI1与TI2双边沿计数(4倍频) */
		TIM_EncoderInterfaceConfig(s_encHW[i].tim, TIM_EncoderMode_TI12,
								   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

		/* 输入捕获滤波, 抑制毛刺 */
		TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter    = 0x0F;
		TIM_ICInit(s_encHW[i].tim, &TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
		TIM_ICInit(s_encHW[i].tim, &TIM_ICInitStructure);

		TIM_SetCounter(s_encHW[i].tim, 0);
		TIM_Cmd(s_encHW[i].tim, ENABLE);

		s_total[i] = 0;
	}
}

/*************************************************************************************************
*	函 数 名:	Encoder_ReadDelta
*
*	入口参数:	id - 编码器编号
*
*	返 回 值:	距上次调用之间的脉冲增量(带方向), 同时累加到总计数
*
*	函数功能:	以回绕差值方式读取编码器增量, 需周期性调用
*
*************************************************************************************************/
int16_t Encoder_ReadDelta(uint8_t id)
{
	static uint16_t s_last[ENCODER_NUM];
	uint16_t now;
	int16_t  delta;

	if (id >= ENCODER_NUM) return 0;

	now   = (uint16_t)TIM_GetCounter(s_encHW[id].tim);
	delta = (int16_t)(now - s_last[id]);	// 16位回绕差值即增量
	s_last[id] = now;

	if (s_encHW[id].flip) delta = -delta;

	s_total[id] += delta;
	return delta;
}

/*************************************************************************************************
*	函 数 名:	Encoder_GetTotal
*
*	入口参数:	id - 编码器编号
*
*	返 回 值:	累计脉冲数 (依赖 Encoder_ReadDelta 周期调用)
*
*************************************************************************************************/
int32_t Encoder_GetTotal(uint8_t id)
{
	if (id >= ENCODER_NUM) return 0;
	return s_total[id];
}

/*************************************************************************************************
*	函 数 名:	Encoder_ClearTotal
*
*	入口参数:	id - 编码器编号
*
*************************************************************************************************/
void Encoder_ClearTotal(uint8_t id)
{
	if (id >= ENCODER_NUM) return;
	s_total[id] = 0;
}

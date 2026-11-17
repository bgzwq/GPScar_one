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
#include <stdio.h>		/* Encoder_DiagDump() 用的 printf */

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
		/* 输入捕获滤波, 抑制毛刺。
		 * 【2026-11 调整】原来 0x0F = 连续 8 次采样一致才认边沿(fDTS/32),
		 * 对"集电极开路输出 + 依赖 MCU 内部弱上拉"的编码器来说边沿偏慢,
		 * 慢沿会被整段滤掉 -> 表现为"某几路完全没示数"。
		 * 改成 0x03(连续 4 次, fDTS/16): 保留去毛刺, 又不会吃掉慢边沿。
		 * 若线路干扰大、读数偶发跳变, 可以往回调大(0x07 / 0x0B)。 */
		TIM_ICInitStructure.TIM_ICFilter    = 0x03;
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
/*************************************************************************************************
*	函 数 名:	Encoder_DiagDump
*
*	函数功能:	【诊断】把 4 路编码器相关寄存器一次性打印出来
*
*	说    明:	某一路"转轮子没示数"时, 这一条打印可以一次分清三类原因:
*				  1) 信号没进 MCU / 引脚被板载外设占用 -> IDR 对应位不跟轮子变
*				  2) 引脚复用没生效(被别处重配)        -> MODER/AFR0 不是 AF2
*				  3) 定时器没跑 / 没配成编码器模式      -> CR1.CEN=0 或 SMCR.SMS!=3
*				调用: 电机自检(KEY4)阶段3每秒自动打印一次, 也可自己挂到串口命令。
*
*	期 望 值(拿正常工作的 TIM4/TIM8 当参照对比):
*				GPIOA->MODER : PA0/PA1/PA6/PA7 的 2bit = 10 (AF)
*				GPIOA->AFR[0]: 第 0/1/6/7 个半字节 = 2 (AF2)
*				TIMx->CR1    : bit0(CEN)=1
*				TIMx->SMCR   : SMS[2:0]=011 (编码器 TI1+TI2 双边沿)
*				TIMx->CCMR1  : CC1S[1:0]=01, CC2S[9:8]=01
*				TIMx->CCER   : bit0(CC1E)=1, bit4(CC2E)=1
*
*************************************************************************************************/
void Encoder_DiagDump(void)
{
	printf("\r\n[ENC-DIAG] GPIOA MODER=%08lX AFR0=%08lX IDR=%08lX\r\n",
		   (unsigned long)GPIOA->MODER, (unsigned long)GPIOA->AFR[0],
		   (unsigned long)GPIOA->IDR);
	printf("[ENC-DIAG] PA0/PA1/PA6/PA7 = %u %u %u %u  (手转轮子时应看到跳变)\r\n",
		   (unsigned)((GPIOA->IDR >> 0) & 1u), (unsigned)((GPIOA->IDR >> 1) & 1u),
		   (unsigned)((GPIOA->IDR >> 6) & 1u), (unsigned)((GPIOA->IDR >> 7) & 1u));
	printf("[ENC-DIAG] TIM5 CNT=%5lu CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04lX\r\n",
		   (unsigned long)TIM5->CNT, (unsigned long)TIM5->CR1, (unsigned long)TIM5->SMCR,
		   (unsigned long)TIM5->CCMR1, (unsigned long)TIM5->CCER);
	printf("[ENC-DIAG] TIM3 CNT=%5lu CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04lX\r\n",
		   (unsigned long)TIM3->CNT, (unsigned long)TIM3->CR1, (unsigned long)TIM3->SMCR,
		   (unsigned long)TIM3->CCMR1, (unsigned long)TIM3->CCER);
	printf("[ENC-DIAG] TIM4 CNT=%5lu CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04lX\r\n",
		   (unsigned long)TIM4->CNT, (unsigned long)TIM4->CR1, (unsigned long)TIM4->SMCR,
		   (unsigned long)TIM4->CCMR1, (unsigned long)TIM4->CCER);
	printf("[ENC-DIAG] TIM8 CNT=%5lu CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04lX\r\n",
		   (unsigned long)TIM8->CNT, (unsigned long)TIM8->CR1, (unsigned long)TIM8->SMCR,
		   (unsigned long)TIM8->CCMR1, (unsigned long)TIM8->CCER);
	printf("[ENC-DIAG] RCC APB1ENR=%08lX(bit1=TIM3 bit2=TIM4 bit3=TIM5) APB2ENR=%08lX(bit0=TIM8)\r\n",
		   (unsigned long)RCC->APB1ENR, (unsigned long)RCC->APB2ENR);
	printf("[ENC-DIAG] ACC: ENC1=%ld ENC2=%ld ENC3=%ld ENC4=%ld\r\n",
		   (long)s_total[0], (long)s_total[1], (long)s_total[2], (long)s_total[3]);
}

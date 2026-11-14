/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   HC-SR04 超声波测距 (雷达位 + 固定位)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. TRIG 输出 10us 高电平触发, ECHO 高电平宽度与距离成正比
	*     距离(cm) = 高电平时间(us) / 58
	*  2. 采用轮询方式测量, 单次最长阻塞约 30ms, 主循环中两路交替调用即可
	*  3. 依赖 timer.c 提供的 1MHz 微秒时基
	*
	************************************************************************************************
***/

#include "ultrasonic.h"
#include "timer.h"

typedef struct
{
	GPIO_TypeDef *trigPort;
	uint16_t      trigPin;
	GPIO_TypeDef *echoPort;
	uint16_t      echoPin;
	uint32_t      trigClk;
	uint32_t      echoClk;
} UltraHW_TypeDef;

static const UltraHW_TypeDef s_ultraHW[ULTRA_NUM] =
{
	/* 雷达位: TRIG=PB15, ECHO=PD14 */
	{ GPIOB, GPIO_Pin_15, GPIOD, GPIO_Pin_14, RCC_AHB1Periph_GPIOB, RCC_AHB1Periph_GPIOD },
	/* 固定位: TRIG=PD0,  ECHO=PD1 */
	{ GPIOD, GPIO_Pin_0,  GPIOD, GPIO_Pin_1,  RCC_AHB1Periph_GPIOD, RCC_AHB1Periph_GPIOD },
};

/*************************************************************************************************
*	函 数 名:	Ultrasonic_Init
*
*	函数功能:	初始化 TRIG(推挽输出) 与 ECHO(下拉输入)
*
*************************************************************************************************/
void Ultrasonic_Init(void)
{
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStructure;

	for (i = 0; i < ULTRA_NUM; i++)
	{
		RCC_AHB1PeriphClockCmd(s_ultraHW[i].trigClk | s_ultraHW[i].echoClk, ENABLE);

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Pin   = s_ultraHW[i].trigPin;
		GPIO_Init(s_ultraHW[i].trigPort, &GPIO_InitStructure);
		GPIO_ResetBits(s_ultraHW[i].trigPort, s_ultraHW[i].trigPin);

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Pin   = s_ultraHW[i].echoPin;
		GPIO_Init(s_ultraHW[i].echoPort, &GPIO_InitStructure);
	}
}

/*************************************************************************************************
*	函 数 名:	Ultrasonic_GetDistance
*
*	入口参数:	id - ULTRA_RADAR(舵机位) / ULTRA_FIXED(固定位)
*
*	返 回 值:	距离, 单位cm; 超时/超量程返回 ULTRA_MAX_CM
*
*	函数功能:	触发一次测距并计算距离
*
*************************************************************************************************/
float Ultrasonic_GetDistance(Ultrasonic_TypeDef id)
{
	uint32_t start, highTime;

	if (id >= ULTRA_NUM) return ULTRA_MAX_CM;

	/* 触发: 至少10us高电平 */
	GPIO_SetBits(s_ultraHW[id].trigPort, s_ultraHW[id].trigPin);
	Delay_us(15);
	GPIO_ResetBits(s_ultraHW[id].trigPort, s_ultraHW[id].trigPin);

	/* 等待ECHO拉高 */
	start = Get_Us();
	while (GPIO_ReadInputDataBit(s_ultraHW[id].echoPort, s_ultraHW[id].echoPin) == 0)
	{
		if ((uint32_t)(Get_Us() - start) > ULTRA_TIMEOUT_US) return ULTRA_MAX_CM;
	}

	/* 测量高电平时间 */
	start = Get_Us();
	while (GPIO_ReadInputDataBit(s_ultraHW[id].echoPort, s_ultraHW[id].echoPin) == 1)
	{
		if ((uint32_t)(Get_Us() - start) > ULTRA_TIMEOUT_US) return ULTRA_MAX_CM;
	}
	highTime = Get_Us() - start;

	/* 距离(cm) = 时间(us) / 58 */
	return (float)highTime / 58.0f;
}

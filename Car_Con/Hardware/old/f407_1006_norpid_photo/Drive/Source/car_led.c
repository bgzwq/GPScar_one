/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   车身4路LED (PG0~PG3)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 推挽输出, 默认低电平点亮, 可通过 CAR_LED_ACTIVE_LOW 修改极性
	*  2. 可用于转向灯、状态指示等
	*
	************************************************************************************************
***/

#include "car_led.h"

static const uint16_t s_ledPin[CAR_LED_NUM] =
{
	GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3
};

static void CarLED_Write(uint8_t id, uint8_t on)
{
#if CAR_LED_ACTIVE_LOW
	if (on) GPIO_ResetBits(GPIOG, s_ledPin[id]);
	else    GPIO_SetBits(GPIOG, s_ledPin[id]);
#else
	if (on) GPIO_SetBits(GPIOG, s_ledPin[id]);
	else    GPIO_ResetBits(GPIOG, s_ledPin[id]);
#endif
}

/*************************************************************************************************
*	函 数 名:	CarLED_Init
*
*	函数功能:	初始化 PG0~PG3 为推挽输出, 默认全灭
*
*************************************************************************************************/
void CarLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	CarLED_AllOff();
}

void CarLED_On(uint8_t id)
{
	if (id >= CAR_LED_NUM) return;
	CarLED_Write(id, 1);
}

void CarLED_Off(uint8_t id)
{
	if (id >= CAR_LED_NUM) return;
	CarLED_Write(id, 0);
}

void CarLED_Toggle(uint8_t id)
{
	if (id >= CAR_LED_NUM) return;
	GPIO_ToggleBits(GPIOG, s_ledPin[id]);
}

void CarLED_AllOff(void)
{
	uint8_t i;
	for (i = 0; i < CAR_LED_NUM; i++) CarLED_Write(i, 0);
}

void CarLED_AllOn(void)
{
	uint8_t i;
	for (i = 0; i < CAR_LED_NUM; i++) CarLED_Write(i, 1);
}

/*************************************************************************************************
*	函 数 名:	CarLED_Get
*
*	入口参数:	id - LED编号
*
*	返 回 值:	1 - 点亮, 0 - 熄灭
*
*	函数功能:	回读输出寄存器, 供菜单显示当前状态
*
*************************************************************************************************/
uint8_t CarLED_Get(uint8_t id)
{
	uint8_t bit;

	if (id >= CAR_LED_NUM) return 0;

	bit = GPIO_ReadOutputDataBit(GPIOG, s_ledPin[id]);
#if CAR_LED_ACTIVE_LOW
	return bit ? 0 : 1;
#else
	return bit ? 1 : 0;
#endif
}

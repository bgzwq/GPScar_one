#include "estop.h"
#include "car.h"
#include "timer.h"
#include "delay.h"

#define ESTOP_PORT      GPIOE
#define ESTOP_PIN       GPIO_Pin_0
#define ESTOP_CLK       RCC_AHB1Periph_GPIOE
#define ESTOP_STBY_PORT GPIOB
#define ESTOP_STBY_PIN  GPIO_Pin_1
#define ESTOP_STBY_CLK  RCC_AHB1Periph_GPIOB

static uint8_t s_active;
static uint8_t s_source;
static uint8_t s_lowCount;
static uint32_t s_count;
static uint8_t  s_stopped;		// 1=本次急停已经停过车(避免每5ms重复写TIM1比较寄存器)

void Estop_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(ESTOP_CLK | ESTOP_STBY_CLK, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = ESTOP_PIN;
    GPIO_Init(ESTOP_PORT, &gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Pin = ESTOP_STBY_PIN;
    GPIO_Init(ESTOP_STBY_PORT, &gpio);
    GPIO_ResetBits(ESTOP_STBY_PORT, ESTOP_STBY_PIN);
    Delay_ms(2);
    GPIO_SetBits(ESTOP_STBY_PORT, ESTOP_STBY_PIN);
}

void Estop_Request(uint8_t source)
{
    if (!s_active) s_count++;
	s_stopped = 1;
    s_active = 1;
    s_source |= source;
    Car_Stop();
    GPIO_ResetBits(ESTOP_STBY_PORT, ESTOP_STBY_PIN);
}

uint8_t Estop_Reset(uint8_t source)
{
    (void)source;
    if (GPIO_ReadInputDataBit(ESTOP_PORT, ESTOP_PIN) == Bit_RESET) return 0;
    s_active = 0;
    s_source = 0;
	s_stopped = 0;
    s_lowCount = 0;
    GPIO_SetBits(ESTOP_STBY_PORT, ESTOP_STBY_PIN);
    return 1;
}

void Estop_Task(uint32_t now)
{
    (void)now;
    if (GPIO_ReadInputDataBit(ESTOP_PORT, ESTOP_PIN) == Bit_RESET)
    {
        if (s_lowCount < 3) s_lowCount++;
        if (s_lowCount >= 3) Estop_Request(ESTOP_SRC_GPIO);
    }
    else s_lowCount = 0;
	if (s_active)
	{
		if (!s_stopped)			/* 只停一次: 之后每5ms只保证 STBY 是低的 */
		{
			s_stopped = 1;
			Car_Stop();
		}
		GPIO_ResetBits(ESTOP_STBY_PORT, ESTOP_STBY_PIN);
	}
}

uint8_t Estop_IsActive(void) { return s_active; }
uint8_t Estop_GetSource(void) { return s_source; }
uint32_t Estop_GetCount(void) { return s_count; }

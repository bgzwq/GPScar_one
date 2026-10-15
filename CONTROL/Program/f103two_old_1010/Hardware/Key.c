#include "stm32f10x.h"
#include "Key.h"

/**
  * 按键模块
  * PB12~PB15 上拉输入，按下为低电平
  * 需要以固定周期（建议10~20ms）调用Key_Scan进行软件消抖
  */

static uint8_t Key_State;		//消抖后的按键状态位图，1：按下
static uint8_t Key_Event;		//按键按下事件位图（边沿），读后自动清零
static uint8_t Key_LastRaw;		//上一次扫描的原始电平位图
static uint8_t Key_StableRaw;	//消抖确认中的稳定电平位图
static uint8_t Key_Count;		//消抖计数

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * 函    数：读取4个按键的原始电平，转换为位图
  * 参    数：无
  * 返 回 值：原始按键位图，对应位为1表示按下（低电平）
  */
static uint8_t Key_ReadRaw(void)
{
	uint8_t Raw = 0;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0) {Raw |= KEY_1;}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) {Raw |= KEY_2;}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0) {Raw |= KEY_3;}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0) {Raw |= KEY_4;}
	return Raw;
}

/**
  * 函    数：按键扫描（消抖），需以固定周期调用
  * 参    数：无
  * 返 回 值：无
  * 说    明：连续2次扫描电平一致才确认状态变化
  */
void Key_Scan(void)
{
	uint8_t Raw = Key_ReadRaw();
	
	if (Raw != Key_LastRaw)
	{
		/*电平发生变化，重新开始消抖计数*/
		Key_LastRaw = Raw;
		Key_Count = 0;
	}
	else if (Raw != Key_StableRaw)
	{
		/*电平持续稳定，计数2次（约20~40ms）后确认*/
		Key_Count ++;
		if (Key_Count >= 2)
		{
			Key_Count = 0;
			Key_Event |= Raw & ~Key_StableRaw;	//记录按下沿事件
			Key_StableRaw = Raw;
			Key_State = Raw;
		}
	}
	else
	{
		Key_Count = 0;
	}
}

/**
  * 函    数：获取消抖后的按键状态位图
  * 参    数：无
  * 返 回 值：按键状态位图，对应位为1表示当前按住
  */
uint8_t Key_GetState(void)
{
	return Key_State;
}

/**
  * 函    数：获取按键按下事件位图（边沿触发，读后清零）
  * 参    数：无
  * 返 回 值：按键事件位图，对应位为1表示刚刚按下
  */
uint8_t Key_GetEvent(void)
{
	uint8_t Event = Key_Event;
	Key_Event = 0;
	return Event;
}

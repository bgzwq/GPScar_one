#include "stm32f10x.h"
#include "AD.h"

/**
  * 摇杆ADC采集模块
  * 本遥控器只使用1个摇杆：PA0~PA1 对应 ADC1_IN0~IN1
  * 扫描模式 + 连续转换 + DMA循环搬运
  * 原始值范围：0~4095
  */

/*DMA循环搬运的目标数组，ADC转换结果实时更新*/
uint16_t AD_Value[2];

/**
  * 函    数：AD初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：初始化后，AD_Value数组自动实时刷新，无需手动触发转换
  */
void AD_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*ADC时钟分频，72MHz / 6 = 12MHz（不超过14MHz上限）*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	/*配置PA0~PA1为模拟输入*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*ADC1配置：独立模式、扫描模式、连续转换、软件触发*/
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*规则组通道配置：IN0~IN1依次转换*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	
	/*DMA1通道1配置：ADC1 -> AD_Value，循环模式*/
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/*使能DMA和ADC*/
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	
	/*ADC校准*/
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	
	/*软件触发，启动连续转换*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * 函    数：获取指定通道的ADC原始值
  * 参    数：Channel 通道索引，范围：AD_J1X~AD_J1Y
  * 返 回 值：ADC原始值，范围：0~4095
  */
uint16_t AD_GetValue(uint8_t Channel)
{
	if (Channel > 1) {return 0;}
	return AD_Value[Channel];
}

/**
  * 函    数：获取指定通道映射后的值
  * 参    数：Channel 通道索引，范围：AD_J1X~AD_J1Y
  * 返 回 值：映射后的值，范围：0~255，中位带死区（120~135归一为128）
  */
uint8_t AD_GetMapped(uint8_t Channel)
{
	uint8_t Value;
	if (Channel > 1) {return 128;}
	
	Value = AD_Value[Channel] >> 4;		//4095 -> 255
	
	/*中位死区处理，消除摇杆中位抖动*/
	if (Value >= 120 && Value <= 135)
	{
		Value = 128;
	}
	return Value;
}

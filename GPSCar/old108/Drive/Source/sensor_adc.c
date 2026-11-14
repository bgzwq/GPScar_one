/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   电源/亮度/湿度 ADC采样 (ADC1 + DMA循环扫描)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. ADC1 扫描模式 + DMA2 Stream0 Channel0 循环传输, 后台自动刷新4个通道
	*  2. 通道顺序(Rank1~4): PA3(IN3) PA4(IN4) PB0(IN8) PC4(IN14)
	*  3. 采样时间144周期, 适合电源分压/光敏/湿敏等慢变信号
	*  4. 注意: PB0/PC4 仅支持 ADC1/ADC2, 本驱动固定使用 ADC1
	*
	************************************************************************************************
***/

#include "sensor_adc.h"

static volatile uint16_t s_adcBuf[ADC_CH_NUM];	// DMA目标缓冲, 顺序与Rank一致

/*************************************************************************************************
*	函 数 名:	SensorADC_Init
*
*	函数功能:	初始化GPIO、ADC1扫描模式与DMA循环传输
*
*************************************************************************************************/
void SensorADC_Init(void)
{
	GPIO_InitTypeDef       GPIO_InitStructure;
	ADC_InitTypeDef        ADC_InitStructure;
	ADC_CommonInitTypeDef  ADC_CommonInitStructure;
	DMA_InitTypeDef        DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
						   RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	/* 模拟输入, 无上下拉 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;		// PA3 PA4
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;					// PB0
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;					// PC4
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* DMA2 Stream0 Channel0 -> ADC1 */
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel            = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_adcBuf;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize         = ADC_CH_NUM;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* ADC通用配置: 独立模式, PCLK2/4 = 21MHz */
	ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1: 扫描+连续转换, 12位 */
	ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode         = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion      = ADC_CH_NUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* 扫描顺序必须与 SensorADC_TypeDef 枚举一致 */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  1, ADC_SampleTime_144Cycles);	// PA3 电源1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  2, ADC_SampleTime_144Cycles);	// PA4 电源2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  3, ADC_SampleTime_144Cycles);	// PB0 亮度
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_144Cycles);	// PC4 湿度

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);
}

/*************************************************************************************************
*	函 数 名:	SensorADC_GetRaw
*
*	入口参数:	ch - 通道枚举
*
*	返 回 值:	ADC原始值 0~4095
*
*************************************************************************************************/
uint16_t SensorADC_GetRaw(SensorADC_TypeDef ch)
{
	if (ch >= ADC_CH_NUM) return 0;
	return s_adcBuf[ch];
}

/*************************************************************************************************
*	函 数 名:	SensorADC_GetVoltage
*
*	入口参数:	ch - 通道枚举
*
*	返 回 值:	引脚电压, 单位V
*
*	说    明:	若电源检测经分压电阻接入, 需自行乘上分压比
*
*************************************************************************************************/
float SensorADC_GetVoltage(SensorADC_TypeDef ch)
{
	return SensorADC_GetRaw(ch) * ADC_REF_VOLTAGE / ADC_FULL_SCALE;
}

/***
	***************************************************************************
	*	@file  	stm32f4xx_it.c
	*	@brief   中断服务函数
   ***************************************************************************
   *  @description
	*
	*  此文件存放了STM32所有的异常操作和中断服务函数
	* 	
	***************************************************************************
***/
#include "stm32f4xx_it.h"
#include "sdio_sd.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


extern void TimingDelay_Decrement(void);	//函数声明，定义在delay.c

//	函数：SysTick中断服务处理
//
void SysTick_Handler(void)
{
	TimingDelay_Decrement();

}



/******************************************************************************/
/*                				                      									*/
/*  								STM32F4xx 外设中断服务函数 									*/
/*  																									*/
/******************************************************************************/

/*************************************************************************************************
*	函 数 名:	SDIO_IRQHandler / DMA2_Stream3_IRQHandler
*
*	函数功能:	SD卡(SDIO)传输完成中断与它的DMA中断
*
*	说    明:	【重要】这两个函数名在启动文件 startup_stm32f40_41xxx.s 里是 [WEAK]
*				的 Default_Handler(内容是一条 "B ." 死循环)。而 FatFs 底层
*				sdio_sd.c 在执行 SD_Init() 以及块读写时会:
*				  1) SDIO_ITConfig(... DATAEND/DCRCFAIL/DTIMEOUT/RXOVERR ...)
*				  2) NVIC 使能 SDIO_IRQn 和 DMA2_Stream3_IRQn
*				一旦中断触发而这里没有同名函数, CPU 就会跳进 Default_Handler
*				死循环 —— 表现为"一读写SD卡整车就卡死"。
*				所以这两个中断必须实现, 分别把事件转交给 sdio_sd.c 里的
*				SD_ProcessIRQSrc() / SD_ProcessDMAIRQ() 去置位传输完成标志。
*
*************************************************************************************************/
void SDIO_IRQHandler(void)
{
	SD_ProcessIRQSrc();
}

void DMA2_Stream3_IRQHandler(void)
{
	SD_ProcessDMAIRQ();
}






/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

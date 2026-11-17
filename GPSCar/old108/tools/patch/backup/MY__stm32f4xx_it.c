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
#include <stdio.h>

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
	/* [2026-11] 原来只是 while(1), 现场什么都看不到。
	 * 现在把 Cortex-M4 的故障寄存器打出来(用当前 MSP 的 ISR 栈, 4KB 够用),
	 * 然后把 PC/LR 也留一份在静态变量里, 调试器停在这里就能看到。
	 * 常见判读: CFSR 的 bit[7:0]=MMFSR(存储器), bit[15:8]=BFSR(总线),
	 *           bit[31:16]=UFSR(用法故障); HFSR bit30=FORCED; MMFAR/BFAR=出错地址。
	 * 注意: 打印完仍然死循环, 保留现场给调试器, 不做自动复位。 */
	volatile uint32_t cfsr  = SCB->CFSR;
	volatile uint32_t hfsr  = SCB->HFSR;
	volatile uint32_t mmfar = SCB->MMFAR;
	volatile uint32_t bfar  = SCB->BFAR;

	printf("\r\n[HARDFAULT] CFSR=%08lX HFSR=%08lX MMFAR=%08lX BFAR=%08lX\r\n",
		   (unsigned long)cfsr, (unsigned long)hfsr,
		   (unsigned long)mmfar, (unsigned long)bfar);
	printf("[HARDFAULT] (MMFSR=%02lX BFSR=%02lX UFSR=%04lX)\r\n",
		   (unsigned long)(cfsr & 0xFFu), (unsigned long)((cfsr >> 8) & 0xFFu),
		   (unsigned long)((cfsr >> 16) & 0xFFFFu));

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

/*----------------------------------------------------------------------------*/
/*  [RTOS 兼容改动] SVC_Handler / PendSV_Handler / SysTick_Handler 三个空函数    */
/*  已删除, 改由 FreeRTOS 移植层 port.c 提供(启动文件里它们是 [WEAK], 删除后会    */
/*  自动落到 port.o 的强符号)。对应改名在 MY/FreeRTOSConfig.h:                   */
/*      #define vPortSVCHandler    SVC_Handler                                 */
/*      #define xPortPendSVHandler PendSV_Handler                              */
/*      #define xPortSysTickHandler SysTick_Handler                             */
/*  保留同名空函数会与 port.o 撞成 L6200E 重复定义; 把 SysTick 留在本文件里则会  */
/*  让 FreeRTOS 拿不到节拍(vTaskDelay 永不返回)。                                */
/*----------------------------------------------------------------------------*/


/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
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

/***
	*************************************************************************************************
	*	@version V1.0
	*	@author  鹿小锟斤拷萍锟?
	*	@brief   usart 锟接匡拷锟斤拷睾锟斤拷锟?
   *************************************************************************************************
   *  @description
	*
	*	实锟斤拷平台锟斤拷鹿小锟斤拷STM32F407ZGT6锟斤拷锟侥帮拷 锟斤拷锟酵号ｏ拷LXB407ZG-P1锟斤拷
	* 锟酵凤拷微锟脚ｏ拷19949278543
	*
>>>>> 锟侥硷拷说锟斤拷锟斤拷
	*
	*  1.锟斤拷始锟斤拷USART1锟斤拷锟斤拷锟斤拷 PA9/PA10锟斤拷
	*  2.锟斤拷锟斤拷USART1锟斤拷锟斤拷锟斤拷锟秸凤拷模式锟斤拷锟斤拷位8位锟斤拷停止位1位锟斤拷锟斤拷校锟介、锟斤拷使锟斤拷硬锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡ｏ拷
	*    锟斤拷锟节的诧拷锟斤拷锟斤拷锟斤拷锟斤拷为115200锟斤拷锟斤拷锟斤拷要锟斤拷锟侥诧拷锟斤拷锟斤拷直锟斤拷锟睫革拷usart.h锟斤拷暮甓拷锟経SART1_BaudRate锟斤拷
	*  3.锟截讹拷锟斤拷fputc锟斤拷锟斤拷,锟斤拷锟斤拷支锟斤拷使锟斤拷printf锟斤拷锟斤拷锟斤拷印锟斤拷锟斤拷
	*
	************************************************************************************************
***/


#include "usart.h"  

/* --- [RTOS 兼容改动] printf 串行化 ---
 * USART1 是"调试 printf + K230 视觉"共用口, 多个任务同时 printf 会让输出互相劈断,
 * 而 MicroLIB 的 printf 本身不可重入。这里用一个互斥量把 fputc 的发送过程锁起来。
 * 调度器启动前 / 中断上下文里不进锁, 只做裸发送(此时还没并发, 安全)。 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t s_dbgMutex = NULL;	/* printf 串行化用 */

// 锟斤拷锟斤拷锟斤拷usart IO锟节筹拷始锟斤拷
//
void  USART_GPIO_Config	(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd ( USART1_TX_CLK|USART1_RX_CLK, ENABLE); 	//IO锟斤拷时锟斤拷锟斤拷锟斤拷

	//IO锟斤拷锟斤拷
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;   	 //锟斤拷锟斤拷模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //锟斤拷锟斤拷
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;		 //锟斤拷锟斤拷
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //锟劫度等硷拷

	//锟斤拷始锟斤拷 TX	锟斤拷锟斤拷
	GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;	 
	GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);	
	//锟斤拷始锟斤拷 RX 锟斤拷锟斤拷													   
	GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;	
	GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);		
	
	//IO锟斤拷锟矫ｏ拷锟斤拷锟矫碉拷USART1
	GPIO_PinAFConfig(USART1_TX_PORT,USART1_TX_PinSource,GPIO_AF_USART1); 
	GPIO_PinAFConfig(USART1_RX_PORT,USART1_RX_PinSource,GPIO_AF_USART1);	
}

// 锟斤拷锟斤拷锟斤拷USART 锟节筹拷始锟斤拷
//
void Usart_Config(void)
{		
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// IO锟节筹拷始锟斤拷
	USART_GPIO_Config();
		 
	// 锟斤拷锟矫达拷锟节革拷锟斤拷锟斤拷锟?
	USART_InitStructure.USART_BaudRate 	 = USART1_BaudRate; //锟斤拷锟斤拷锟斤拷
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //锟斤拷锟斤拷位8位
	USART_InitStructure.USART_StopBits   = USART_StopBits_1; //停止位1位
	USART_InitStructure.USART_Parity     = USART_Parity_No ; //锟斤拷校锟斤拷
	USART_InitStructure.USART_Mode 	    = USART_Mode_Rx | USART_Mode_Tx; //锟斤拷锟酵和斤拷锟斤拷模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 锟斤拷使锟斤拷硬锟斤拷锟斤拷锟斤拷锟斤拷
	
	USART_Init(USART1,&USART_InitStructure); //锟斤拷始锟斤拷锟斤拷锟斤拷1
	USART_Cmd(USART1,ENABLE);	//使锟杰达拷锟斤拷1

	/* [RTOS 兼容改动] 创建 printf 互斥量(调度器启动前创建, heap_4 可直接用) */
	if (s_dbgMutex == NULL)
	{
		s_dbgMutex = xSemaphoreCreateMutex();
	}
}

// 锟斤拷锟斤拷锟斤拷锟截讹拷锟斤拷fputc锟斤拷锟斤拷

/*---------------------------------------------------------------
 *  [RTOS 兼容改动] fputc 串行化
 *  加锁条件: 互斥量已创建 + 调度器在跑 + 不在中断里。
 *  其余情况(启动阶段/中断里)直接发送, 避免在中断上下文里调用 RTOS API。
 *-------------------------------------------------------------*/
static uint8_t Usart_DebugLock(void)
{
	if (s_dbgMutex != NULL &&
		xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
		__get_IPSR() == 0)
	{
		xSemaphoreTake(s_dbgMutex, portMAX_DELAY);
		return 1;
	}
	return 0;
}

int fputc(int c, FILE *fp)
{
	uint8_t locked = Usart_DebugLock();
	uint32_t guard = 0;

	(void)fp;

	USART_SendData( USART1, (u8)c );					// 发送单字节数据
		/* 原来是无上限死等: 万一 USART1 时钟/引脚出问题, 这里会把整个任务永久卡住
		 * (而且是在持有 printf 互斥量的情况下)。加一个上限, 正常发送永远走不到。 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		{
			if (++guard > 200000u) break;
		}

	if (locked)
	{
		xSemaphoreGive(s_dbgMutex);
	}

	return (c);	// 返回字符
}



/***
	*************************************************************************************************
	*	@version V1.0
	*	@author  Â¹Ð¡ï¿½ï¿½Æ¼ï¿?
	*	@brief   usart ï¿½Ó¿ï¿½ï¿½ï¿½Øºï¿½ï¿½ï¿?
   *************************************************************************************************
   *  @description
	*
	*	Êµï¿½ï¿½Æ½Ì¨ï¿½ï¿½Â¹Ð¡ï¿½ï¿½STM32F407ZGT6ï¿½ï¿½ï¿½Ä°ï¿½ ï¿½ï¿½ï¿½ÍºÅ£ï¿½LXB407ZG-P1ï¿½ï¿½
	* ï¿½Í·ï¿½Î¢ï¿½Å£ï¿½19949278543
	*
>>>>> ï¿½Ä¼ï¿½Ëµï¿½ï¿½ï¿½ï¿½
	*
	*  1.ï¿½ï¿½Ê¼ï¿½ï¿½USART1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ PA9/PA10ï¿½ï¿½
	*  2.ï¿½ï¿½ï¿½ï¿½USART1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Õ·ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½Î»8Î»ï¿½ï¿½Í£Ö¹Î»1Î»ï¿½ï¿½ï¿½ï¿½Ð£ï¿½é¡¢ï¿½ï¿½Ê¹ï¿½ï¿½Ó²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ£ï¿½
	*    ï¿½ï¿½ï¿½ÚµÄ²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Îª115200ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Òªï¿½ï¿½ï¿½Ä²ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½ï¿½ï¿½Þ¸ï¿½usart.hï¿½ï¿½Äºê¶¨ï¿½ï¿½USART1_BaudRateï¿½ï¿½
	*  3.ï¿½Ø¶ï¿½ï¿½ï¿½fputcï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½Ö§ï¿½ï¿½Ê¹ï¿½ï¿½printfï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ó¡ï¿½ï¿½ï¿½ï¿½
	*
	************************************************************************************************
***/


#include "usart.h"  

/* --- [RTOS ¼æÈÝ¸Ä¶¯] printf ´®ÐÐ»¯ ---
 * USART1 ÊÇ"µ÷ÊÔ printf + K230 ÊÓ¾õ"¹²ÓÃ¿Ú, ¶à¸öÈÎÎñÍ¬Ê± printf »áÈÃÊä³ö»¥ÏàÅü¶Ï,
 * ¶ø MicroLIB µÄ printf ±¾Éí²»¿ÉÖØÈë¡£ÕâÀïÓÃÒ»¸ö»¥³âÁ¿°Ñ fputc µÄ·¢ËÍ¹ý³ÌËøÆðÀ´¡£
 * µ÷¶ÈÆ÷Æô¶¯Ç° / ÖÐ¶ÏÉÏÏÂÎÄÀï²»½øËø, Ö»×öÂã·¢ËÍ(´ËÊ±»¹Ã»²¢·¢, °²È«)¡£ */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t s_dbgMutex = NULL;	/* printf ´®ÐÐ»¯ÓÃ */

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½usart IOï¿½Ú³ï¿½Ê¼ï¿½ï¿½
//
void  USART_GPIO_Config	(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd ( USART1_TX_CLK|USART1_RX_CLK, ENABLE); 	//IOï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

	//IOï¿½ï¿½ï¿½ï¿½
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;   	 //ï¿½ï¿½ï¿½ï¿½Ä£Ê½
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   //ï¿½ï¿½ï¿½ï¿½
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;		 //ï¿½ï¿½ï¿½ï¿½
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //ï¿½Ù¶ÈµÈ¼ï¿½

	//ï¿½ï¿½Ê¼ï¿½ï¿½ TX	ï¿½ï¿½ï¿½ï¿½
	GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;	 
	GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);	
	//ï¿½ï¿½Ê¼ï¿½ï¿½ RX ï¿½ï¿½ï¿½ï¿½													   
	GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;	
	GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);		
	
	//IOï¿½ï¿½ï¿½Ã£ï¿½ï¿½ï¿½ï¿½Ãµï¿½USART1
	GPIO_PinAFConfig(USART1_TX_PORT,USART1_TX_PinSource,GPIO_AF_USART1); 
	GPIO_PinAFConfig(USART1_RX_PORT,USART1_RX_PinSource,GPIO_AF_USART1);	
}

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½USART ï¿½Ú³ï¿½Ê¼ï¿½ï¿½
//
void Usart_Config(void)
{		
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// IOï¿½Ú³ï¿½Ê¼ï¿½ï¿½
	USART_GPIO_Config();
		 
	// ï¿½ï¿½ï¿½Ã´ï¿½ï¿½Ú¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
	USART_InitStructure.USART_BaudRate 	 = USART1_BaudRate; //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //ï¿½ï¿½ï¿½ï¿½Î»8Î»
	USART_InitStructure.USART_StopBits   = USART_StopBits_1; //Í£Ö¹Î»1Î»
	USART_InitStructure.USART_Parity     = USART_Parity_No ; //ï¿½ï¿½Ð£ï¿½ï¿½
	USART_InitStructure.USART_Mode 	    = USART_Mode_Rx | USART_Mode_Tx; //ï¿½ï¿½ï¿½ÍºÍ½ï¿½ï¿½ï¿½Ä£Ê½
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ï¿½ï¿½Ê¹ï¿½ï¿½Ó²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	
	USART_Init(USART1,&USART_InitStructure); //ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1
	USART_Cmd(USART1,ENABLE);	//Ê¹ï¿½Ü´ï¿½ï¿½ï¿½1

	/* [RTOS ¼æÈÝ¸Ä¶¯] ´´½¨ printf »¥³âÁ¿(µ÷¶ÈÆ÷Æô¶¯Ç°´´½¨, heap_4 ¿ÉÖ±½ÓÓÃ) */
	if (s_dbgMutex == NULL)
	{
		s_dbgMutex = xSemaphoreCreateMutex();
	}
}

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø¶ï¿½ï¿½ï¿½fputcï¿½ï¿½ï¿½ï¿½

/*---------------------------------------------------------------
 *  [RTOS ¼æÈÝ¸Ä¶¯] fputc ´®ÐÐ»¯
 *  ¼ÓËøÌõ¼þ: »¥³âÁ¿ÒÑ´´½¨ + µ÷¶ÈÆ÷ÔÚÅÜ + ²»ÔÚÖÐ¶ÏÀï¡£
 *  ÆäÓàÇé¿ö(Æô¶¯½×¶Î/ÖÐ¶ÏÀï)Ö±½Ó·¢ËÍ, ±ÜÃâÔÚÖÐ¶ÏÉÏÏÂÎÄÀïµ÷ÓÃ RTOS API¡£
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

	USART_SendData( USART1, (u8)c );					// ·¢ËÍµ¥×Ö½ÚÊý¾Ý
		/* Ô­À´ÊÇÎÞÉÏÏÞËÀµÈ: ÍòÒ» USART1 Ê±ÖÓ/Òý½Å³öÎÊÌâ, ÕâÀï»á°ÑÕû¸öÈÎÎñÓÀ¾Ã¿¨×¡
		 * (¶øÇÒÊÇÔÚ³ÖÓÐ printf »¥³âÁ¿µÄÇé¿öÏÂ)¡£¼ÓÒ»¸öÉÏÏÞ, Õý³£·¢ËÍÓÀÔ¶×ß²»µ½¡£ */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		{
			if (++guard > 200000u) break;
		}

	if (locked)
	{
		xSemaphoreGive(s_dbgMutex);
	}

	return (c);	// ·µ»Ø×Ö·û
}



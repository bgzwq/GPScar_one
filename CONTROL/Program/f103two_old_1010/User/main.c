#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "AD.h"
#include "Key.h"
//#include "NRF24L01.h"					//NRF24L01无线已停用
#include "Serial.h"
#include "RemoteUI.h"

/**
  * 遥控器主程序
  * 功能：采集1个摇杆（2路ADC）+ 1个按键（PB12急停），组成4字节蓝牙遥控帧，
  *       通过蓝牙（HC-05/06透传）发出，
  *       OLED实时显示摇杆数值、发送的遥控帧、按键状态和链路状态
  * 
  * 蓝牙遥控协议（4字节帧）：
  *       [0]0xA5 [1]X [2]Y [3]KEY
  *       X/Y：0~255；KEY：0=未按下，1=PB12（急停），2=PB13，3=PB14，4=PB15
  * 
  * NRF24L01无线发送已停用，相关代码保留为注释
  */

/*主循环周期（ms），即遥控帧发送频率50Hz*/
#define LOOP_PERIOD_MS		20
/*界面刷新间隔（以主循环次数计），20ms*5=100ms*/
#define UI_REFRESH_DIV		5

/*NRF24L01发送缓冲区（已停用）
uint8_t TxPacket[NRF24L01_PAYLOAD];		//发送缓冲区
uint8_t aaa[NRF24L01_PAYLOAD]={0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
*/

/*组装NRF24L01遥控包（8字节，已停用）
static void BuildPacket(uint8_t *Joy, uint8_t KeyState)
{
	uint8_t i;
	uint8_t CheckSum = 0;
	
	TxPacket[0] = 0xA5;
	TxPacket[1] = Joy[0];
	TxPacket[2] = Joy[1];
	TxPacket[3] = Joy[2];
	TxPacket[4] = Joy[3];
	TxPacket[5] = KeyState & 0x0F;
	for (i = 0; i < 6; i ++)
	{
		CheckSum += TxPacket[i];
	}
	TxPacket[6] = CheckSum;
	TxPacket[7] = 0x5A;
}
*/

int main(void)
{
	uint8_t Joy[2];				//摇杆1两路：X、Y
	uint8_t KeyState;
	uint8_t TxKey;				//实际发送的按键数据字节
	//uint8_t NrfOnline;		//NRF24L01已停用
	//uint8_t AckOk;
	//uint8_t NrfStatus;
	uint8_t BtState;
	uint32_t TxCount = 0;
	uint8_t LoopCount = 0;

	
	/*外设初始化*/
	OLED_Init();
	AD_Init();
	Key_Init();
	Serial_Init();
	
	/*NRF24L01初始化，同时检测模块是否在线（已停用）
	NrfOnline = NRF24L01_Init();
	*/
	
	/*界面初始化*/
	UI_Init();
	
	while (1)
	{
		/*读取摇杆两路（映射到0~255，带中位死区）*/
		Joy[0] = AD_GetMapped(AD_J1X);
		Joy[1] = AD_GetMapped(AD_J1Y);
		/*摇杆2多余，已停用
		Joy[2] = 128;
		Joy[3] = 128;
		*/
		
		/*按键扫描（消抖）并读取状态*/
		Key_Scan();
		KeyState = Key_GetState();
		
		/*NRF24L01组装并发送遥控包（带ACK应答），已停用
		BuildPacket(Joy, KeyState);
		if (NrfOnline)
		{
			AckOk = NRF24L01_SendPacket(TxPacket, NRF24L01_PAYLOAD);
			if (AckOk)
			{
				TxCount ++;
				NrfStatus = UI_NRF_OK;
			}
			else
			{
				NrfStatus = UI_NRF_LOST;
			}
		}
		else
		{
			NrfStatus = UI_NRF_OFFLINE;
		}
		*/
		
		/*按键数据编码为按键编号：PB12→1，PB13→2，PB14→3，PB15→4，无按键→0（多键同按取小号）*/
		if (KeyState & KEY_1)		{TxKey = 1;}
		else if (KeyState & KEY_2)	{TxKey = 2;}
		else if (KeyState & KEY_3)	{TxKey = 3;}
		else if (KeyState & KEY_4)	{TxKey = 4;}
		else						{TxKey = 0;}
		
		/*蓝牙串口发送4字节遥控帧（帧头+X+Y+按键数据）*/
		Serial_SendRemoteFrame(Joy[0], Joy[1], TxKey);
		TxCount ++;
		
		/*读取蓝牙配对状态（STATE脚电平）*/
		BtState = Serial_BTConnected();
		
		/*定期刷新OLED界面（OLED全屏刷新较慢，不逐次刷新）*/
		LoopCount ++;
		if (LoopCount >= UI_REFRESH_DIV)
		{
			LoopCount = 0;
			UI_Update(Joy, KeyState, UI_NRF_OFFLINE, BtState, TxCount, TxKey);
		}
		
		Delay_ms(LOOP_PERIOD_MS);
	}
}

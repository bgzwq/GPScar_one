#include "stm32f10x.h"
#include "OLED.h"
#include "RemoteUI.h"

/**
  * 遥控器显示界面
  * 128x64，6x8字体为主
  * 
  *  y=0   NRF:OK  BT:ON  S:12345     状态栏：NRF链路 / 蓝牙 / 已发包数
  *  y=15  J1X 128 [####------]       摇杆 X轴
  *  y=28  J1Y 128 [####------]       摇杆 Y轴
  *  y=41  TX:A5 80 80 02             实际发送的4字节遥控帧（十六进制）
  *  y=53  K1:0 K2:0 K3:0 K4:0        按键状态
  */

/*条形图参数：起始X、宽度、高度*/
#define UI_BAR_X		46
#define UI_BAR_W		80
#define UI_BAR_H		8

/**
  * 函    数：画一个百分比条形图
  * 参    数：Y 条形图左上角纵坐标
  * 参    数：Value 数值，范围：0~255
  * 返 回 值：无
  */
static void UI_DrawBar(int16_t Y, uint8_t Value)
{
	uint8_t Fill = (uint16_t)Value * (UI_BAR_W - 2) / 255;
	
	OLED_ClearArea(UI_BAR_X, Y, UI_BAR_W, UI_BAR_H);
	OLED_DrawRectangle(UI_BAR_X, Y, UI_BAR_W, UI_BAR_H, OLED_UNFILLED);
	if (Fill > 0)
	{
		OLED_DrawRectangle(UI_BAR_X + 1, Y + 1, Fill, UI_BAR_H - 2, OLED_FILLED);
	}
}

/**
  * 函    数：界面初始化，画静态框架
  * 参    数：无
  * 返 回 值：无
  */
void UI_Init(void)
{
	OLED_Clear();
	
	OLED_ShowString(0, 15, "J1X", OLED_6X8);
	OLED_ShowString(0, 28, "J1Y", OLED_6X8);
	
	OLED_Update();
}

/**
  * 函    数：刷新界面动态内容
  * 参    数：Joy 摇杆映射值数组（0~255），只显示前2路：J1X J1Y
  * 参    数：KeyState 按键状态位图（bit0~bit3对应K1~K4）
  * 参    数：NrfStatus NRF链路状态，取值：UI_NRF_OFFLINE/LOST/OK
  * 参    数：BtState 蓝牙配对状态，1=已连接，0=未连接
  * 参    数：TxCount 已成功发送的包计数
  * 参    数：TxKey 实际发送的按键数据字节
  * 返 回 值：无
  */
void UI_Update(uint8_t *Joy, uint8_t KeyState, uint8_t NrfStatus, uint8_t BtState, uint32_t TxCount, uint8_t TxKey)
{
	/*状态栏*/
	OLED_ClearArea(0, 0, 128, 8);
	if (NrfStatus == UI_NRF_OK)
	{
		OLED_ShowString(0, 0, "NRF:OK  ", OLED_6X8);
	}
	else if (NrfStatus == UI_NRF_LOST)
	{
		OLED_ShowString(0, 0, "NRF:LOST", OLED_6X8);
	}
	else
	{
		OLED_ShowString(0, 0, "NRF:--  ", OLED_6X8);
	}
	if (BtState)
	{
		OLED_ShowString(54, 0, "BT:OK", OLED_6X8);
	}
	else
	{
		OLED_ShowString(54, 0, "BT:--", OLED_6X8);
	}
	OLED_Printf(90, 0, OLED_6X8, "S:%-5u", TxCount);
	
	/*摇杆两轴数值与条形图*/
	OLED_ShowNum(24, 15, Joy[0], 3, OLED_6X8);
	UI_DrawBar(15, Joy[0]);
	
	OLED_ShowNum(24, 28, Joy[1], 3, OLED_6X8);
	UI_DrawBar(28, Joy[1]);
	
	/*实际发送的4字节遥控帧：帧头 A5 + X + Y + KEY*/
	OLED_Printf(0, 41, OLED_6X8, "TX:%02X %02X %02X %02X",
				0xA5, Joy[0], Joy[1], TxKey);
	
	/*按键状态*/
	OLED_Printf(0, 53, OLED_6X8, "K1:%d K2:%d K3:%d K4:%d",
				(KeyState & 0x01) ? 1 : 0,
				(KeyState & 0x02) ? 1 : 0,
				(KeyState & 0x04) ? 1 : 0,
				(KeyState & 0x08) ? 1 : 0);
	
	/*将显存更新到屏幕*/
	OLED_Update();
}

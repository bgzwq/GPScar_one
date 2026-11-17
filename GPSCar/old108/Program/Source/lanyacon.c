/***
	*************************************************************************************************
	*	@file  	lanyacon.c
	*	@brief  蓝牙摇杆遥控 (4字节帧: 0xA5 + X + Y + 按键)
   *************************************************************************************************
   *  @description
	*
	*  如果蓝牙模块连接上，则启动蓝牙控制模式
	*  蓝牙接收到的指令，控制小车的运动
	*  数据格式如下：
	*  总共4字节，采集1个摇杆（2路ADC）+ 1个按键，
	*  帧头：0xA5
	*  x轴（0~255）: 0x00表示左极限，0xFF表示右极限
	*  y轴（0~255）: 0x00表示上极限，0xFF表示下极限
	*  按键：0x00表示未按下，0x01表示按下；按下即急停,
	*  消抖确认的按下沿翻转全部车身LED作急停指示(再按一次熄灭)
	*
	*  实现说明:
	*    1. LanyaCon_Feed() 由 main.c 的蓝牙字节处理(App_BTProcess)逐字节喂入,
	*       属于遥控帧的字节被消费(返回1), 其余字节继续走单字符指令
	*    2. 摇杆转差速: 左轮 = 前进量+转向量/2, 右轮 = 前进量-转向量/2
	*    3. 运动控制仅在手动模式(MODE_MANUAL)下生效, 急停任何模式都生效
	*    4. 按键存在物理抖动, 原始状态需稳定 LANYA_BTN_DEBOUNCE_MS 才确认;
	*       急停停车本身立即执行不等消抖(保证安全), 消抖只用于LED翻转
	*    5. 500ms收不到新帧自动停车(掉线保护)
	*
	*************************************************************************************************
***/

#include "lanyacon.h"
#include "car.h"
#include "car_led.h"
#include "menu.h"
#include "timer.h"

/*---------------------- 静态状态 ----------------------------*/
static LanyaCon_DataTypeDef s_ly;			// 最新遥控数据
static uint8_t  s_rxBuf[3];					// 帧头之后的 X/Y/按键
static uint8_t  s_rxCnt  = 0;				// 已收到的帧内字节数(0=等待帧头)
static uint32_t s_lastFrameMs = 0;			// 最后一个完整帧的时间戳
static uint8_t  s_wasControlling = 0;		// 上一拍是否真的在用摇杆驱动底盘
static uint8_t  s_btnCandidate = 0;			// 按键消抖: 待确认的原始状态
static uint8_t  s_btnDebounced = 0;			// 按键消抖: 已确认的状态
static uint32_t s_btnChangeMs  = 0;			// 按键消抖: 原始状态最近一次变化的时间戳

/*************************************************************************************************
*	函 数 名:	LanyaCon_DebounceButton
*
*	函数功能:	急停按键消抖。物理抖动会让按键字节在0/1间跳变, 原始状态稳定
*				LANYA_BTN_DEBOUNCE_MS 后才确认; 确认按下的上升沿翻转全部车身LED。
*				停车动作本身不做消抖(见 LanyaCon_Apply), 保证急停立即生效。
*
*************************************************************************************************/
static void LanyaCon_DebounceButton(void)
{
	uint32_t now = Get_Ms();

	if (s_ly.button != s_btnCandidate)
	{
		s_btnCandidate = s_ly.button;	// 原始状态变化, 重新计时
		s_btnChangeMs  = now;
	}
	else if (s_ly.button != s_btnDebounced &&
			 (uint32_t)(now - s_btnChangeMs) >= LANYA_BTN_DEBOUNCE_MS)
	{
		s_btnDebounced = s_ly.button;	// 稳定超时, 确认新状态
		if (s_btnDebounced)				// 确认按下的上升沿: 翻转全部车身LED
		{
			CarLED_Toggle(CAR_LED1);
			CarLED_Toggle(CAR_LED2);
			CarLED_Toggle(CAR_LED3);
			CarLED_Toggle(CAR_LED4);
		}
	}
}

/*************************************************************************************************
*	函 数 名:	LanyaCon_Apply
*
*	函数功能:	把一帧遥控数据转换为小车运动(差速), 急停优先
*
*************************************************************************************************/
static void LanyaCon_Apply(void)
{
	int16_t fwd, turn, left, right;

	LanyaCon_DebounceButton();

	if (s_ly.button)	// 按键=急停, 任何模式都生效(立即停车, 不等消抖)
	{
		Car_Stop();
		s_wasControlling = 0;
		return;
	}

	if (g_mode != MODE_MANUAL)
	{
		s_wasControlling = 0;
		return;			// 只有手动模式响应摇杆
	}

	/* 摇杆偏移: 上推=前进为正, 右推=右转为正 */
	fwd  = (int16_t)LANYA_CENTER - (int16_t)s_ly.y;
	turn = (int16_t)s_ly.x - (int16_t)LANYA_CENTER;

	/* 死区: 摇杆松开时自动回中附近, 视为停车 */
	if (fwd  > -LANYA_DEADZONE && fwd  < LANYA_DEADZONE) fwd  = 0;
	if (turn > -LANYA_DEADZONE && turn < LANYA_DEADZONE) turn = 0;

	/* 差速合成 + 映射到目标速度 */
	left  = (int16_t)((fwd + turn / 2) * LANYA_SPEED_MAX / LANYA_CENTER);
	right = (int16_t)((fwd - turn / 2) * LANYA_SPEED_MAX / LANYA_CENTER);

	if (left == 0 && right == 0)
	{
		Car_Stop();
		s_wasControlling = 0;
		return;
	}

	/* M1 M2 左侧, M3 M4 右侧 */
	/* 只投递"左右两侧速度", 真正写电机在 ControlTask(4个轮子一次写完, 不会半截) */
	Car_PostLR(CAR_SRC_MANUAL, left, right);
	s_wasControlling = 1;
}

/*************************************************************************************************
*	函 数 名:	LanyaCon_Feed
*
*	入口参数:	byte - 蓝牙串口收到的1字节
*
*	返 回 值:	1 - 该字节属于遥控帧(已消费), 0 - 与遥控帧无关
*
*	函数功能:	4字节帧组帧状态机, 收到完整帧立即执行运动控制
*
*************************************************************************************************/
uint8_t LanyaCon_Feed(uint8_t byte)
{
	if (s_rxCnt == 0)
	{
		if (byte != LANYA_FRAME_HEAD) return 0;	// 非帧头: 不是遥控帧
		s_rxCnt = 1;
		return 1;
	}

	s_rxBuf[s_rxCnt - 1] = byte;
	if (++s_rxCnt < 4) return 1;				// 帧未收满
	s_rxCnt = 0;								// 收满4字节, 复位等下一帧

	s_ly.x      = s_rxBuf[0];
	s_ly.y      = s_rxBuf[1];
	s_ly.button = (s_rxBuf[2] != 0) ? 1 : 0;

	s_ly.online = 1;
	s_lastFrameMs = Get_Ms();

	LanyaCon_Apply();
	return 1;
}

/*************************************************************************************************
*	函 数 名:	LanyaCon_GetData
*
*	函数功能:	获取最新遥控数据指针(可用于菜单显示)
*
*************************************************************************************************/
LanyaCon_DataTypeDef *LanyaCon_GetData(void)
{
	return &s_ly;
}

/*************************************************************************************************
*	函 数 名:	LanyaCon_Task
*
*	函数功能:	掉线保护: 超过 LANYA_TIMEOUT_MS 没有新帧则停车
*
*	入口参数:	now - 系统毫秒戳(Get_Ms())
*
*	说    明:	只在"之前确实是用摇杆在驱动底盘"时才刹车。
*				原来的实现只要收到过一帧, 之后无论切到避障还是K230视觉模式,
*				500ms后都会无条件 Car_Stop(), 和其它模式抢底盘控制权。
*
*************************************************************************************************/
void LanyaCon_Task(uint32_t now)
{
	if (s_ly.online && (uint32_t)(now - s_lastFrameMs) > LANYA_TIMEOUT_MS)
	{
		s_ly.online = 0;
		if (s_wasControlling)
		{
			s_wasControlling = 0;
			Car_Stop();
		}
	}
}

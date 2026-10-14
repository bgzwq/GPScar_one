/***
	*************************************************************************************************
	*	@file  	K230.c
	*	@brief  K230视觉模块驱动 (USART1, PA9/PA10)
   *************************************************************************************************
   *  @description
	*
	*  K230 -> 主控 帧格式(8字节, 无校验, 见K230.h结构体注释):
	*    0x55 | LR | X高字节 | X低字节 | UD | Y高字节 | Y低字节 | flag
	*    LR=1目标在左侧 0=右侧   UD=1上侧 0=下侧
	*    X/Y=目标与屏幕中心的距离(像素)   flag=1要夹取 0=继续前行
	*
	*  主控 -> K230 帧格式(变长, 带校验):
	*    0xAA | CMD | LEN | payload[LEN] | CHK
	*    CHK = 从0xAA到payload最后一字节逐字节异或
	*
	*  实现说明:
	*    1. USART1接收中断把字节存入环形缓冲区 K230_arr
	*    2. 主循环调用 K230_Process() 组帧 -> K230_sovle() 解析为控制量
	*       -> VISION模式下驱动小车
	*    3. 500ms收不到新帧自动停车(掉线保护)
	*    4. flag=1(要夹取)时先停车, 回 ACK 给K230, 等机械臂动作完成
	*       (没有机械臂时由 K230_GrabTask() 超时自动收尾, 流程不会卡死)
	*    5. 1s向K230发一次心跳, K230可据此判断主控是否在线
	*
	*  【与其它模块的分工】
	*    原来K230是在 MODE_MANUAL 下直接驱动电机的, 而蓝牙摇杆(lanyacon.c)和
	*    语音(asrpro)在手动模式下也会写电机速度 —— 两套控制互相覆盖轮速。
	*    现在K230只在独立的 MODE_VISION 下驱动底盘(见menu.h/WorkMode_TypeDef),
	*    手动模式只受摇杆/语音控制, 从根上消除抢控制权的问题。
	*
	*************************************************************************************************
***/

#include "K230.h"
#include "usart.h"
#include "car.h"
#include "menu.h"
#include "timer.h"
#include "buzzer.h"

/*---------------------- 协议与控制参数 ------------------------*/
#define K230_FRAME_HEAD		0x55	// 帧头
#define K230_TIMEOUT_MS		500		// 掉线保护: 无新帧自动停车
#define K230_HEARTBEAT_MS	1000	// 心跳周期
#define K230_GRAB_TIMEOUT_MS 3000	// 夹取挂起超时(没有机械臂时用来收尾)

#define K230_BASE_SPEED		40		// 基础速度(0~100)
#define K230_CAR_SPEED		60		// 100%对应的闭环目标速度(脉冲/20ms)
#define K230_X_ALIGNED		20		// X小于该值认为已对准, 不纠偏
#define K230_Y_NEAR			50		// Y小于该值认为已接近目标, 减速

/*---------------------- 缓冲区 ----------------------------*/
static uint8_t K230_arr[100];	// K230发过来的信息缓冲区(接收中断写, K230_Process读)

/*
 * K230测试数据: 按 0x55|LR|X高|X低|UD|Y高|Y低|flag 帧格式,
 * 模拟视觉追踪一个目标的典型过程: 左侧远处 -> 逐渐对准 -> 接近 -> 夹取
 */
uint8_t K230_test[400] =
{
	/* 帧1: 目标在左侧, 横向偏差288, 纵向200 */
	0x55, 1, 0x01, 0x20, 1, 0x00, 0xC8, 0,
	/* 帧2: 目标仍偏左, 横向80, 纵向150 */
	0x55, 1, 0x00, 0x50, 1, 0x00, 0x96, 0,
	/* 帧3: 基本对准, 横向10, 纵向80 */
	0x55, 1, 0x00, 0x0A, 1, 0x00, 0x50, 0,
	/* 帧4: 目标偏到右侧, 横向150, 纵向120 */
	0x55, 0, 0x00, 0x96, 1, 0x00, 0x78, 0,
	/* 帧5: 已对准且很近(下侧), 纵向40 */
	0x55, 0, 0x00, 0x14, 0, 0x00, 0x28, 0,
	/* 帧6: 到达夹取点, flag=1请求夹取 */
	0x55, 0, 0x00, 0x05, 0, 0x00, 0x0A, 1,
	/* 帧7: 错误帧头(非0x55), 应被解析丢弃 */
	0xAA, 1, 0x00, 0x64, 1, 0x00, 0x64, 0,
	/* 其余字节保持0(无数据) */
};

/*---------------------- 静态状态 ----------------------------*/
static volatile uint8_t s_rxWr = 0;		// 环形缓冲写指针(中断)
static volatile uint8_t s_rxRd = 0;		// 环形缓冲读指针(主循环)
static K230_con s_con;					// 最新解析出的控制量
static uint32_t s_lastFrameMs = 0;		// 最后一个有效帧时间戳
static uint8_t  s_wasControlling = 0;	// 上一拍是否真的在驱动底盘
static uint32_t s_grabMs = 0;			// 夹取挂起开始时刻
static uint32_t s_lastHeartMs = 0;		// 上次心跳时刻
static K230_Status_TypeDef s_st;		// 链路统计

/*************************************************************************************************
*	函 数 名:	USART1_IRQHandler
*
*	函数功能:	USART1接收中断, 把K230发来的字节存入环形缓冲区
*
*************************************************************************************************/
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t b    = (uint8_t)USART_ReceiveData(USART1);	// 读DR自动清RXNE
		uint8_t next = (uint8_t)((s_rxWr + 1) % sizeof(K230_arr));

		if (next != s_rxRd)			// 缓冲满则丢弃
		{
			K230_arr[s_rxWr] = b;
			s_rxWr = next;
		}
		else
		{
			s_st.overflow++;		// 记录因为没及时处理而丢掉的字节
		}
	}
}

/*************************************************************************************************
*	函 数 名:	K230_Init
*
*	函数功能:	初始化串口1(PA9/PA10)并开启接收中断, 作为主控板和K230的桥梁
*
*************************************************************************************************/
void K230_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	Usart_Config();	// USART1 初始化(main中已调用, 重复调用无害)

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	s_rxWr = 0;
	s_rxRd = 0;
	s_con.flag = 0;
	s_lastFrameMs    = 0;
	s_wasControlling = 0;
	s_st.grabPending = 0;
	s_st.online      = 0;
}

/*************************************************************************************************
*	函 数 名:	K230_SendCmd
*
*	入口参数:	cmd     - K230_CMD_* 指令
*				payload - 负载数据, 可为0
*				len     - 负载长度(<=K230_TX_MAX_PAYLOAD)
*
*	函数功能:	按 0xAA|CMD|LEN|payload|CHK 组帧并发送给K230
*
*	说    明:	发送是阻塞的, 但整帧最长12字节, 115200下约1ms, 可接受
*
*************************************************************************************************/
void K230_SendCmd(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	uint8_t i;
	uint8_t chk;
	uint8_t buf[4 + K230_TX_MAX_PAYLOAD];

	if (len > K230_TX_MAX_PAYLOAD) len = K230_TX_MAX_PAYLOAD;

	buf[0] = K230_TX_HEAD;
	buf[1] = cmd;
	buf[2] = len;
	chk    = (uint8_t)(buf[0] ^ buf[1] ^ buf[2]);

	for (i = 0; i < len; i++)
	{
		buf[3 + i] = payload[i];
		chk ^= payload[i];
	}
	buf[3 + len] = chk;

	for (i = 0; i < (uint8_t)(4 + len); i++)
	{
		USART_SendData(USART1, buf[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}

	s_st.txFrames++;
}

/*************************************************************************************************
*	函 数 名:	K230_sovle
*
*	入口参数:	k230_t - 一帧原始数据(K230_time)
*				k230_c - 输出解析后的控制量(K230_con)
*
*	函数功能:	解析K230视觉数据, 转换成4轮速度与转向:
*				flag=1        -> 要夹取: 停车(机械臂动作预留)
*				flag=0 且未对准 -> 按X距离比例差速纠偏(LR决定方向)
*				flag=0 且已对准 -> 直行, 接近目标(Y小)时减速
*
*************************************************************************************************/
void K230_sovle(struct K230_time *k230_t, struct K230_con *k230_c)
{
	uint8_t base, slow;

	if (k230_t->head != K230_FRAME_HEAD)	// 帧头不对, 数据无效
	{
		k230_c->flag = 0;
		return;
	}
	k230_c->flag = 1;	// 数据正常

	if (k230_t->flag == 1)	// 要夹取: 停车等待夹取
	{
		k230_c->speed1 = k230_c->speed2 = 0;
		k230_c->speed3 = k230_c->speed4 = 0;
		k230_c->turn = 90;
		return;
	}

	/* 继续前行: 接近目标(下方/纵向距离小)时减速 */
	base = K230_BASE_SPEED;
	if (k230_t->UD == 0 || k230_t->y < K230_Y_NEAR) base = K230_BASE_SPEED / 2;

	/* 按横向偏差纠偏: 偏差越大内侧轮减速越多(最多减一半) */
	slow = 0;
	if (k230_t->x > K230_X_ALIGNED)
	{
		uint16_t s = (uint16_t)k230_t->x * (K230_BASE_SPEED / 2) / 200;
		slow = (s > K230_BASE_SPEED / 2) ? (K230_BASE_SPEED / 2) : (uint8_t)s;
	}

	if (k230_t->x <= K230_X_ALIGNED)
	{
		k230_c->turn = 90;	// 已对准, 直行
		k230_c->speed1 = k230_c->speed2 = base;
		k230_c->speed3 = k230_c->speed4 = base;
	}
	else if (k230_t->LR == 1)
	{
		k230_c->turn = 45;	// 目标在左: 左轮减速
		k230_c->speed1 = k230_c->speed2 = base - slow;	// M1 M2 左侧
		k230_c->speed3 = k230_c->speed4 = base;			// M3 M4 右侧
	}
	else
	{
		k230_c->turn = 135;	// 目标在右: 右轮减速
		k230_c->speed1 = k230_c->speed2 = base;
		k230_c->speed3 = k230_c->speed4 = base - slow;
	}
}

/*************************************************************************************************
*	函 数 名:	K230_ControlCar
*
*	函数功能:	用解析出的控制量调控小车运动(速度0~100映射到闭环目标速度)
*
*	说    明:	只在 MODE_VISION 下驱动底盘; 其它模式(尤其MANUAL下的摇杆遥控)
*				不受K230影响, 避免两套控制源互相覆盖轮速。
*
*************************************************************************************************/
static void K230_ControlCar(const K230_con *k230_c)
{
	if (k230_c->flag != 1) return;

	if (g_mode != MODE_VISION)
	{
		s_wasControlling = 0;
		return;					// 非视觉模式不接管底盘
	}

	Car_SetWheelSpeed(MOTOR_1, (int16_t)k230_c->speed1 * K230_CAR_SPEED / 100);
	Car_SetWheelSpeed(MOTOR_2, (int16_t)k230_c->speed2 * K230_CAR_SPEED / 100);
	Car_SetWheelSpeed(MOTOR_3, (int16_t)k230_c->speed3 * K230_CAR_SPEED / 100);
	Car_SetWheelSpeed(MOTOR_4, (int16_t)k230_c->speed4 * K230_CAR_SPEED / 100);

	s_wasControlling = 1;
}

/*************************************************************************************************
*	函 数 名:	K230_StartTrack / K230_StopTrack
*
*	函数功能:	进入/离开视觉模式时通知K230开始或停止输出目标
*
*************************************************************************************************/
void K230_StartTrack(void)
{
	K230_SendCmd(K230_CMD_TRACK_START, 0, 0);
}

void K230_StopTrack(void)
{
	K230_SendCmd(K230_CMD_TRACK_STOP, 0, 0);
}

/*************************************************************************************************
*	函 数 名:	K230_GrabDone
*
*	函数功能:	机械臂动作完成后调用: 清除挂起状态并通知K230继续追踪目标
*
*	说    明:	目前机械臂尚未接入, 由 K230_GrabTask() 超时自动调用本函数
*
*************************************************************************************************/
void K230_GrabDone(void)
{
	if (!s_st.grabPending) return;

	s_st.grabPending = 0;
	K230_SendCmd(K230_CMD_GRAB_DONE, 0, 0);
	Buzzer_BeepAsync(2600, 60);
}

/*************************************************************************************************
*	函 数 名:	K230_GrabTask
*
*	函数功能:	夹取挂起超时收尾, 主循环调用
*
*	说    明:	K230请求夹取后小车停车并等待机械臂。若机械臂没有接入(或动作失败),
*				这里超时后自动结束挂起, 否则K230会一直等DONE, 流程卡死。
*				接入机械臂后, 在动作完成处调用 K230_GrabDone() 即可提前结束。
*
*************************************************************************************************/
void K230_GrabTask(void)
{
	if (!s_st.grabPending) return;

	if ((uint32_t)(Get_Ms() - s_grabMs) >= K230_GRAB_TIMEOUT_MS)
	{
		K230_GrabDone();
	}
}

/*************************************************************************************************
*	函 数 名:	K230_Process
*
*	函数功能:	主循环调用: 从接收缓冲区组帧 -> 解析 -> 控制小车; 含掉线保护/心跳/夹取处理
*
*************************************************************************************************/
void K230_Process(void)
{
	static uint8_t    state = 0;	// 0等帧头 1等LR 2收X 3等UD 4收Y 5等flag
	static K230_time  frame;
	static uint8_t    tmp;
	uint8_t b;
	uint32_t now = Get_Ms();

	while (s_rxRd != s_rxWr)
	{
		b = K230_arr[s_rxRd];
		s_rxRd = (uint8_t)((s_rxRd + 1) % sizeof(K230_arr));

		switch (state)
		{
			case 0:	// 等帧头
				if (b == K230_FRAME_HEAD) { frame.head = b; state = 1; }
				break;
			case 1:	frame.LR = b; state = 2; break;
			case 2:	tmp = b;      state = 3; break;	// X高字节
			case 3:	frame.x = ((uint16_t)tmp << 8) | b; state = 4; break;
			case 4:	frame.UD = b; state = 5; break;
			case 5:	tmp = b;      state = 6; break;	// Y高字节
			case 6:	frame.y = ((uint16_t)tmp << 8) | b; state = 7; break;
			case 7:	// flag, 一帧收齐
				frame.flag = b;
				state = 0;

				/* 协议约定这三位只能是0/1, 超出说明是错帧(仅统计, 不改变原有处理) */
				if (frame.LR > 1 || frame.UD > 1 || frame.flag > 1) s_st.rxBadFrames++;

				K230_sovle(&frame, &s_con);
				if (s_con.flag == 1)
				{
					s_st.rxFrames++;
					s_st.raw         = frame;
					s_st.con         = s_con;
					s_lastFrameMs    = now;
					s_st.lastFrameMs = now;
					s_st.online      = 1;

					if (frame.flag == 1)	// 请求夹取
					{
						Car_Stop();			// 先停车, 不管当前什么模式
						s_wasControlling = 0;

						if (!s_st.grabPending)	// 只在第一次请求时回ACK/蜂鸣
						{
							s_st.grabPending = 1;
							s_grabMs = now;
							K230_SendCmd(K230_CMD_GRAB_ACK, 0, 0);
							Buzzer_BeepAsync(3200, 120);
						}
					}
					else
					{
						K230_ControlCar(&s_con);
					}
				}
				break;
			default: state = 0; break;
		}
	}

	/* 掉线保护: 500ms无有效帧, 且之前确实在控制底盘, 才停车。
	 * 【重要】原来这里无条件 Car_Stop(), 即使当前是避障/雷达/手动模式,
	 * 只要K230曾经发过一帧, 500ms后就会去踩刹车, 和其它模式抢底盘。 */
	if (s_con.flag == 1 && (uint32_t)(now - s_lastFrameMs) > K230_TIMEOUT_MS)
	{
		s_con.flag  = 0;
		s_st.online = 0;
		if (s_wasControlling)
		{
			s_wasControlling = 0;
			Car_Stop();
		}
	}

	/* 心跳: 让K230知道主控还在 */
	if ((uint32_t)(now - s_lastHeartMs) >= K230_HEARTBEAT_MS)
	{
		s_lastHeartMs = now;
		K230_SendCmd(K230_CMD_HEARTBEAT, 0, 0);
	}
}

/*************************************************************************************************
*	函 数 名:	K230_IsOnline / K230_GetStateName / K230_GetStatus
*
*	函数功能:	链路状态查询(供菜单显示)
*
*************************************************************************************************/
uint8_t K230_IsOnline(void)
{
	if (s_st.lastFrameMs == 0) return 0;
	return ((uint32_t)(Get_Ms() - s_st.lastFrameMs) <= K230_TIMEOUT_MS) ? 1 : 0;
}

const char *K230_GetStateName(void)
{
	if (K230_IsOnline())  return "ON  ";
	if (s_st.lastFrameMs) return "LOST";
	return "WAIT";
}

K230_Status_TypeDef *K230_GetStatus(void)
{
	return &s_st;
}

/***
	***************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*	@brief   STM32F407ZGT6 智能小车
   ****************************************************************************************************
   *  @description
	*
	*	实验平台：鹿小班STM32F407ZGT6核心板 （型号：LXB407ZG-P1）
	*
>>>>> 功能说明：
	*
	*	1. 四驱底盘: TIM1四路PWM + 4路编码器, 开环直驱(速度PID闭环已注释)
	*	2. 传感器: MPU6050(姿态), 2路电源ADC, 亮度ADC, 湿度ADC
	*	3. 避障: 车头固定超声波 + 舵机扫描超声波雷达; 车头距离2~5cm触发蜂鸣过近警报(带毛刺过滤)
	*	4. 通信: USART1调试+K230视觉, USART2蓝牙/WiFi遥控, USART3 ASRPRO语音, USART6 GPS
	*	5. 人机: LCD显示, 4个LED, 4个按键, 无源蜂鸣器 (按键菜单/LCD界面见 menu.c)
	*	6. 定位: GPS + MPU6050惯导融合(location.c), 上电静止调零
	*	7. 记录: 轨迹地图显示 + 存SD卡(navlog.c/mapview.c)
	*
>>>>> 按键功能(分页菜单见 menu.c)：
	*
	*	KEY1(PE2): 菜单上一页            KEY2(PE3): 菜单下一页
	*	KEY3(PG5): 切换工作模式          KEY4(PG7): 电机自检(地图页=保存地图)
	*	长按(仅地图页): KEY1/KEY2 缩小/放大视野, KEY4 恢复自动缩放
	*	(原KEY5=PG8与电机M3方向脚冲突, 已移除)
	*
>>>>> 工作模式(KEY3循环)：
	*
	*	MANUAL 手动: 蓝牙摇杆/单字符/语音遥控
	*	AVOID  避障: 超声波触发后停车扫描并转向
	*	RADAR  雷达: 舵机0~180往复扫描
	*	VISION 视觉: K230接管底盘做目标追踪(与手动互不干扰)
	*
>>>>> 蓝牙/WiFi指令 (USART2)：
	*
	*	单字符: 'F'前进 'B'后退 'L'左转 'R'右转 'S'停止 '+'加速 '-'减速 'M'切换模式
	*	        (行进中收到'L'/'R'为差速转弯, 边行进边转向; 静止时才是原地转向)
	*	摇杆帧: 0xA5+X+Y+按键 (见 lanyacon.c, 按键=急停并翻转全部车身LED, 500ms掉线自动停车)
	*
>>>>> K230视觉 (USART1, 双向协议见 K230.h)：
	*
	*	K230->主控: 0x55+LR+X+UD+Y+flag (8字节, flag=1请求夹取)
	*	主控->K230: 0xAA+CMD+LEN+payload+CHK (心跳/开始停止追踪/夹取应答)
	*
>>>>> ASRPRO语音指令 (USART3, 按行匹配)：
	*
	*	forward / back / left / right / stop (在ASRPRO上位机中把识别结果
	*	配置为发送对应字符串即可, 可自行修改 App_ASRProcess 中的匹配表)
	*
>>>>> SD卡文件 (FatFs)：
	*
	*	0:/CARLOG/TRACK.CSV  轨迹点, 每次上电续写(经纬度/速度/航向/卫星/HDOP)
	*	0:/CARLOG/MAP.BMP    地图页按KEY4保存的 240x240 24位轨迹图
	*
	***************************************************************************************************
***/


#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include "delay.h"
#include "timer.h"
#include "led.h"
#include "usart.h"
#include "car.h"
#include "mpu6050.h"
#include "bluetooth.h"
#include "asrpro.h"
#include "sensor_adc.h"
#include "servo.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "car_led.h"
#include "car_key.h"
#include "lcd_spi_154.h"
#include "gps.h"
#include "location.h"
#include "K230.h"
#include "lanyacon.h"
#include "menu.h"
#include "navlog.h"
#include "mapview.h"
#include "bsp.h"
/*---------------------- FreeRTOS ----------------------------*/
#include "FreeRTOS.h"
#include "task.h"


/*---------------------- 运行参数 ----------------------------*/
#define DEFAULT_SPEED		30		// 默认目标速度(编码器脉冲/20ms)
#define SPEED_STEP			5		// 速度调节步进
#define SPEED_MAX			60
#define SPEED_MIN			10

#define AVOID_DIST_CM		25.0f	// 避障触发距离
#define AVOID_TURN_MS		450		// 避障转向时间

/* 车头超声波过近警报(2~5cm触发蜂鸣器) */
#define ALARM_NEAR_MIN_CM	2.0f	// 警报区间下限
#define ALARM_NEAR_MAX_CM	5.0f	// 警报区间上限
#define ALARM_ON_COUNT		3		// 连续N次落在区间内才触发(滤毛刺)
#define ALARM_OFF_COUNT		3		// 连续N次离开区间才解除(回差防抖)
#define ALARM_FREQ_HZ		3500	// 警报音调

/*---------------------- RTOS 任务参数 ----------------------------*/
/* 优先级: 数值越大越高。Idle=0, 软件定时器任务=31(configTIMER_TASK_PRIORITY)。
   心跳给 3 > 应用 2, 保证超声波长阻塞时 LED 节拍不受影响。 */
#define APP_TASK_PRIO		2
#define APP_TASK_STACK		1024	// 单位 word(4B) => 4KB, 要跑 FatFs/LCD/printf
									// 实测最深调用链 1780B, 留 2 倍余量
#define LED_TASK_PRIO		3
#define LED_TASK_STACK		128		// 512B, 只翻转一个 IO

/*---------------------- 共享运行状态(menu.c 人机交互共用, 声明见 menu.h) ---*/
WorkMode_TypeDef g_mode       = MODE_MANUAL;
int16_t  g_speed              = DEFAULT_SPEED;
	uint8_t  g_mpuOK              = 0;
float    g_distFixed          = 0;	// 固定超声波距离
float    g_distRadar          = 0;	// 雷达超声波距离
float    g_servoAngle         = 90;	// 当前舵机角度
MPU6050_DataTypeDef g_mpu;

/*---------------------- 自动避障状态机 ------------------------*/
typedef enum
{
	AVOID_FORWARD = 0,	// 前进巡航
	AVOID_SCAN,			// 停车扫描
	AVOID_TURN,			// 转向避障
} AvoidState_TypeDef;

static AvoidState_TypeDef g_avoidState = AVOID_FORWARD;
static uint32_t g_avoidTimer   = 0;
static float    g_distLeft     = 0;
static float    g_distRight    = 0;


/* 函数声明 */
static void App_Init(void);
static void App_BTProcess(void);
static void App_ASRProcess(void);
static void App_SensorTask(uint32_t now);
static void App_UltraAlarm(float dist);
static void App_AvoidTask(uint32_t now);
static void App_RadarTask(uint32_t now);

/* RTOS 任务 */
static void AppTask(void *pvParameters);
static void LedTask(void *pvParameters);


/***************************************************************************************************
*	函 数 名: main
*	函数功能: 初始化全部外设, 创建任务并启动 FreeRTOS 调度器
*
*	说    明: 【RTOS 兼容改动】
*			1. 先把 NVIC 优先级分组锁死为 Group_4(4 位全部作为抢占优先级)。
*			   FreeRTOS 的 BASEPRI 临界区机制要求所有实现的优先级位都是抢占位;
*			   若改成常见的 NVIC_PriorityGroup_2, 临界区会错位, 表现为随机死机。
*			2. App_Init() 仍在调度器启动前完成 —— BSP_Init() 里的 Delay_ms()
*			   会自动走"忙等"分支(见 delay.c), 不需要改任何驱动。
*			3. 业务逻辑整体搬进 AppTask, 框架与裸机版本一一对应。
****************************************************************************************************/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		/* FreeRTOS 硬性要求 */

	App_Init();		// 全部外设初始化（必须在调度器启动之前）

	/* 应用主任务: 原来 while(1) 里的全部业务 */
	if (xTaskCreate(AppTask, "APP", APP_TASK_STACK, NULL, APP_TASK_PRIO, NULL) != pdPASS)
	{
		printf("[RTOS] create AppTask failed\r\n");
		while (1);
	}

	/* 心跳任务: 只翻转 PC13, 与应用任务解耦, 也是以后加任务的抄写模板 */
	if (xTaskCreate(LedTask, "LED", LED_TASK_STACK, NULL, LED_TASK_PRIO, NULL) != pdPASS)
	{
		printf("[RTOS] create LedTask failed\r\n");
		while (1);
	}

	vTaskStartScheduler();		/* 正常情况下不返回 */

	/* 只有堆不足导致调度器起不来才会走到这里 */
	printf("[RTOS] scheduler start failed (heap?)\r\n");
	while (1);
}

/***************************************************************************************************
*	函 数 名: AppTask
*	函数功能: 应用主任务 —— 承载原来 main() 的 while(1) 主循环
*
*	说    明: 周期判定仍用 Get_Ms()(TIM2 时基), 与裸机版本的时间语义完全一致;
*			  循环末尾 vTaskDelay(1) 让出 1 个 tick, 避免独占 CPU 把心跳任务饿死。
*			  12 处 Delay_ms() 现在会在任务里自动转成 vTaskDelay, 不再死等。
****************************************************************************************************/
static void AppTask(void *pvParameters)
{
	uint32_t now;
	uint32_t lastEnc    = 0;	// 编码器反馈刷新(20ms, 仅显示, 无PID)
	uint32_t lastKey    = 0;
	uint32_t lastSensor = 0;
	uint32_t lastLCD    = 0;

	(void)pvParameters;

	for (;;)
	{
		now = Get_Ms();

		/* 20ms: 编码器反馈刷新 —— 只更新菜单显示用的轮速, PID闭环保持注释不启用 */
		if ((uint32_t)(now - lastEnc) >= 20)
		{
			lastEnc = now;
			Car_SpeedUpdate();
		}

		/* 50ms: 按键菜单 */
		if ((uint32_t)(now - lastKey) >= 50)
		{
			lastKey = now;
			Menu_KeyProcess();
		}

		/* 100ms: 传感器采样 */
		if ((uint32_t)(now - lastSensor) >= 100)
		{
			lastSensor = now;
			App_SensorTask(now);
		}

		/* 300ms: LCD界面刷新 */
		if ((uint32_t)(now - lastLCD) >= 300)
		{
			lastLCD = now;
			Menu_Refresh();
		}

		/* 每次循环: 处理通信指令 */
		App_BTProcess();
		App_ASRProcess();
		K230_Process();			// K230视觉指令
		K230_GrabTask();		// 夹取挂起超时收尾
		LanyaCon_Task(now);		// 蓝牙遥控掉线保护
		Buzzer_Task();			// 非阻塞蜂鸣到点静音

		/* 模式任务 */
		if (g_mode == MODE_AVOID)      App_AvoidTask(now);
		else if (g_mode == MODE_RADAR) App_RadarTask(now);

		vTaskDelay(1);			/* 让出 1 tick */
	}
}

/***************************************************************************************************
*	函 数 名: LedTask
*	函数功能: 心跳任务 —— 每 500ms 翻转板载 LED1(PC13)
*
*	说    明: 用 vTaskDelayUntil 保证周期不漂移。它是"调度器真的在运行"的最直观证据:
*			  即使 AppTask 卡在超声波 30ms 超时里, 这颗灯也照样 500ms 跳一次。
****************************************************************************************************/
static void LedTask(void *pvParameters)
{
	TickType_t lastWake = xTaskGetTickCount();

	(void)pvParameters;

	for (;;)
	{
		LED1_Toggle;
		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));
	}
}

/*************************************************************************************************
*	函 数 名:	App_Init
*
*	函数功能:	全部外设初始化 + LCD开机画面
*
*************************************************************************************************/
static void App_Init(void)
{
	BSP_Init();			// board/module init (BSP layer)
	g_mpuOK = BSP_IsMpuReady();

	printf("\r\n================================\r\n");
	printf(" Smart Car STM32F407ZGT6\r\n");
	printf(" MPU6050 : %s\r\n", g_mpuOK ? "OK" : "NOT FOUND");
	printf(" GyroZero: %s (biasZ=%.3f dps)\r\n",
		   Location_IsCalibrated() ? "OK" : "FAIL(check static!)", Location_GetGyroBias());
	printf(" SD card : %s\r\n", NavLog_GetStateName());
	printf("================================\r\n");

	Menu_SplashScreen();	// LCD开机画面 + 开机提示音
}

/*************************************************************************************************
*	函 数 名:	App_BTProcess
*
*	函数功能:	蓝牙/WiFi单字符遥控指令处理
*
*************************************************************************************************/
static void App_BTProcess(void)
{
	uint8_t cmd;

	while (BT_ReadByte(&cmd))
	{
		if (LanyaCon_Feed(cmd)) continue;	// 属于摇杆遥控帧的字节, 不走单字符指令

		switch (cmd)
		{
			case 'F': if (g_mode == MODE_MANUAL) Car_Move(CAR_DIR_FORWARD,  g_speed); break;
			case 'B': if (g_mode == MODE_MANUAL) Car_Move(CAR_DIR_BACKWARD, g_speed); break;
			case 'L': if (g_mode == MODE_MANUAL) Car_Turn(CAR_DIR_LEFT,     g_speed); break;
			case 'R': if (g_mode == MODE_MANUAL) Car_Turn(CAR_DIR_RIGHT,    g_speed); break;
			case 'S': Car_Stop(); break;

			case '+':
				g_speed += SPEED_STEP;
				if (g_speed > SPEED_MAX) g_speed = SPEED_MAX;
				break;

			case '-':
				g_speed -= SPEED_STEP;
				if (g_speed < SPEED_MIN) g_speed = SPEED_MIN;
				break;

			case 'M': App_SwitchMode(); break;

			default: break;
		}
	}
}

/*************************************************************************************************
*	函 数 名:	App_ASRProcess
*
*	函数功能:	ASRPRO语音识别结果处理(按行匹配字符串)
*
*	说    明:	在ASRPRO上位机中将各词条的串口输出配置为对应字符串即可,
*				也可以把下面的匹配串改成自己定义的(如拼音 qianjin 等)
*
*************************************************************************************************/
static void App_ASRProcess(void)
{
	char line[ASR_LINE_BUF_SIZE];

	if (!ASR_ReadLine(line, sizeof(line))) return;

	printf("ASR: %s\r\n", line);

	if (g_mode == MODE_MANUAL)
	{
		if      (strstr(line, "forward")) Car_Move(CAR_DIR_FORWARD,  g_speed);
		else if (strstr(line, "back"))    Car_Move(CAR_DIR_BACKWARD, g_speed);
		else if (strstr(line, "left"))    Car_Move(CAR_DIR_LEFT,     g_speed);
		else if (strstr(line, "right"))   Car_Move(CAR_DIR_RIGHT,    g_speed);
		else if (strstr(line, "stop"))    Car_Stop();
	}
}

/*************************************************************************************************
*	函 数 名:	App_SwitchMode
*
*	函数功能:	循环切换工作模式, 切换前停车并复位相关状态
*
*	说    明:	按键(KEY3)/蓝牙('M')共用, 声明在 menu.h
*				进入/离开 VISION 模式时会通知K230开始或停止输出目标,
*				这样K230(视觉)与摇杆/语音(手动)不会同时抢底盘控制权。
*
*************************************************************************************************/
void App_SwitchMode(void)
{
	WorkMode_TypeDef old = g_mode;

	Car_Stop();
	g_mode = (WorkMode_TypeDef)((g_mode + 1) % MODE_NUM);

	g_avoidState = AVOID_FORWARD;
	g_servoAngle = 90;
	Servo_SetAngle(g_servoAngle);

	/* 通知K230跟随模式变化 */
	if (g_mode == MODE_VISION && old != MODE_VISION)      K230_StartTrack();
	else if (g_mode != MODE_VISION && old == MODE_VISION) K230_StopTrack();

	Buzzer_BeepAsync(2500, 60);		// 非阻塞提示音, 不再卡住主循环
	printf("Mode -> %s\r\n", Menu_GetModeName());
}

/*************************************************************************************************
*	函 数 名:	App_SensorTask
*
*	函数功能:	100ms周期采样: 固定超声波 + MPU6050 + 融合定位 + 轨迹记录;
*				雷达超声波在非雷达模式下也以300ms低速率采样(供菜单显示)
*
*	说    明:	HC-SR04 是轮询式测距, 前方没有回波时要等满 ULTRA_TIMEOUT_US(30ms)
*				才返回。空旷场地里这几乎是每次调用都会发生, 100ms周期下等于
*				主循环有30%的时间被占住, 期间GPS语句解析、K230组帧、遥控指令
*				都得排队。所以这里把车头超声波的采样率做成自适应:
*				避障模式必须100ms一次(安全相关), 其它模式只是给菜单显示,
*				300ms一次足够, 把时间让给通信和定位。
*
*************************************************************************************************/
static void App_SensorTask(uint32_t now)
{
	static uint32_t lastMpu      = 0;
	static uint32_t lastRadar    = 0;
	static uint32_t lastFixedUlt = 0;

	/* 车头距离: 避障模式每100ms, 其它模式每300ms */
	if (g_mode == MODE_AVOID || (uint32_t)(now - lastFixedUlt) >= 300)
	{
		lastFixedUlt = now;
		g_distFixed  = Ultrasonic_GetDistance(ULTRA_FIXED);
		App_UltraAlarm(g_distFixed);	// 2~5cm过近蜂鸣警报(带毛刺过滤)
	}

	if (g_mode != MODE_RADAR && (uint32_t)(now - lastRadar) >= 300)
	{
		lastRadar   = now;
		g_distRadar = Ultrasonic_GetDistance(ULTRA_RADAR);	// 雷达模式由App_RadarTask采样
	}

	Location_Update(now);								// GPS + 惯导融合定位
	NavLog_Update(now);									// 轨迹记录(精度过滤后写CSV)

	if (g_mpuOK && (uint32_t)(now - lastMpu) >= 200)
	{
		lastMpu = now;
		MPU6050_ReadData(&g_mpu);
	}
}

/*************************************************************************************************
*	函 数 名:	App_UltraAlarm
*
*	入口参数:	dist - 本次车头超声波距离(cm)
*
*	函数功能:	过近警报: 距离落在 2~5cm 区间时蜂鸣器持续鸣叫, 离开区间后静音
*
*	说    明:	HC-SR04 近距离读数常有毛刺(假回波/回波丢失), 单次超限不动作:
*				连续 ALARM_ON_COUNT 次落在区间内才开警报,
*				连续 ALARM_OFF_COUNT 次离开区间才关警报(回差防抖, 边界处不闪断)。
*				由 App_SensorTask 在每次车头超声波采样后调用, 音调见 ALARM_FREQ_HZ。
*
*************************************************************************************************/
static void App_UltraAlarm(float dist)
{
	static uint8_t s_onCnt  = 0;	// 连续落在警报区间的次数
	static uint8_t s_offCnt = 0;	// 连续离开警报区间的次数
	static uint8_t s_active = 0;	// 警报是否已触发

	if (dist >= ALARM_NEAR_MIN_CM && dist <= ALARM_NEAR_MAX_CM)
	{
		s_offCnt = 0;
		if (s_onCnt < 255) s_onCnt++;
	}
	else
	{
		s_onCnt = 0;
		if (s_offCnt < 255) s_offCnt++;
	}

	if (!s_active && s_onCnt >= ALARM_ON_COUNT)
	{
		s_active = 1;
		Buzzer_On(ALARM_FREQ_HZ);
	}
	else if (s_active && s_offCnt >= ALARM_OFF_COUNT)
	{
		s_active = 0;
		Buzzer_Off();
	}
}

/*************************************************************************************************
*	函 数 名:	App_AvoidTask
*
*	函数功能:	自动避障状态机(非阻塞)
*
*	说    明:	巡航时车头超声波检测到障碍 -> 停车 -> 舵机左右扫描 ->
*				转向空旷一侧 -> 继续巡航
*
*************************************************************************************************/
static void App_AvoidTask(uint32_t now)
{
	switch (g_avoidState)
	{
		case AVOID_FORWARD:
			Car_Move(CAR_DIR_FORWARD, g_speed);
			if (g_distFixed < AVOID_DIST_CM)	// 发现障碍
			{
				Car_Stop();
				Buzzer_BeepAsync(3000, 100);	// 非阻塞, 不打断避障状态机
				CarLED_AllOn();
				g_avoidState = AVOID_SCAN;
				g_avoidTimer = now;
				g_servoAngle = 45;					// 先看右侧(角度定义以实际安装为准)
				Servo_SetAngle(g_servoAngle);
			}
			break;

		case AVOID_SCAN:
			/* 舵机到位需要一点时间, 分两步测左右距离 */
			if (g_servoAngle == 45 && (uint32_t)(now - g_avoidTimer) >= 400)
			{
				g_distRight  = Ultrasonic_GetDistance(ULTRA_RADAR);
				g_servoAngle = 135;
				Servo_SetAngle(g_servoAngle);
				g_avoidTimer = now;
			}
			else if (g_servoAngle == 135 && (uint32_t)(now - g_avoidTimer) >= 400)
			{
				g_distLeft   = Ultrasonic_GetDistance(ULTRA_RADAR);
				g_servoAngle = 90;
				Servo_SetAngle(g_servoAngle);
				g_avoidTimer = now;
				g_avoidState = AVOID_TURN;
				printf("Avoid: L=%.0f R=%.0f\r\n", g_distLeft, g_distRight);
			}
			break;

		case AVOID_TURN:
			if ((uint32_t)(now - g_avoidTimer) < AVOID_TURN_MS)
			{
				/* 转向距离远的一侧, 两侧都近则后退 */
				if (g_distLeft > g_distRight && g_distLeft > AVOID_DIST_CM)
					Car_Move(CAR_DIR_LEFT, g_speed);
				else if (g_distRight > AVOID_DIST_CM)
					Car_Move(CAR_DIR_RIGHT, g_speed);
				else
					Car_Move(CAR_DIR_BACKWARD, g_speed);
			}
			else
			{
				CarLED_AllOff();
				g_avoidState = AVOID_FORWARD;
			}
			break;

		default:
			g_avoidState = AVOID_FORWARD;
			break;
	}
}

/*************************************************************************************************
*	函 数 名:	App_RadarTask
*
*	函数功能:	雷达扫描演示: 舵机 0~180 往复摆动, 逐点测距
*
*	说    明:	扫描数据通过 USART1 以 "RADAR,角度,距离" 格式输出,
*				可在电脑上用串口绘图工具观察
*
*************************************************************************************************/
static void App_RadarTask(uint32_t now)
{
	static uint32_t lastStep = 0;
	static int8_t   dir = 1;

	if ((uint32_t)(now - lastStep) < 40) return;	// 每40ms走一步
	lastStep = now;

	g_servoAngle += dir * 3;
	if (g_servoAngle >= 180) { g_servoAngle = 180; dir = -1; }
	if (g_servoAngle <= 0)   { g_servoAngle = 0;   dir =  1; }

	Servo_SetAngle(g_servoAngle);
	Delay_ms(15);	// 等舵机稳定
	g_distRadar = Ultrasonic_GetDistance(ULTRA_RADAR);

	printf("RADAR,%d,%d\r\n", (int)g_servoAngle, (int)g_distRadar);
}

/*=================================================================================================
 *                          FreeRTOS 回调函数
 *  对应 MY/FreeRTOSConfig.h:
 *      configCHECK_FOR_STACK_OVERFLOW = 2
 *      configUSE_MALLOC_FAILED_HOOK   = 1
 *      configASSERT(x) -> vAssertCalled()
 *===============================================================================================*/

/* 栈溢出: 说明某个任务的栈写穿了。
 * 这里故意不调 printf —— 此时栈已经不可信, 打印大概率二次 HardFault。
 * 调试方法: 全速跑, 触发后在调试器里看 g_stackOverflowTask 指向的任务名,
 *           回到 xTaskCreate 处把对应 APP_TASK_STACK / LedTask 的栈调大。 */
volatile const char *g_stackOverflowTask = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	g_stackOverflowTask = pcTaskName;
	taskDISABLE_INTERRUPTS();
	for (;;);
}

/* 堆分配失败: 任务/队列/信号量创建时 configTOTAL_HEAP_SIZE(36KB) 不够会进这里 */
void vApplicationMallocFailedHook(void)
{
	printf("[RTOS] MALLOC FAILED: configTOTAL_HEAP_SIZE too small\r\n");
	taskDISABLE_INTERRUPTS();
	for (;;);
}

/* 断言失败: 关中断后死循环。调试时在这里下断点, 读 pcFile / ulLine 即可定位。
 * (原来的实现是 printf, 在 RTOS 下可能在 fputc 互斥量里重入而死等, 已替换) */
void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
	(void)pcFile;
	(void)ulLine;
	__disable_irq();
	for (;;);
}

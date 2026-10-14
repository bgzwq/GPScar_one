/***
	***************************************************************************************************
	*	@file  	menu.c
	*	@brief  STM32F407ZGT6 智能小车 人机交互(按键菜单 + LCD分页界面)
   ****************************************************************************************************
   *  @description
	*
	*	按键分配:
	*	  KEY1: 上一页   KEY2: 下一页   KEY3: 切换工作模式
	*	  KEY4: 电机自检 (地图页变为"保存地图到SD卡")
	*	  长按: 地图页 KEY1/KEY2=缩小/放大视野, KEY4=恢复自动缩放
	*
	*	分页内容:
	*	  1 OVERVIEW 概览(模式/目标速度/轮速/电压/距离/定位来源)
	*	  2 MOTOR    电机(目标/反馈/占空比/方向脚电平/编码器累计)
	*	  3 SENSOR   传感器(超声波/舵机/电压/亮度/湿度)
	*	  4 MPU6050  姿态(加速度/角速度/温度)
	*	  5 GPS      定位(来源/经纬度/速度/航向/卫星)
	*	  6 IO       按键与LED实时状态
	*	  7 MAP      轨迹地图(走过的路画成图, 可存到SD卡)
	*
	*	运行状态变量(g_mode/g_speed 等)定义在 main.c, 通过 menu.h 中的
	*	extern 声明共享; 模式切换由 main.c 的 App_SwitchMode 完成。
	*
	***************************************************************************************************
***/


#include "menu.h"
#include "delay.h"
#include "timer.h"
#include "lcd_spi_154.h"
#include "led.h"
#include "motor.h"
#include "encoder.h"
#include "car.h"
#include "sensor_adc.h"
#include "buzzer.h"
#include "car_led.h"
#include "car_key.h"
#include "location.h"
#include "gps.h"
#include "navlog.h"
#include "mapview.h"
#include "K230.h"


/* LCD/串口用的模式名(定宽6字符, 防止刷新残留) */
static const char *s_modeName[MODE_NUM] = { "MANUAL", "AVOID ", "RADAR ", "VISION" };

/*---------------------- 分页定义 ----------------------------*/
typedef enum
{
	PAGE_OVERVIEW = 0,	// 概览
	PAGE_MOTOR,			// 电机
	PAGE_SENSOR,		// 传感器
	PAGE_MPU,			// MPU6050姿态
	PAGE_GPS,			// GPS融合定位
	PAGE_IO,			// 按键/LED状态
	PAGE_MAP,			// 轨迹地图
	PAGE_NUM
} MenuPage_TypeDef;

static uint8_t s_page = PAGE_OVERVIEW;

/* 地图保存结果的临时提示(只显示几秒, 之后回到正常信息行) */
static const char *s_mapMsg = 0;
static uint32_t    s_mapMsgUntil = 0;

/* 页名(定宽4字符) */
static const char *s_pageName[PAGE_NUM] =
{ "OVER", "MOTR", "SENS", "MPU ", "GPS ", "IO  ", "MAP " };

/* 电机方向脚查找表(与motor.h一致), 用于回读实际电平 */
static const struct { GPIO_TypeDef *port; uint16_t pin; } s_motorDirPin[4][2] =
{
	{ {M1_IN1_PORT, M1_IN1_PIN}, {M1_IN2_PORT, M1_IN2_PIN} },
	{ {M2_IN1_PORT, M2_IN1_PIN}, {M2_IN2_PORT, M2_IN2_PIN} },
	{ {M3_IN1_PORT, M3_IN1_PIN}, {M3_IN2_PORT, M3_IN2_PIN} },
	{ {M4_IN1_PORT, M4_IN1_PIN}, {M4_IN2_PORT, M4_IN2_PIN} },
};

/* 函数声明 */
static void Menu_SelfTestStart(void);
static void Menu_SelfTestTick(void);
static void Menu_PageOverview(void);
static void Menu_PageMotor(void);
static void Menu_PageSensor(void);
static void Menu_PageMpu(void);
static void Menu_PageGps(void);
static void Menu_PageIO(void);
static void Menu_PageMap(void);


/*************************************************************************************************
*	函 数 名:	Menu_GetModeName
*
*	函数功能:	返回当前模式名字符串
*
*************************************************************************************************/
const char *Menu_GetModeName(void)
{
	return s_modeName[g_mode];
}

/*************************************************************************************************
*	函 数 名:	Menu_SplashScreen
*
*	函数功能:	LCD开机画面 + 开机提示音
*
*************************************************************************************************/
void Menu_SplashScreen(void)
{
	LCD_ShowNumMode(Fill_Space);	// 数字多余位补空格(默认补0, 显示难看)

	LCD_SetBackColor(LCD_BLACK);
	LCD_Clear();
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_SetColor(LCD_CYAN);
	LCD_DisplayString(20, 90, "Smart Car");
	LCD_SetColor(LCD_WHITE);
	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_DisplayString(20, 120, "STM32F407ZGT6");

	Buzzer_Beep(2000, 80);	// 开机提示音
	Delay_ms(800);

	LCD_SetBackColor(LCD_BLACK);
	LCD_Clear();
}

/*************************************************************************************************
*	函 数 名:	Menu_KeyProcess
*
*	函数功能:	按键菜单处理(50ms周期调用, 全程非阻塞)
*
*				KEY1: 上一页      KEY2: 下一页    KEY3: 切换工作模式
*				KEY4: 电机自检(地图页: 保存地图到SD卡)
*				长按: 地图页 KEY1/KEY2 = 视野缩小/放大, KEY4 = 恢复自动缩放
*
*	说    明:	CarKey_Poll() 只做一次非阻塞扫描, 事件从队列里取,
*				不再有 "while(按下);" 那种会把主循环卡住的等待。
*
*************************************************************************************************/
void Menu_KeyProcess(void)
{
	uint8_t key;
	uint8_t ev;

	Menu_SelfTestTick();	// 电机自检状态机(非阻塞)

	CarKey_Poll();

	while ((ev = CarKey_GetEvent(&key)) != CAR_KEY_EVENT_NONE)
	{
		/* --- 地图页的长按: 缩放/清空轨迹 --- */
		if (s_page == PAGE_MAP && ev == CAR_KEY_EVENT_LONG)
		{
			switch (key)
			{
				case CAR_KEY1:	MapView_ZoomOut(); Buzzer_BeepAsync(2200, 30); break;
				case CAR_KEY2:	MapView_ZoomIn();  Buzzer_BeepAsync(2200, 30); break;

				case CAR_KEY4:	/* 清空当前轨迹并恢复自动缩放(重新开始画一张新图)。
								 * 注意: 已经写进 TRACK.CSV 的历史数据不会被删除,
								 * 只是RAM里的绘图轨迹清零。 */
					NavLog_Clear();
					MapView_NotifyCleared();
					LCD_Clear();
					Buzzer_BeepAsync(1800, 120);
					break;

				default: break;
			}
			continue;
		}

		if (ev != CAR_KEY_EVENT_PRESS) continue;	// 其余长按暂不使用

		switch (key)
		{
			case CAR_KEY1:	// 上一页
				s_page = (s_page == 0) ? (PAGE_NUM - 1) : (s_page - 1);
				LCD_Clear();	// 换页清屏, 防止上一页内容残留
				break;

			case CAR_KEY2:	// 下一页
				s_page = (s_page + 1) % PAGE_NUM;
				LCD_Clear();
				break;

			case CAR_KEY3:	// 切换模式
				App_SwitchMode();
				break;

			case CAR_KEY4:	// 地图页: 保存地图; 其它页: 电机自检
				if (s_page == PAGE_MAP) Menu_MapSave();
				else                    Menu_SelfTestStart();
				break;

			default: break;
		}
	}
}

/*************************************************************************************************
*	函 数 名:	Menu_Refresh
*
*	函数功能:	LCD分页界面刷新(300ms周期调用)
*
*	说    明:	页眉显示 页码/页名/工作模式, 页脚显示按键提示
*
*************************************************************************************************/
void Menu_Refresh(void)
{
	/* 页眉: SmartCar 页码/总页数 页名 模式 */
	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_SetColor(LCD_CYAN);
	LCD_DisplayString(4, 0, "SmartCar");
	LCD_DisplayNumber(76, 0, s_page + 1, 1);
	LCD_DisplayString(84, 0, "/");
	LCD_DisplayNumber(92, 0, PAGE_NUM, 1);
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(108, 0, (char *)s_pageName[s_page]);
	LCD_SetColor(LCD_YELLOW);
	LCD_DisplayString(188, 0, (char *)s_modeName[g_mode]);

	/* 页面内容 */
	switch (s_page)
	{
		case PAGE_OVERVIEW: Menu_PageOverview(); break;
		case PAGE_MOTOR:    Menu_PageMotor();    break;
		case PAGE_SENSOR:   Menu_PageSensor();   break;
		case PAGE_MPU:      Menu_PageMpu();      break;
		case PAGE_GPS:      Menu_PageGps();      break;
		case PAGE_IO:       Menu_PageIO();       break;
		case PAGE_MAP:      Menu_PageMap();      break;
		default:            s_page = PAGE_OVERVIEW; break;
	}

	/* 地图页自己画信息行和页脚(内容更合适), 其它页用通用页脚 */
	if (s_page == PAGE_MAP) return;

	/* 非地图页: 明确告诉地图模块"现在不可见"。
	 * 换页时 Menu_KeyProcess 会 LCD_Clear(), 如果这里不通知,
	 * 下次再进地图页时模块会以为画面还在而跳过重绘, 结果地图区域一片空白。 */
	MapView_Refresh(0);

	/* 页脚: 按键提示 */
	LCD_SetColor(LIGHT_GREY);
	LCD_SetAsciiFont(&ASCII_Font12);
	LCD_DisplayString(4, 226, "K1:Prev K2:Next K3:Mode K4:Test");
}

/*************************************************************************************************
*	函 数 名:	Menu_MapSave
*
*	函数功能:	把当前轨迹地图渲染成BMP存到SD卡, 并把CSV刷盘
*
*	说    明:	地图页KEY4(短按)触发。会阻塞一两百毫秒(写约136KB), 属用户主动操作。
*				保存结果用一条3秒的临时提示显示, 否则300ms后就被信息行覆盖掉了。
*
*************************************************************************************************/
void Menu_MapSave(void)
{
	LCD_SetAsciiFont(&ASCII_Font12);
	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 212, "Saving map to SD...");

	if (MapView_SaveToSd())
	{
		Buzzer_BeepAsync(3000, 80);
		s_mapMsg = "Saved CARLOG/MAP.BMP";
	}
	else
	{
		Buzzer_BeepAsync(800, 200);
		s_mapMsg = "Save FAILED (SD card?)";
	}
	s_mapMsgUntil = Get_Ms() + 3000;
}

/*---------------------- 各页面绘制 ---------------------------*/

/*************************************************************************************************
*	页1 概览: 目标速度/轮速/电压/距离/定位来源
*************************************************************************************************/
static void Menu_PageOverview(void)
{
	Location_DataTypeDef *loc = Location_GetData();

	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_SetColor(LCD_WHITE);

	/* 目标速度 + 闭环状态 */
	LCD_DisplayString(4, 24, "Target:"); LCD_DisplayNumber(70, 24, g_speed, 4);
	LCD_DisplayString(140, 24, "Run:");
	LCD_DisplayString(180, 24, Car_IsRunning() ? "YES" : "NO ");

	/* 轮速反馈 */
	LCD_DisplayString(4, 44, "W1:"); LCD_DisplayNumber(40, 44, Car_GetSpeed(0), 4);
	LCD_DisplayString(120, 44, "W2:"); LCD_DisplayNumber(156, 44, Car_GetSpeed(1), 4);
	LCD_DisplayString(4, 64, "W3:"); LCD_DisplayNumber(40, 64, Car_GetSpeed(2), 4);
	LCD_DisplayString(120, 64, "W4:"); LCD_DisplayNumber(156, 64, Car_GetSpeed(3), 4);

	/* 电源电压 */
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(4, 88, "PWR1:"); LCD_DisplayDecimals(56, 88, SensorADC_GetVoltage(ADC_CH_POWER1), 4, 2);
	LCD_DisplayString(120, 88, "PWR2:"); LCD_DisplayDecimals(172, 88, SensorADC_GetVoltage(ADC_CH_POWER2), 4, 2);

	/* 距离与舵机 */
	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 112, "DistF:"); LCD_DisplayNumber(64, 112, (int32_t)g_distFixed, 4);
	LCD_DisplayString(120, 112, "Servo:"); LCD_DisplayNumber(180, 112, (int32_t)g_servoAngle, 3);

	/* 定位概览 */
	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 136, "GPS:"); LCD_DisplayString(44, 136, (char *)Location_GetSourceName());
	LCD_DisplayString(120, 136, "Sats:"); LCD_DisplayNumber(172, 136, loc->satellites, 2);
	LCD_DisplayString(4, 156, "Spd:"); LCD_DisplayDecimals(44, 156, loc->speedKmh, 5, 1);
	LCD_DisplayString(120, 156, "km/h");
}

/*************************************************************************************************
*	页2 电机: 每电机 目标/反馈/占空比 + 方向脚电平 + 编码器累计
*************************************************************************************************/
static void Menu_PageMotor(void)
{
	uint8_t i;

	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_SetColor(LCD_WHITE);

	/* 每行一个电机: M1 T:目标 F:反馈 P:占空比 */
	for (i = 0; i < MOTOR_NUM; i++)
	{
		uint16_t y = 24 + i * 20;

		LCD_DisplayString(4, y, "M"); LCD_DisplayNumber(20, y, i + 1, 1);
		LCD_DisplayString(36, y, "T:"); LCD_DisplayNumber(64, y, Car_GetTarget(i), 5);
		LCD_DisplayString(108, y, "F:"); LCD_DisplayNumber(132, y, Car_GetSpeed(i), 5);
		LCD_DisplayString(176, y, "P:"); LCD_DisplayNumber(200, y, Motor_GetPwm(i), 4);
	}

	/* 方向脚实际电平: 正转应为 10 */
	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 108, "Dir");
	for (i = 0; i < MOTOR_NUM; i++)
	{
		uint16_t x = 44 + i * 48;

		LCD_DisplayNumber(x, 108, i + 1, 1);
		LCD_DisplayString(x + 8, 108, ":");
		LCD_DisplayNumber(x + 16, 108,
			(int32_t)GPIO_ReadOutputDataBit(s_motorDirPin[i][0].port, s_motorDirPin[i][0].pin), 1);
		LCD_DisplayNumber(x + 24, 108,
			(int32_t)GPIO_ReadOutputDataBit(s_motorDirPin[i][1].port, s_motorDirPin[i][1].pin), 1);
	}

	/* 编码器累计脉冲 */
	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 132, "ENC1:"); LCD_DisplayNumber(56, 132, Encoder_GetTotal(0), 8);
	LCD_DisplayString(124, 132, "ENC2:"); LCD_DisplayNumber(176, 132, Encoder_GetTotal(1), 8);
	LCD_DisplayString(4, 152, "ENC3:"); LCD_DisplayNumber(56, 152, Encoder_GetTotal(2), 8);
	LCD_DisplayString(124, 152, "ENC4:"); LCD_DisplayNumber(176, 152, Encoder_GetTotal(3), 8);

	LCD_SetColor(LIGHT_GREY);
	LCD_DisplayString(4, 176, "T:target F:feedback P:PWM");
}

/*************************************************************************************************
*	页3 传感器: 超声波/舵机/电压/亮度/湿度
*************************************************************************************************/
static void Menu_PageSensor(void)
{
	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_SetColor(LIGHT_CYAN);

	LCD_DisplayString(4, 24, "DistF:"); LCD_DisplayNumber(64, 24, (int32_t)g_distFixed, 4);
	LCD_DisplayString(120, 24, "cm");
	LCD_DisplayString(4, 44, "DistR:"); LCD_DisplayNumber(64, 44, (int32_t)g_distRadar, 4);
	LCD_DisplayString(120, 44, "cm");
	LCD_DisplayString(4, 64, "Servo:"); LCD_DisplayNumber(64, 64, (int32_t)g_servoAngle, 3);
	LCD_DisplayString(120, 64, "deg");

	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(4, 88, "PWR1:"); LCD_DisplayDecimals(56, 88, SensorADC_GetVoltage(ADC_CH_POWER1), 4, 2);
	LCD_DisplayString(120, 88, "V");
	LCD_DisplayString(4, 108, "PWR2:"); LCD_DisplayDecimals(56, 108, SensorADC_GetVoltage(ADC_CH_POWER2), 4, 2);
	LCD_DisplayString(120, 108, "V");

	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 132, "Light:"); LCD_DisplayNumber(64, 132, SensorADC_GetRaw(ADC_CH_LIGHT), 4);
	LCD_DisplayDecimals(120, 132, SensorADC_GetVoltage(ADC_CH_LIGHT), 4, 2);
	LCD_DisplayString(160, 132, "V");
	LCD_DisplayString(4, 152, "Humi :"); LCD_DisplayNumber(64, 152, SensorADC_GetRaw(ADC_CH_HUMIDITY), 4);
	LCD_DisplayDecimals(120, 152, SensorADC_GetVoltage(ADC_CH_HUMIDITY), 4, 2);
	LCD_DisplayString(160, 152, "V");

	LCD_SetColor(LIGHT_GREY);
	LCD_DisplayString(4, 176, "Light/Humi: raw + pin V");
}

/*************************************************************************************************
*	页4 MPU6050: 加速度/角速度/温度(显示矫正后的数据)
*
*	说    明:	加速度显示 = 原始值 + 零位矫正值(mpu6050.h 的 AX/AY/AZ_errot),
*				陀螺Z轴显示 = 原始值 - 上电静止校准的零偏(Location_GetGyroBias)。
*				静止水平放置时 AX/AY/GZ 应接近0, AZ 接近9.8。
*************************************************************************************************/
static void Menu_PageMpu(void)
{
	LCD_SetAsciiFont(&ASCII_Font16);

	if (!g_mpuOK)
	{
		LCD_SetColor(LIGHT_RED);
		LCD_DisplayString(4, 24, "MPU6050: NOT FOUND");
		return;
	}

	LCD_SetColor(LIGHT_MAGENTA);
	LCD_DisplayString(4, 24, "AX:"); LCD_DisplayDecimals(36, 24, g_mpu.accXms + AX_errot, 7, 2);
	LCD_DisplayString(140, 24, "m/s2");
	LCD_DisplayString(4, 44, "AY:"); LCD_DisplayDecimals(36, 44, g_mpu.accYms + AY_errot, 7, 2);
	LCD_DisplayString(140, 44, "m/s2");
	LCD_DisplayString(4, 64, "AZ:"); LCD_DisplayDecimals(36, 64, g_mpu.accZms + AZ_errot, 7, 2);
	LCD_DisplayString(140, 64, "m/s2");

	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 88, "GX:"); LCD_DisplayDecimals(36, 88, g_mpu.gyroXds, 7, 1);
	LCD_DisplayString(140, 88, "dps");
	LCD_DisplayString(4, 108, "GY:"); LCD_DisplayDecimals(36, 108, g_mpu.gyroYds, 7, 1);
	LCD_DisplayString(140, 108, "dps");
	LCD_DisplayString(4, 128, "GZ:"); LCD_DisplayDecimals(36, 128, g_mpu.gyroZds - Location_GetGyroBias(), 7, 1);
	LCD_DisplayString(140, 128, "dps");

	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 152, "Temp:"); LCD_DisplayDecimals(52, 152, g_mpu.temperature, 5, 1);
	LCD_DisplayString(140, 152, "C");

	/* 上电静止调零的结果: 开机时必须让小车静止, 这里确认零偏是否有效。
	 * Zero=FAIL 说明校准期间车在动(或MPU不在线), 此时航向会持续漂移, 需要重新上电校准。 */
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(4, 176, "Zero:");
	LCD_DisplayString(44, 176, Location_IsCalibrated() ? "OK  " : "FAIL");
	LCD_DisplayString(88, 176, "BiasZ:");
	LCD_DisplayDecimals(136, 176, Location_GetGyroBias(), 5, 2);

	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 192, "Noise:");
	LCD_DisplayDecimals(52, 192, Location_GetGyroNoise(), 5, 2);
	LCD_DisplayString(100, 192, "dps (static cal)");
}

/*************************************************************************************************
*	页5 GPS: 融合定位(来源/经纬度/速度/航向/卫星/UTC)
*************************************************************************************************/
static void Menu_PageGps(void)
{
	Location_DataTypeDef *loc = Location_GetData();
	GPS_DataTypeDef      *gps = GPS_GetData();

	LCD_SetAsciiFont(&ASCII_Font16);

	LCD_SetColor(LIGHT_YELLOW);
	LCD_DisplayString(4, 24, "Src:"); LCD_DisplayString(44, 24, (char *)Location_GetSourceName());
	LCD_DisplayString(120, 24, "Sats:"); LCD_DisplayNumber(172, 24, loc->satellites, 2);

	LCD_SetColor(LCD_WHITE);
	LCD_DisplayString(4, 48, "Lat:"); LCD_DisplayDecimals(44, 48, loc->latitude, 9, 5);
	LCD_DisplayString(4, 68, "Lon:"); LCD_DisplayDecimals(44, 68, loc->longitude, 9, 5);
	LCD_DisplayString(4, 88, "Spd:"); LCD_DisplayDecimals(44, 88, loc->speedKmh, 5, 1);
	LCD_DisplayString(110, 88, "km/h");

	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 112, "Course:"); LCD_DisplayDecimals(76, 112, loc->course, 6, 1);
	LCD_DisplayString(4, 132, "Head  :"); LCD_DisplayDecimals(76, 132, loc->heading, 6, 1);

	LCD_SetColor(LIGHT_GREY);
	LCD_DisplayString(4, 156, "Fix:"); LCD_DisplayNumber(44, 156, gps->fixQuality, 1);
	LCD_DisplayString(64, 156, "HDOP:"); LCD_DisplayDecimals(116, 156, gps->hdop, 4, 1);
	LCD_DisplayString(4, 176, "UTC:");
	LCD_DisplayNumber(44, 176, gps->hour, 2);   LCD_DisplayString(60, 176, ":");
	LCD_DisplayNumber(68, 176, gps->minute, 2); LCD_DisplayString(84, 176, ":");
	LCD_DisplayNumber(92, 176, gps->second, 2);

	/* 精度诊断行: 确认RMC真的在收(定位生效), 以及语句队列有没有丢数据 */
	LCD_SetColor(LIGHT_GREEN);
	LCD_DisplayString(4, 196, "RMC:"); LCD_DisplayNumber(40, 196, (int32_t)GPS_GetRmcCount(), 6);
	LCD_DisplayString(88, 196, "Drop:"); LCD_DisplayNumber(128, 196, (int32_t)GPS_GetQueueDropped(), 3);
	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 212, "Tk:"); LCD_DisplayNumber(28, 212, (int32_t)NavLog_GetCount(), 4);
	LCD_DisplayString(60, 212, "SD:"); LCD_DisplayString(84, 212, (char *)NavLog_GetStateName());
}

/*************************************************************************************************
*	页6 IO: 按键与LED实时状态
*************************************************************************************************/
static void Menu_PageIO(void)
{
	uint8_t i;

	LCD_SetAsciiFont(&ASCII_Font16);
	LCD_SetColor(LCD_WHITE);

	/* 按键实时状态: DOWN=按下 */
	for (i = 0; i < CAR_KEY_NUM; i++)
	{
		uint16_t x = 4 + (i % 2) * 116;
		uint16_t y = 24 + (i / 2) * 20;

		LCD_DisplayString(x, y, "KEY"); LCD_DisplayNumber(x + 24, y, i + 1, 1);
		LCD_DisplayString(x + 32, y, ":");
		LCD_DisplayString(x + 40, y, CarKey_GetState(CAR_KEY1 + i) ? "DOWN" : "UP  ");
	}

	/* 车身LED状态 */
	LCD_SetColor(LIGHT_YELLOW);
	for (i = 0; i < CAR_LED_NUM; i++)
	{
		uint16_t x = 4 + (i % 2) * 116;
		uint16_t y = 88 + (i / 2) * 20;

		LCD_DisplayString(x, y, "LED"); LCD_DisplayNumber(x + 24, y, i + 1, 1);
		LCD_DisplayString(x + 32, y, ":");
		LCD_DisplayString(x + 40, y, CarLED_Get(i) ? "ON " : "OFF");
	}

	/* 板载LED(PC13, 低电平点亮) */
	LCD_SetColor(LIGHT_CYAN);
	LCD_DisplayString(4, 132, "PC13:");
	LCD_DisplayString(52, 132,
		(GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN) == 0) ? "ON " : "OFF");
	LCD_DisplayString(120, 132, "(heartbeat)");
}

/*************************************************************************************************
*	页7 地图: 把小车走过的路画成"北朝上"的平面轨迹图
*
*	说    明:	1. 地图本身由 mapview.c 绘制在 MAP_AREA_* 指定的区域里,
*				   这里只负责画信息行和页脚
*				2. MapView_Refresh(1) 内部只在"轨迹有变化"时才重画,
*				   所以300ms刷新一次不会闪屏
*				3. 没有SD卡时轨迹依然记录在RAM里并正常显示, 只是不能保存
*
*************************************************************************************************/
static void Menu_PageMap(void)
{
	/* 地图区域(内部按需重绘) */
	MapView_Refresh(1);

	/* 信息行 */
	LCD_SetAsciiFont(&ASCII_Font12);

	/* 刚保存过地图 -> 先显示3秒结果提示 */
	if (s_mapMsg && (int32_t)(Get_Ms() - s_mapMsgUntil) < 0)
	{
		LCD_SetColor(LIGHT_GREEN);
		LCD_DisplayString(4, 212, (char *)s_mapMsg);
	}
	else
	{
		s_mapMsg = 0;

		LCD_SetColor(LIGHT_CYAN);
		LCD_DisplayString(4, 212, "Pts:");
		LCD_DisplayNumber(30, 212, (int32_t)NavLog_GetCount(), 4);

		LCD_SetColor(LIGHT_GREEN);
		LCD_DisplayDecimals(56, 212, NavLog_GetDistanceM(), 5, 1);
		LCD_DisplayString(88, 212, "m");

		LCD_SetColor(LIGHT_YELLOW);
		LCD_DisplayString(100, 212, "SD:");
		LCD_DisplayString(118, 212, (char *)NavLog_GetStateName());

		LCD_SetColor(LIGHT_MAGENTA);
		LCD_DisplayString(152, 212, "K2:");
		LCD_DisplayString(170, 212, (char *)K230_GetStateName());
	}

	/* 页脚 */
	LCD_SetColor(LIGHT_GREY);
	LCD_DisplayString(4, 226, "K4:Save holdK4:Clear holdK1/2:Zoom");
}

/*************************************************************************************************
*	函 数 名:	Menu_SelfTestStart / Menu_SelfTestTick
*
*	函数功能:	电机三段自检(非阻塞状态机), 结果在 MOTR 页观察(编码器累计/占空比/方向脚):
*				阶段1: 4个电机同时30%PWM转2秒
*				阶段2: 逐个电机单独转1秒
*				阶段3: 手转测试, 10秒内用手逐个转动轮子, 观察MOTR页4路编码器累计值,
*				       用于确认 编码器通道 与 轮子 的实际对应关系
*
*	说    明:	KEY4启动, Tick由 Menu_KeyProcess 每50ms调用;
*				测试期间 LCD/心跳/传感器 均正常运行
*
*************************************************************************************************/
typedef enum
{
	ST_IDLE = 0,	// 空闲
	ST1_RUN,		// 阶段1: 全部电机转2s
	ST1_REST,		// 阶段1: 停1s
	ST2_SETTLE,		// 阶段2: 单电机启动后等50ms回读电平
	ST2_RUN,		// 阶段2: 单电机转950ms
	ST2_REST,		// 阶段2: 停500ms换下一个
	ST3_HAND,		// 阶段3: 10s手转, 每秒打印
} SelfTestState_TypeDef;

static SelfTestState_TypeDef s_testState = ST_IDLE;
static uint32_t s_testTimer  = 0;
static uint8_t  s_testMotor  = 0;	// 阶段2当前电机
static uint8_t  s_testSecond = 0;	// 阶段3已过秒数

static void Menu_SelfTestStart(void)
{
	uint8_t i;

	if (s_testState != ST_IDLE) return;

	Car_Stop();	// 停车, 避免干扰自检(PID已注释)

	for (i = 0; i < MOTOR_NUM; i++) Encoder_ReadDelta(i);	// 清增量
	for (i = 0; i < MOTOR_NUM; i++) Motor_SetSpeed(i, 300);

	s_testTimer = Get_Ms();
	s_testState = ST1_RUN;
}

static void Menu_SelfTestTick(void)
{
	uint32_t now = Get_Ms();
	uint8_t  i;

	switch (s_testState)
	{
		case ST_IDLE:
			break;

		case ST1_RUN:	// 齐转2s到 -> 停车
			if ((uint32_t)(now - s_testTimer) >= 2000)
			{
				for (i = 0; i < MOTOR_NUM; i++) Encoder_ReadDelta(i);
				Motor_StopAll();
				s_testTimer = now;
				s_testState = ST1_REST;
			}
			break;

		case ST1_REST:	// 停1s -> 进入阶段2, 启动第一个电机
			if ((uint32_t)(now - s_testTimer) >= 1000)
			{
				s_testMotor = 0;
				Encoder_ReadDelta(s_testMotor);
				Buzzer_Beep(2000, 50);
				Motor_SetSpeed(s_testMotor, 300);
				s_testTimer = now;
				s_testState = ST2_SETTLE;
			}
			break;

		case ST2_SETTLE:	// 50ms电平稳定 -> 方向脚电平可在MOTR页回读
			if ((uint32_t)(now - s_testTimer) >= 50)
			{
				s_testTimer = now;
				s_testState = ST2_RUN;
			}
			break;

		case ST2_RUN:	// 再转950ms -> 停车
			if ((uint32_t)(now - s_testTimer) >= 950)
			{
				Motor_Stop(s_testMotor);
				Encoder_ReadDelta(s_testMotor);
				s_testTimer = now;
				s_testState = ST2_REST;
			}
			break;

		case ST2_REST:	// 停500ms -> 下一个电机, 全部测完进入阶段3
			if ((uint32_t)(now - s_testTimer) >= 500)
			{
				s_testMotor++;
				if (s_testMotor < MOTOR_NUM)
				{
					Encoder_ReadDelta(s_testMotor);
					Buzzer_Beep(2000, 50);
					Motor_SetSpeed(s_testMotor, 300);
					s_testTimer = now;
					s_testState = ST2_SETTLE;
				}
				else
				{
					for (i = 0; i < MOTOR_NUM; i++) Encoder_ClearTotal(i);
					s_testSecond = 0;
					s_testTimer  = now;
					s_testState  = ST3_HAND;
				}
			}
			break;

		case ST3_HAND:	// 每秒清一次计时, 共10s(期间手转轮子, 在MOTR页看ENC累计)
			if ((uint32_t)(now - s_testTimer) >= 1000)
			{
				s_testTimer = now;
				if (++s_testSecond >= 10)
				{
					s_testState = ST_IDLE;
				}
			}
			break;

		default:
			s_testState = ST_IDLE;
			break;
	}
}

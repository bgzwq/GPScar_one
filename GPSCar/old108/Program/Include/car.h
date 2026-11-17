#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx.h"
#include "motor.h"

/*---------------------- 底盘方向 ----------------------------*/
typedef enum
{
	CAR_DIR_STOP = 0,	// 停止
	CAR_DIR_FORWARD,	// 前进
	CAR_DIR_BACKWARD,	// 后退
	CAR_DIR_LEFT,		// 原地左转
	CAR_DIR_RIGHT,		// 原地右转
} CarDir_TypeDef;

/*---------------------- 开环参数(PID已注释) --------------------*/
/* 目标速度(脉冲/20ms)直接按比例映射为电机PWM千分比: PWM = 目标 × CAR_OPENLOOP_SCALE
 * 60档 ≈ 960‰, 嫌整体偏快/偏慢只调这一个数即可 */
#define CAR_OPENLOOP_SCALE	16

/*---------------------- PID参数(已整体注释) --------------------*/
/* 蓝牙遥控时PID闭环导致轮胎顿挫明显, 底盘改为开环PWM直驱.
 * 如需恢复速度闭环, 取消以下宏、Car_PID_Update 声明/定义及 main.c 中20ms调用的注释.
 *
 * 速度闭环: 目标值/反馈值单位都是 编码器脉冲数 / 控制周期(20ms)
 * 输出为电机PWM千分比 -1000~1000
 *
#define CAR_PID_KP		40.0f
#define CAR_PID_KI		6.0f
#define CAR_PID_KD		0.0f
#define CAR_PID_IMAX	600.0f		// 积分限幅
#define CAR_PID_OUTMAX	1000		// 输出限幅
*/

/* 左右轮目标速度极性: 差速/原地转向时右侧取反, 若实际装车方向不符可在此调整 */
#define CAR_LEFT_MOTORS		{ MOTOR_1, MOTOR_2 }	// 左侧: M1左前 M2左后
#define CAR_RIGHT_MOTORS	{ MOTOR_3, MOTOR_4 }	// 右侧: M3右前 M4右后

/* 行进中差速转弯: 内侧轮速度 = 外侧轮 x CAR_TURN_INNER_PCT% (数值越小转弯越急) */
#define CAR_TURN_INNER_PCT	40

/*---------------------- 函数声明 ----------------------------*/

void 	Car_Init(void);									// 底盘初始化(电机+编码器)
void 	Car_SetWheelSpeed(uint8_t id, int16_t target);	// 设置单轮速度(开环直接映射PWM)
void 	Car_Move(CarDir_TypeDef dir, int16_t speed);	// 整车运动
void 	Car_Turn(CarDir_TypeDef dir, int16_t speed);	// 转弯: 行进中差速转弯, 静止时原地转
void 	Car_Stop(void);									// 整车停止(直接制动)
void 	Car_SpeedUpdate(void);							// 每20ms刷新编码器反馈(仅显示用, 无PID)
//void 	Car_PID_Update(void);							// PID计算, 需每20ms调用一次 (已注释)
int16_t Car_GetSpeed(uint8_t id);						// 读取轮速反馈(脉冲/周期)
int16_t Car_GetTarget(uint8_t id);						// 读取轮速目标
uint8_t Car_IsRunning(void);							// 是否处于闭环运行状态

/*---------------------- 底盘命令源(谁在开车) --------------------*/
/*
 * 【为什么改成"投递命令"】
 * 原来 lanyacon.c / uart_host.c / K230.c / main.c 都直接调 Car_Move()/Car_SetWheelSpeed(),
 * 而 Car_Move() 内部是 4 次独立的 Car_SetWheelSpeed():
 *   1) 中间被更高优先级任务打断 -> 出现"左轮已改、右轮还没改"的半截状态, 车会抽一下;
 *   2) 两个源(例如摇杆与视觉)交替覆盖, 谁最后写谁赢, 行为不可预测。
 * 现在: 任何任务只调用 Car_Post*() 投递一条命令(小结构体, 临界区保护),
 *       只有 ControlTask 每 20ms 调一次 Car_ControlTask() 时才真正写电机。
 * Car_Stop() 例外: 它是"全零 + 制动"的幂等操作, 任何任务都可以直接调用(急停/切模式)。
 */
#define CAR_SRC_NONE	0
#define CAR_SRC_MANUAL	1	// 蓝牙摇杆 / ESP32 文本指令
#define CAR_SRC_AVOID	2	// 雷达避障(MissionTask)
#define CAR_SRC_VISION	3	// K230 视觉
#define CAR_SRC_TEST	4	// KEY4 电机自检

void 	Car_PostDir(uint8_t src, CarDir_TypeDef dir, int16_t speed);		// 整车按方向走
void 	Car_PostTurn(uint8_t src, CarDir_TypeDef dir, int16_t speed);	// 行进中差速转弯(等价 Car_Turn)
void 	Car_PostLR(uint8_t src, int16_t left, int16_t right);			// 直接给左右两侧速度
void 	Car_PostWheels(uint8_t src, const int16_t w[4]);				// 直接给4个轮子速度
void 	Car_PostWheel(uint8_t src, uint8_t id, int16_t speed);			// 单轮目标速度
void 	Car_PostPwm(uint8_t src, uint8_t id, int16_t pwm);				// 单轮原始PWM千分比(-1000~1000)
void 	Car_PostPwmAll(uint8_t src, int16_t pwm);						// 四轮同PWM(自检阶段1用)
void 	Car_PostBrake(uint8_t src);										// 四轮制动
void 	Car_ControlTask(void);											// 【只在 ControlTask 每20ms调一次】
void 	Car_TestLock(uint8_t on);										// 1=自检独占底盘(其它源命令被丢弃)
uint8_t Car_GetActiveSrc(void);											// 最近一次生效的命令源(诊断用)
#endif //__CAR_H

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
//void 	Car_PID_Update(void);							// PID计算, 需每20ms调用一次 (已注释)
int16_t Car_GetSpeed(uint8_t id);						// 读取轮速反馈(脉冲/周期)
int16_t Car_GetTarget(uint8_t id);						// 读取轮速目标
uint8_t Car_IsRunning(void);							// 是否处于闭环运行状态

#endif //__CAR_H

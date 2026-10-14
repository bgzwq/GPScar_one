/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   四驱底盘速度闭环控制
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 组合 motor.c 与 encoder.c, 4轮开环PWM直驱 (速度PID闭环已注释, 需要时取消注释恢复)
	*  2. Car_PID_Update() 已注释, 原需固定每 20ms 调用一次
	*  3. 目标/反馈速度单位: 编码器脉冲数 / 20ms (开环时按 CAR_OPENLOOP_SCALE 直接映射PWM)
	*  4. 车轮布局: M1=左前 M2=右前 M3=左后 M4=右后
	*
	************************************************************************************************
***/

#include "car.h"
#include "motor.h"
#include "encoder.h"

typedef struct
{
	float kp, ki, kd;
	float integral;
	float lastErr;
	int16_t target;		// 目标速度
	int16_t speed;		// 反馈速度
	uint8_t motorId;
	uint8_t encoderId;
} Wheel_TypeDef;

static Wheel_TypeDef s_wheel[MOTOR_NUM];
static uint8_t s_running = 0;

/*************************************************************************************************
*	函 数 名:	Car_Init
*
*	函数功能:	初始化电机、编码器与PID参数
*
*************************************************************************************************/
void Car_Init(void)
{
	uint8_t i;

	Motor_Init();
	Encoder_Init();

	for (i = 0; i < MOTOR_NUM; i++)
	{
		/* PID参数初始化已注释(开环直驱)
		s_wheel[i].kp        = CAR_PID_KP;
		s_wheel[i].ki        = CAR_PID_KI;
		s_wheel[i].kd        = CAR_PID_KD;
		s_wheel[i].integral  = 0;
		s_wheel[i].lastErr   = 0;
		*/
		s_wheel[i].target    = 0;
		s_wheel[i].speed     = 0;
		s_wheel[i].motorId   = i;
		s_wheel[i].encoderId = i;	// 电机与编码器一一对应
	}
	s_running = 0;
}

/*************************************************************************************************
*	函 数 名:	Car_SetWheelSpeed
*
*	入口参数:	id     - 轮号 0~3
*				target - 目标速度(编码器脉冲数/20ms), 负值为反转
*
*	函数功能:	设置单轮速度, PID已注释, 开环直接按 CAR_OPENLOOP_SCALE 映射输出PWM
*
*************************************************************************************************/
void Car_SetWheelSpeed(uint8_t id, int16_t target)
{
	if (id >= MOTOR_NUM) return;
	s_wheel[id].target = target;
	s_running = 1;

	Motor_SetSpeed(id, (int16_t)(target * CAR_OPENLOOP_SCALE));	// 开环直驱, 不再经过PID
}

/*************************************************************************************************
*	函 数 名:	Car_Move
*
*	入口参数:	dir   - 运动方向
*				speed - 目标速度(脉冲/20ms), 取绝对值
*
*	函数功能:	整车运动控制, 原地转向时左右轮反向
*
*************************************************************************************************/
void Car_Move(CarDir_TypeDef dir, int16_t speed)
{
	int16_t left = 0, right = 0;

	if (speed < 0) speed = -speed;

	switch (dir)
	{
		case CAR_DIR_FORWARD:  left =  speed; right =  speed; break;
		case CAR_DIR_BACKWARD: left = -speed; right = -speed; break;
		case CAR_DIR_LEFT:     left = -speed; right =  speed; break;	// 左轮倒转右轮正转 -> 原地左转
		case CAR_DIR_RIGHT:    left =  speed; right = -speed; break;
		case CAR_DIR_STOP:
		default:
			Car_Stop();
			return;
	}

	/* M1 M2 左侧, M3 M4 右侧 */
	Car_SetWheelSpeed(MOTOR_1, left);
	Car_SetWheelSpeed(MOTOR_2, left);
	Car_SetWheelSpeed(MOTOR_3, right);
	Car_SetWheelSpeed(MOTOR_4, right);
}

/*************************************************************************************************
*	函 数 名:	Car_Turn
*
*	入口参数:	dir   - CAR_DIR_LEFT / CAR_DIR_RIGHT
*				speed - 静止时原地转向的目标速度
*
*	函数功能:	转弯控制。行进中(左右轮同向且在动)做差速转弯: 外侧轮保持当前速度,
*				内侧轮降到外侧的 CAR_TURN_INNER_PCT%, 实现边行进边转弯, 不再原地自旋;
*				静止或正在原地转时退化为 Car_Move 原地转向。
*				再次同向转弯结果不变(始终以较快一侧为基准), 反向转弯则切到另一侧。
*
*************************************************************************************************/
void Car_Turn(CarDir_TypeDef dir, int16_t speed)
{
	int16_t lt, rt, al, ar, base, inner, outer, sign;

	lt = s_wheel[MOTOR_1].target;	// 左侧当前目标速度
	rt = s_wheel[MOTOR_3].target;	// 右侧当前目标速度

	/* 静止 / 正在原地转(左右异号) / 仅单侧在动: 直接原地转向 */
	if (!s_running || lt == 0 || rt == 0 || ((lt < 0) != (rt < 0)))
	{
		Car_Move(dir, speed);
		return;
	}

	/* 以较快一侧为外侧基准, 保持整体前进/后退方向不变 */
	al   = (lt >= 0) ? lt : -lt;
	ar   = (rt >= 0) ? rt : -rt;
	base = (al > ar) ? al : ar;
	sign = (lt > 0) ? 1 : -1;

	outer = (int16_t)(base * sign);
	inner = (int16_t)(base * CAR_TURN_INNER_PCT / 100 * sign);

	if (dir == CAR_DIR_LEFT)	// 左转: 左侧为内侧
	{
		Car_SetWheelSpeed(MOTOR_1, inner);
		Car_SetWheelSpeed(MOTOR_2, inner);
		Car_SetWheelSpeed(MOTOR_3, outer);
		Car_SetWheelSpeed(MOTOR_4, outer);
	}
	else						// 右转: 右侧为内侧
	{
		Car_SetWheelSpeed(MOTOR_1, outer);
		Car_SetWheelSpeed(MOTOR_2, outer);
		Car_SetWheelSpeed(MOTOR_3, inner);
		Car_SetWheelSpeed(MOTOR_4, inner);
	}
}

/*************************************************************************************************
*	函 数 名:	Car_Stop
*
*	函数功能:	制动全部电机
*
*************************************************************************************************/
void Car_Stop(void)
{
	uint8_t i;

	for (i = 0; i < MOTOR_NUM; i++)
	{
		s_wheel[i].target   = 0;
		/* PID状态清零已注释(开环直驱)
		s_wheel[i].integral = 0;
		s_wheel[i].lastErr  = 0;
		*/
	}
	s_running = 0;
	Motor_StopAll();
}

/*************************************************************************************************
*	函 数 名:	Car_PID_Update
*
*	函数功能:	位置式速度PID, 需固定每 20ms 调用一次
*
*	说    明:	已整体注释 —— 蓝牙遥控时PID闭环导致轮胎顿挫明显, 底盘改为开环PWM直驱.
*				如需恢复闭环, 取消本函数、Car_Init/Car_Stop 中被注释的PID状态行、
*				main.c 中20ms调用以及 car.h 中PID参数与函数声明的注释即可.
*
*************************************************************************************************/
//void Car_PID_Update(void)
//{
//	uint8_t i;
//	float err, out;
//
//	for (i = 0; i < MOTOR_NUM; i++)
//	{
//		s_wheel[i].speed = Encoder_ReadDelta(i);	// 读取本周期脉冲增量
//
//		if (!s_running) continue;
//
//		err = (float)s_wheel[i].target - (float)s_wheel[i].speed;
//
//		// 积分项 + 限幅
//		s_wheel[i].integral += err * s_wheel[i].ki;
//		if (s_wheel[i].integral >  CAR_PID_IMAX) s_wheel[i].integral =  CAR_PID_IMAX;
//		if (s_wheel[i].integral < -CAR_PID_IMAX) s_wheel[i].integral = -CAR_PID_IMAX;
//
//		out = err * s_wheel[i].kp
//			+ s_wheel[i].integral
//			+ (err - s_wheel[i].lastErr) * s_wheel[i].kd;
//		s_wheel[i].lastErr = err;
//
//		if (out >  CAR_PID_OUTMAX) out =  CAR_PID_OUTMAX;
//		if (out < -CAR_PID_OUTMAX) out = -CAR_PID_OUTMAX;
//
//		// 目标为0时直接制动, 避免积分爬行
//		if (s_wheel[i].target == 0)
//		{
//			s_wheel[i].integral = 0;
//			Motor_Stop(i);
//		}
//		else
//		{
//			Motor_SetSpeed(i, (int16_t)out);
//		}
//	}
//}

/*************************************************************************************************
*	函 数 名:	Car_GetSpeed / Car_GetTarget / Car_IsRunning
*
*	函数功能:	状态查询
*
*************************************************************************************************/
int16_t Car_GetSpeed(uint8_t id)
{
	if (id >= MOTOR_NUM) return 0;
	return s_wheel[id].speed;
}

int16_t Car_GetTarget(uint8_t id)
{
	if (id >= MOTOR_NUM) return 0;
	return s_wheel[id].target;
}

uint8_t Car_IsRunning(void)
{
	return s_running;
}

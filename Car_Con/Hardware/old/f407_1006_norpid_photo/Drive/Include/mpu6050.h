#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"

/*---------------------- MPU6050配置宏 ------------------------*/
/*
 * 使用硬件I2C1, 引脚 PB8=SCL, PB9=SDA (复用AF4)
 * AD0接地时器件地址为 0x68
 */
#define MPU6050_I2C			I2C1
#define MPU6050_I2C_CLK		RCC_APB1Periph_I2C1
#define MPU6050_I2C_AF		GPIO_AF_I2C1
#define MPU6050_I2C_SPEED	400000		// 400kHz快速模式

#define MPU6050_SCL_PORT	GPIOB
#define MPU6050_SCL_PIN		GPIO_Pin_8
#define MPU6050_SCL_PINSRC	GPIO_PinSource8
#define MPU6050_SDA_PORT	GPIOB
#define MPU6050_SDA_PIN		GPIO_Pin_9
#define MPU6050_SDA_PINSRC	GPIO_PinSource9
#define MPU6050_GPIO_CLK	RCC_AHB1Periph_GPIOB

#define MPU6050_ADDR		0x68	// 器件地址(7位)

/* 寄存器 */
#define MPU_REG_SMPLRT_DIV		0x19
#define MPU_REG_CONFIG			0x1A
#define MPU_REG_GYRO_CONFIG		0x1B
#define MPU_REG_ACCEL_CONFIG	0x1C
#define MPU_REG_ACCEL_XOUT_H	0x3B
#define MPU_REG_PWR_MGMT_1		0x6B
#define MPU_REG_WHO_AM_I		0x75

/* 数据换算系数 (加速度±2g, 陀螺仪±250dps) */
#define MPU_ACCEL_SCALE		(9.8f / 16384.0f)	// LSB -> m/s^2
#define MPU_GYRO_SCALE		(1.0f / 131.0f)		// LSB -> 度/秒
#define MPU_TEMP_SCALE		(1.0f / 340.0f)		// 温度 = 36.53 + raw/340

/* 加速度零位矫正值(m/s^2), 静止水平放置时实测得到,
 * 显示矫正后数据时: 矫正值 = 原始值 + 对应errot (见 menu.c MPU页) */
#define AX_errot (-0.37f)
#define AY_errot (0.15f)
#define AZ_errot (1.05f)

/* 数据结构 */
typedef struct
{
	int16_t accX, accY, accZ;	// 加速度原始值
	int16_t gyroX, gyroY, gyroZ;// 陀螺仪原始值
	int16_t tempRaw;			// 温度原始值
	float 	accXms, accYms, accZms;	// 加速度 m/s^2
	float 	gyroXds, gyroYds, gyroZds;// 角速度 度/秒
	float 	temperature;			// 温度 摄氏度
} MPU6050_DataTypeDef;

/*---------------------- 函数声明 ----------------------------*/

uint8_t MPU6050_Init(void);							// 初始化, 返回0成功
uint8_t MPU6050_ReadData(MPU6050_DataTypeDef *data);// 读取全部数据, 返回0成功
uint8_t MPU6050_GetID(void);						// 读取WHO_AM_I

#endif //__MPU6050_H

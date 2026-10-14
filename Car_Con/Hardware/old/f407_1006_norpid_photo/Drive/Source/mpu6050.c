/***
	*************************************************************************************************
	*	@version V2.0
	*	@brief   MPU6050 六轴传感器驱动 (硬件I2C)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 硬件I2C1, SCL=PB8, SDA=PB9, 复用AF4开漏+上拉, 400kHz快速模式
	*  2. 加速度量程 ±2g, 陀螺仪量程 ±250dps, 采样率 1kHz/(1+7)=125Hz
	*  3. MPU6050_Init() 会校验 WHO_AM_I, 返回非0表示通信失败
	*  4. 所有I2C操作均带超时保护, 总线异常时不会死等
	*
	************************************************************************************************
***/

#include "mpu6050.h"
#include "timer.h"

/*---------------- 硬件I2C底层 ----------------*/

#define MPU_I2C_TIMEOUT		100000	// 事件等待超时计数

/* 等待I2C事件, 返回0成功, 1超时 */
static uint8_t MPU_WaitEvent(uint32_t event)
{
	uint32_t timeout = MPU_I2C_TIMEOUT;
	while (I2C_CheckEvent(MPU6050_I2C, event) != SUCCESS)
	{
		if (--timeout == 0) return 1;
	}
	return 0;
}

/* 等待标志清除, 返回0成功, 1超时 */
static uint8_t MPU_WaitFlagClear(uint32_t flag)
{
	uint32_t timeout = MPU_I2C_TIMEOUT;
	while (I2C_GetFlagStatus(MPU6050_I2C, flag) != RESET)
	{
		if (--timeout == 0) return 1;
	}
	return 0;
}

/*---------------- MPU6050寄存器读写 ----------------*/

static uint8_t MPU_WriteReg(uint8_t reg, uint8_t value)
{
	if (MPU_WaitFlagClear(I2C_FLAG_BUSY)) return 1;

	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 1;

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDR << 1, I2C_Direction_Transmitter);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 1;

	I2C_SendData(MPU6050_I2C, reg);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 1;

	I2C_SendData(MPU6050_I2C, value);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 1;

	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

	return 0;
}

static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (MPU_WaitFlagClear(I2C_FLAG_BUSY)) return 1;

	I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 1;

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDR << 1, I2C_Direction_Transmitter);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 1;

	I2C_SendData(MPU6050_I2C, reg);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 1;

	I2C_GenerateSTART(MPU6050_I2C, ENABLE);		// 重复起始
	if (MPU_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 1;

	I2C_Send7bitAddress(MPU6050_I2C, MPU6050_ADDR << 1, I2C_Direction_Receiver);
	if (MPU_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 1;

	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
	while (len--)
	{
		if (len == 0)
		{
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);	// 最后一字节发NACK
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		}
		if (MPU_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 1;
		*buf++ = I2C_ReceiveData(MPU6050_I2C);
	}

	return 0;
}

/*---------------- 对外接口 ----------------*/

/*************************************************************************************************
*	函 数 名:	MPU6050_GetID
*
*	返 回 值:	WHO_AM_I 寄存器值, 正常为 0x68
*
*************************************************************************************************/
uint8_t MPU6050_GetID(void)
{
	uint8_t id = 0;
	MPU_ReadRegs(MPU_REG_WHO_AM_I, &id, 1);
	return id;
}

/*************************************************************************************************
*	函 数 名:	MPU6050_Init
*
*	返 回 值:	0 - 成功, 非0 - 通信失败
*
*	函数功能:	初始化硬件I2C1并配置MPU6050
*
*************************************************************************************************/
uint8_t MPU6050_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	uint8_t err;

	RCC_AHB1PeriphClockCmd(MPU6050_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(MPU6050_I2C_CLK, ENABLE);

	/* SCL/SDA 复用开漏输出+上拉 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
	GPIO_Init(MPU6050_SCL_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(MPU6050_SCL_PORT, MPU6050_SCL_PINSRC, MPU6050_I2C_AF);
	GPIO_PinAFConfig(MPU6050_SDA_PORT, MPU6050_SDA_PINSRC, MPU6050_I2C_AF);

	/* I2C1 配置: 400kHz快速模式, 7位地址 */
	I2C_DeInit(MPU6050_I2C);
	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0x00;
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = MPU6050_I2C_SPEED;
	I2C_Init(MPU6050_I2C, &I2C_InitStructure);

	I2C_Cmd(MPU6050_I2C, ENABLE);

	err  = MPU_WriteReg(MPU_REG_PWR_MGMT_1,    0x80);	// 复位
	Delay_us(100000);									// 等待复位完成
	err |= MPU_WriteReg(MPU_REG_PWR_MGMT_1,    0x00);	// 唤醒, 内部8MHz时钟
	err |= MPU_WriteReg(MPU_REG_SMPLRT_DIV,    0x07);	// 采样率 = 1kHz/(1+7) = 125Hz
	err |= MPU_WriteReg(MPU_REG_CONFIG,        0x06);	// DLPF 5Hz, 滤除车身振动
	err |= MPU_WriteReg(MPU_REG_GYRO_CONFIG,   0x00);	// 陀螺仪 ±250dps
	err |= MPU_WriteReg(MPU_REG_ACCEL_CONFIG,  0x00);	// 加速度 ±2g

	if (err) return 1;
	if (MPU6050_GetID() != MPU6050_ADDR) return 2;

	return 0;
}

/*************************************************************************************************
*	函 数 名:	MPU6050_ReadData
*
*	入口参数:	data - 数据结构体指针
*
*	返 回 值:	0 - 成功
*
*	函数功能:	一次突发读取14字节: 加速度+温度+陀螺仪, 并完成单位换算
*
*************************************************************************************************/
uint8_t MPU6050_ReadData(MPU6050_DataTypeDef *data)
{
	uint8_t buf[14];
	uint8_t err;

	err = MPU_ReadRegs(MPU_REG_ACCEL_XOUT_H, buf, 14);
	if (err) return err;

	data->accX  = (int16_t)((buf[0]  << 8) | buf[1]);
	data->accY  = (int16_t)((buf[2]  << 8) | buf[3]);
	data->accZ  = (int16_t)((buf[4]  << 8) | buf[5]);
	data->tempRaw = (int16_t)((buf[6] << 8) | buf[7]);
	data->gyroX = (int16_t)((buf[8]  << 8) | buf[9]);
	data->gyroY = (int16_t)((buf[10] << 8) | buf[11]);
	data->gyroZ = (int16_t)((buf[12] << 8) | buf[13]);

	data->accXms = data->accX * MPU_ACCEL_SCALE;
	data->accYms = data->accY * MPU_ACCEL_SCALE;
	data->accZms = data->accZ * MPU_ACCEL_SCALE;

	data->gyroXds = data->gyroX * MPU_GYRO_SCALE;
	data->gyroYds = data->gyroY * MPU_GYRO_SCALE;
	data->gyroZds = data->gyroZ * MPU_GYRO_SCALE;

	data->temperature = 36.53f + data->tempRaw * MPU_TEMP_SCALE;

	return 0;
}

/***
	*************************************************************************************************
	*	@file  	location.c
	*	@brief  融合定位: GPS + MPU6050惯导推算(Dead Reckoning)
   *************************************************************************************************
   *  @description
	*
	*  逻辑(见location.h注释):
	*    GPS有效   -> 直接输出GPS位置/速度/航向, 同时用GPS航向缓慢校正惯导航向
	*    GPS失效   -> 在最后一次GPS位置(或初始位置)上, 用陀螺积分航向 +
	*                 前向加速度积分速度做位置推算
	*
	*  精度相关处理:
	*    1. 上电静止调零: 陀螺/加速度零偏必须静止采样, 并检查标准差,
	*       检测到晃动自动重试, 避免带着错误零偏积分造成航向持续漂移
	*    2. 位置用 E7 定点(×1e7度)保存, 不受 float 在113°E只有约0.8m分辨率的限制
	*    3. GPS有效且车速足够时, 用GPS航向对陀螺积分航向做互补修正,
	*       兼顾"陀螺短时平滑"和"GPS长期无漂移"
	*
	*  注意: 纯MPU6050惯导只能短时间(几秒~十几秒)维持精度, 加速度积分漂移快,
	*        长时间GPS失效时不要依赖位置绝对值。
	*
	*************************************************************************************************
***/

#include "location.h"
#include <math.h>
#include "delay.h"
#include "timer.h"

/*---------------------- 常量 ----------------------------*/
#define LOC_DEG2RAD			0.01745329252f
#define LOC_RAD2DEG			57.2957795131f
#define LOC_M_PER_DEG_LAT	111320.0f	// 每度纬度对应距离(m), 校园范围近似足够
#define LOC_E7				1.0e7f		// 度 -> E7定点

#define LOC_SPEED_MAX_MS	10.0f		// 推算速度限幅(m/s), 防积分发散
#define LOC_ZUPT_SPEED		0.30f		// 低于该速度且静止时才清零(m/s)

/*---------------------- 静态状态 ----------------------------*/
static Location_DataTypeDef s_loc;		// 融合定位输出

static float    s_heading   = 0;		// 惯导航向(度)
static float    s_speedMs   = 0;		// 惯导速度(m/s)
static float    s_gyroBias  = 0;		// 陀螺Z零偏(度/秒)
static float    s_accBias   = 0;		// 前向加速度零偏(m/s^2)
static float    s_gyroNoise = 0;		// 校准时测得的角速度标准差(度/秒)
static uint8_t  s_calibrated = 0;		// 1=已完成过一次有效静止校准
static uint32_t s_lastMs    = 0;		// 上次推算时间戳
static uint8_t  s_firstRun  = 1;

/*---------------------- 内部函数 ----------------------------*/

/* 取车头方向的加速度分量(轴向见location.h配置) */
static float Loc_AccForward(const MPU6050_DataTypeDef *mpu)
{
#if LOC_ACC_AXIS == 0
	return mpu->accXms * LOC_ACC_SIGN;
#elif LOC_ACC_AXIS == 1
	return mpu->accYms * LOC_ACC_SIGN;
#else
	return mpu->accZms * LOC_ACC_SIGN;
#endif
}

/* 角度归一化到 [0, 360) */
static float Loc_Norm360(float deg)
{
	while (deg >= 360.0f) deg -= 360.0f;
	while (deg <   0.0f)  deg += 360.0f;
	return deg;
}

/* 角度差归一化到 (-180, 180] */
static float Loc_Norm180(float deg)
{
	deg = Loc_Norm360(deg);
	if (deg > 180.0f) deg -= 360.0f;
	return deg;
}

/* 由float经纬度回写E7定点(惯导推算分支用) */
static int32_t Loc_ToE7(float deg)
{
	return (int32_t)(deg * LOC_E7 + (deg >= 0.0f ? 0.5f : -0.5f));
}

/* 标准差, 并对浮点误差导致的极小负数做保护 */
static float Loc_Std(float sum, float sqSum, uint16_t n)
{
	float var = sqSum / (float)n - (sum / (float)n) * (sum / (float)n);
	if (var < 0.0f) var = 0.0f;
	return sqrtf(var);
}

/*---------------------- 对外接口 ----------------------------*/

/*************************************************************************************************
*	函 数 名:	Location_CalibrateGyro
*
*	返 回 值:	1 - 校准成功;  0 - MPU不在线, 或连续检测到晃动(保留上次零偏)
*
*	函数功能:	静止零偏校准: 采样求陀螺Z与车头向加速度的平均值作为零偏,
*				同时求标准差判断"采样期间车是否真的静止"
*
*	说    明:	【调用时必须保证小车静止不动】否则零偏是错的, 航向会一直漂。
*				每次采样约 0.6s, 最多重试 LOC_CAL_RETRY 次。
*				Location_Init() 会自动调用一次, 之后也可以随时重新调用。
*
*************************************************************************************************/
uint8_t Location_CalibrateGyro(void)
{
	MPU6050_DataTypeDef mpu;
	uint16_t i, n;
	uint8_t  attempt;
	float    g, a, gSum, aSum, gSqSum, aSqSum;
	float    gMean, aMean, gStd, aStd;

	for (attempt = 0; attempt < LOC_CAL_RETRY; attempt++)
	{
		gSum = aSum = gSqSum = aSqSum = 0.0f;
		n = 0;

		for (i = 0; i < LOC_CAL_SAMPLES; i++)
		{
			if (MPU6050_ReadData(&mpu) == 0)
			{
				g = mpu.gyroZds;
				a = Loc_AccForward(&mpu);
				gSum   += g;
				aSum   += a;
				gSqSum += g * g;
				aSqSum += a * a;
				n++;
			}
			Delay_ms(5);	// 5ms x 120 ≈ 0.6s, 等DLPF(5Hz)完全稳定下来
		}

		if (n < 10) return 0;	// MPU6050不在线, 直接失败(不影响GPS模式)

		gMean = gSum / (float)n;
		aMean = aSum / (float)n;
		gStd  = Loc_Std(gSum, gSqSum, n);
		aStd  = Loc_Std(aSum, aSqSum, n);

		if (gStd <= LOC_CAL_GYRO_STD_MAX && aStd <= LOC_CAL_ACC_STD_MAX)
		{
			/* 采样期间足够安静, 采用这组零偏 */
			s_gyroBias   = gMean;
			s_accBias    = aMean;
			s_gyroNoise  = gStd;
			s_calibrated = 1;

			return 1;
		}
	}

	return 0;
}

/*************************************************************************************************
*	函 数 名:	Location_Init
*
*	函数功能:	初始化融合定位: 位置置为初始点, 静止采样校准陀螺/加速度零偏
*
*	说    明:	调用前需完成 MPU6050_Init 和 GPS_Init;
*				校准期间(约0.6s)请保持小车静止
*
*************************************************************************************************/
void Location_Init(void)
{
	Location_ResetToDefault();

	/* 上电静止调零(MPU不在线时内部会直接返回, 不影响GPS模式) */
	Location_CalibrateGyro();

	s_heading  = 0;
	s_speedMs  = 0;
	s_firstRun = 1;
}

/*************************************************************************************************
*	函 数 名:	Location_IsCalibrated / Location_GetGyroBias / Location_GetGyroNoise
*
*	函数功能:	静止校准结果查询(供菜单显示)
*
*************************************************************************************************/
uint8_t Location_IsCalibrated(void)
{
	return s_calibrated;
}

float Location_GetGyroBias(void)
{
	return s_gyroBias;
}

float Location_GetGyroNoise(void)
{
	return s_gyroNoise;
}

/*************************************************************************************************
*	函 数 名:	Location_Update
*
*	函数功能:	融合定位主任务: 解析GPS语句, GPS有效用GPS, 失效用惯导推算
*
*	入口参数:	now - 系统毫秒戳(Get_Ms())
*
*	说    明:	建议50~100ms周期调用; 内部自行读取MPU6050
*
*************************************************************************************************/
void Location_Update(uint32_t now)
{
	GPS_DataTypeDef *gps;
	MPU6050_DataTypeDef mpu;
	float dt;
	uint8_t mpuOK;

	/* 1. 解析新到的NMEA语句(队列方式, 一次突发不会丢语句) */
	GPS_Process();
	gps = GPS_GetData();

	/* 2. GPS有效帧: 直接采用, 并记录时间戳
	 *    GGA 和 RMC 都带经纬度, 谁先解析出来就用谁, 保证位置刷新率 */
	if (gps->valid && gps->updated)
	{
		gps->updated    = 0;	// 读后清零(gps.h约定)
		gps->fixUpdated = 0;

		s_loc.latitude   = gps->latitude;
		s_loc.longitude  = gps->longitude;
		s_loc.latE7      = gps->latE7;
		s_loc.lonE7      = gps->lonE7;
		s_loc.speedKmh   = gps->speedKmh;
		s_loc.course     = gps->course;
		s_loc.satellites = gps->satellites;
		s_loc.source     = LOC_SRC_GPS;
		s_loc.lastGpsMs  = now;
		s_loc.fixSeq++;		// 通知轨迹记录: 有新的定位点了

		s_speedMs = gps->speedKmh / 3.6f;
		if (s_speedMs > 0.5f) s_heading = gps->course;	// GPS航向运动时才可信
	}

	/* 3. 惯导推算 */
	mpuOK = (MPU6050_ReadData(&mpu) == 0) ? 1 : 0;

	if (s_firstRun)		// 首次调用只建立时间基准
	{
		s_firstRun = 0;
		s_lastMs   = now;
		return;
	}

	dt = (float)(now - s_lastMs) / 1000.0f;
	s_lastMs = now;
	if (dt <= 0.0f || !mpuOK) return;

	/* 航向: 陀螺Z轴积分(去除静止零偏) */
	s_heading = Loc_Norm360(s_heading + (mpu.gyroZds - s_gyroBias) * LOC_GYRO_SIGN * dt);

	/* GPS有效且车速够 -> 用GPS航向对陀螺积分航向做互补修正:
	 * 陀螺短时平滑但会漂, GPS绝对但抖; 每秒只修正一部分, 长期不漂且不跳变 */
	if (Location_IsGpsLive() && s_loc.speedKmh > LOC_GPS_HDG_MIN_KMH)
	{
		float err = Loc_Norm180(s_loc.course - s_heading);

		/* 限幅: 单次最多纠 20°, 防止GPS航向野值时把航向拽飞 */
		if (err >  20.0f) err =  20.0f;
		if (err < -20.0f) err = -20.0f;

		s_heading = Loc_Norm360(s_heading + err * LOC_GPS_HDG_GAIN * dt);
	}

	s_loc.heading = s_heading;

	/* GPS失效 -> 位置推算 */
	if (!Location_IsGpsLive())
	{
		float accF = Loc_AccForward(&mpu) - s_accBias;
		float dN, dE, mPerDegLon;

		/* 速度: 前向加速度积分; 低速且静止时清零(ZUPT)防漂移 */
		if (s_speedMs < LOC_ZUPT_SPEED &&
			fabsf(accF) < LOC_ZUPT_ACC &&
			fabsf(mpu.gyroZds - s_gyroBias) < LOC_ZUPT_GYRO)
		{
			s_speedMs = 0;
		}
		else
		{
			s_speedMs += accF * dt;
			if (s_speedMs >  LOC_SPEED_MAX_MS) s_speedMs =  LOC_SPEED_MAX_MS;
			if (s_speedMs < -LOC_SPEED_MAX_MS) s_speedMs = -LOC_SPEED_MAX_MS;
		}

		/* 位置: 速度沿航向分解后累加(校园范围内用平面近似) */
		dN = s_speedMs * dt * cosf(s_heading * LOC_DEG2RAD);
		dE = s_speedMs * dt * sinf(s_heading * LOC_DEG2RAD);

		mPerDegLon = LOC_M_PER_DEG_LAT * cosf(s_loc.latitude * LOC_DEG2RAD);
		s_loc.latitude += dN / LOC_M_PER_DEG_LAT;
		if (fabsf(mPerDegLon) > 1.0f) s_loc.longitude += dE / mPerDegLon;

		/* 同步E7定点(推算精度本来就有限, 这里只是保持字段一致) */
		s_loc.latE7 = Loc_ToE7(s_loc.latitude);
		s_loc.lonE7 = Loc_ToE7(s_loc.longitude);

		s_loc.speedKmh = s_speedMs * 3.6f;
		s_loc.course   = s_heading;
		s_loc.source   = (s_loc.lastGpsMs == 0) ? LOC_SRC_INIT : LOC_SRC_DR;
	}
}

/*************************************************************************************************
*	函 数 名:	Location_GetData
*
*	函数功能:	获取融合定位数据指针
*
*************************************************************************************************/
Location_DataTypeDef *Location_GetData(void)
{
	return &s_loc;
}

/*************************************************************************************************
*	函 数 名:	Location_GetSourceName
*
*	函数功能:	定位来源名(定宽4字符, 可直接用于LCD/串口打印)
*
*************************************************************************************************/
const char *Location_GetSourceName(void)
{
	static const char *s_name[3] = { "INIT", "GPS ", "DR  " };
	return s_name[s_loc.source];
}

/*************************************************************************************************
*	函 数 名:	Location_IsGpsLive
*
*	返 回 值:	1 - GPS在线(有效且未超时), 0 - GPS失效
*
*************************************************************************************************/
uint8_t Location_IsGpsLive(void)
{
	if (s_loc.lastGpsMs == 0) return 0;
	return ((uint32_t)(Get_Ms() - s_loc.lastGpsMs) <= LOC_GPS_TIMEOUT_MS) ? 1 : 0;
}

/*************************************************************************************************
*	函 数 名:	Location_ResetToDefault
*
*	函数功能:	把当前位置重置为初始点(湘南学院), 推算速度清零
*
*************************************************************************************************/
void Location_ResetToDefault(void)
{
	s_loc.latitude   = LOC_INIT_LATITUDE;
	s_loc.longitude  = LOC_INIT_LONGITUDE;
	s_loc.latE7      = LOC_INIT_LAT_E7;
	s_loc.lonE7      = LOC_INIT_LON_E7;
	s_loc.speedKmh   = 0;
	s_loc.course     = 0;
	s_loc.heading    = 0;
	s_loc.satellites = 0;
	s_loc.source     = LOC_SRC_INIT;
	s_loc.lastGpsMs  = 0;
	s_loc.fixSeq     = 0;
	s_speedMs        = 0;
}

/*************************************************************************************************
*	函 数 名:	Location_DistanceTo
*
*	返 回 值:	当前位置到目标点的距离(m), 平面近似, 校园/赛道范围足够
*
*************************************************************************************************/
float Location_DistanceTo(float lat, float lon)
{
	float dN = (lat - s_loc.latitude)  * LOC_M_PER_DEG_LAT;
	float dE = (lon - s_loc.longitude) * LOC_M_PER_DEG_LAT *
			   cosf(s_loc.latitude * LOC_DEG2RAD);
	return sqrtf(dN * dN + dE * dE);
}

/*************************************************************************************************
*	函 数 名:	Location_BearingTo
*
*	返 回 值:	当前位置到目标点的方位角(度, 正北0°顺时针)
*
*	说    明:	与 Location_GetData()->heading 相减即得转向角, 可做航点巡航
*
*************************************************************************************************/
float Location_BearingTo(float lat, float lon)
{
	float dN = (lat - s_loc.latitude)  * LOC_M_PER_DEG_LAT;
	float dE = (lon - s_loc.longitude) * LOC_M_PER_DEG_LAT *
			   cosf(s_loc.latitude * LOC_DEG2RAD);
	return Loc_Norm360(atan2f(dE, dN) * LOC_RAD2DEG);
}

/*************************************************************************************************
*	函 数 名:	Location_E7ToLocalM
*
*	函数功能:	两个E7定点经纬度之差 -> 局部平面米(北/东)
*
*	说    明:	先用int32做差再乘系数, 全程不经过float经纬度,
*				因此没有 float 在113°E附近约0.8m的量化误差。
*				参考纬度取两点平均, 校园/赛道范围内误差可忽略。
*
*************************************************************************************************/
void Location_E7ToLocalM(int32_t refLatE7, int32_t refLonE7,
                         int32_t latE7, int32_t lonE7,
                         float *northM, float *eastM)
{
	/* 1e-7 度 -> 米: 纬度 111320 m/度, 经度 111320*cos(lat) m/度 */
	float latDeg = (float)(((int32_t)((refLatE7 + latE7) / 2)) ) * 1.0e-7f;
	float mPerDegLon = LOC_M_PER_DEG_LAT * cosf(latDeg * LOC_DEG2RAD);

	if (northM) *northM = (float)(latE7 - refLatE7) * 1.0e-7f * LOC_M_PER_DEG_LAT;
	if (eastM)  *eastM  = (float)(lonE7 - refLonE7) * 1.0e-7f * mPerDegLon;
}

/*************************************************************************************************
*	函 数 名:	Location_E7DistanceM
*
*	返 回 值:	两个E7定点经纬度之间的平面距离(m)
*
*************************************************************************************************/
float Location_E7DistanceM(int32_t lat1E7, int32_t lon1E7,
                           int32_t lat2E7, int32_t lon2E7)
{
	float dN, dE;

	Location_E7ToLocalM(lat1E7, lon1E7, lat2E7, lon2E7, &dN, &dE);
	return sqrtf(dN * dN + dE * dE);
}

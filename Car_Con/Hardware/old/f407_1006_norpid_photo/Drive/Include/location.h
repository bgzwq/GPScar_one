#ifndef _LOCATION_H_
#define _LOCATION_H_

#include "stm32f4xx.h"
#include "gps.h"
#include "mpu6050.h"

/*---------------------- 融合定位说明 ----------------------------*/
/*
 * 输出结构体在 GPS_DataTypeDef(经纬度/速度/方向等)的基础上,
 * 增加 source 字段用来判断当前位置是 GPS模块定位 还是 惯导辅助推算(DR):
 *
 *   1. GPS信号持续有效   -> 直接输出GPS数据              (LOC_SRC_GPS)
 *   2. GPS信号曾有效, 后中断 -> 最后一次GPS位置 + MPU6050惯导推算 (LOC_SRC_DR)
 *   3. 从未获得过有效定位 -> 初始位置 + MPU6050惯导推算   (LOC_SRC_INIT)
 *
 * 定位初始信息: 湘南学院 北纬25°47′30″ 东经113°6′14″
 * (上电时若无GPS信号, 以该点为起点进行惯导推算)
 *
 * 精度说明:
 *   位置同时以 float 经纬度(显示用)和 int32 定点 latE7/lonE7(×1e7度) 保存。
 *   113°E 附近 float 的最小分辨率约 2^-17 度 ≈ 0.8m, 直接拿 float 相减画轨迹
 *   会出现明显台阶; E7定点只有约1.1cm分辨率, 轨迹记录与绘图都用 E7。
 */
#define LOC_INIT_LATITUDE		25.791667f		// 25°47′30″N
#define LOC_INIT_LONGITUDE		113.103889f		// 113°6′14″E

#define LOC_INIT_LAT_E7			257916670		// 25.791667 × 1e7
#define LOC_INIT_LON_E7			1131038890		// 113.103889 × 1e7

/* GPS在线判超时: 超过该时间没有有效RMC语句则认为GPS失效, 切换到惯导 */
#define LOC_GPS_TIMEOUT_MS		2500

/*---------------------- 惯导安装方向配置(按实际安装修改) --------*/
/*
 * MPU6050各轴向与车身方向不一定对齐, 在此选择:
 *   LOC_ACC_AXIS   车头正方向对应的加速度轴: 0=X 1=Y 2=Z
 *   LOC_ACC_SIGN   该轴正向是否与车头一致: 一致=1 相反=-1
 *   LOC_GYRO_SIGN  陀螺Z轴(偏航)方向: 左转为正=1, 反=-1
 */
#define LOC_ACC_AXIS			0
#define LOC_ACC_SIGN			1
#define LOC_GYRO_SIGN			1

/* 零速更新(ZUPT)阈值: 静止时把推算速度清零, 抑制加速度积分漂移 */
#define LOC_ZUPT_ACC			0.30f	// 加速度 m/s^2
#define LOC_ZUPT_GYRO			2.0f	// 角速度 度/秒

/*---------------------- 静止调零(上电校准)参数 ------------------*/
/*
 * MPU6050的陀螺零偏和加速度零偏必须"上电静止时"采样求平均得到,
 * 否则角速度积分出来的航向会一直往一个方向漂。
 * 为了保证校准有效, 这里除了求平均值还求标准差:
 * 如果校准过程中有人碰了车/车在动, 标准差会明显变大, 就自动重来一次。
 */
#define LOC_CAL_SAMPLES			120		// 每次校准采样点数(×5ms ≈ 0.6s)
#define LOC_CAL_RETRY			3		// 检测到晃动时的最大重试次数
#define LOC_CAL_GYRO_STD_MAX	3.0f	// 允许的陀螺角速度标准差(度/秒)
#define LOC_CAL_ACC_STD_MAX		0.35f	// 允许的加速度标准差(m/s^2)

/*---------------------- 陀螺/GPS航向互补参数 --------------------*/
/*
 * 陀螺积分短时平滑但会缓慢漂移, GPS航向绝对但抖动大且低速不可信。
 * GPS有效且车速超过下限时, 每秒把惯导航向朝GPS航向修正一部分。
 */
#define LOC_GPS_HDG_MIN_KMH		3.0f	// 低于该车速不用GPS航向(km/h)
#define LOC_GPS_HDG_GAIN		0.5f	// 互补修正系数(1/秒), 越大跟随GPS越快

/*---------------------- 定位来源 ----------------------------*/
typedef enum
{
	LOC_SRC_INIT = 0,	// 初始位置(从未定位成功)
	LOC_SRC_GPS,		// GPS模块定位
	LOC_SRC_DR,			// 惯导推算(GPS失效后)
} LocationSrc_TypeDef;

/*---------------------- 融合定位数据 ----------------------------*/
typedef struct
{
	float latitude;				// 纬度(度), 北纬为正  (显示用)
	float longitude;			// 经度(度), 东经为正  (显示用)
	int32_t latE7;				// 纬度 × 1e7 定点(约1.1cm), 轨迹/绘图用
	int32_t lonE7;				// 经度 × 1e7 定点
	float speedKmh;				// 地面速度 km/h
	float course;				// 地面航向角(度, 正北0°顺时针)
	float heading;				// 惯导推算航向(度, 低速/无GPS时也可用)
	uint8_t satellites;			// GPS使用卫星数(DR时保持最后一次值)
	LocationSrc_TypeDef source;	// 定位来源
	uint32_t lastGpsMs;			// 最后一次GPS有效的系统毫秒戳, 0=从未定位
	uint32_t fixSeq;			// 定位更新序号, 每次采纳新GPS位置+1(轨迹记录用它判新帧)
} Location_DataTypeDef;

/*---------------------- 函数声明 ----------------------------*/

void Location_Init(void);						// 初始化(需先 MPU6050_Init/GPS_Init, 内部做静止校准)
void Location_Update(uint32_t now);				// 周期调用(建议50~100ms): 解析GPS + 惯导推算
Location_DataTypeDef *Location_GetData(void);	// 获取融合定位数据指针
const char *Location_GetSourceName(void);		// 定位来源名 "INIT"/"GPS "/"DR  "
uint8_t Location_IsGpsLive(void);				// GPS当前是否在线(有效且未超时)
void  Location_ResetToDefault(void);			// 把当前位置手动重置为初始位置

/*---------------------- 静止调零(可重复调用) --------------------*/
/*
 * 重新做一次静止零偏校准, 调用期间(约0.6~1.8s)必须保持小车静止不动。
 * 返回 1=校准成功, 0=MPU不在线或一直检测到晃动(此时保留上次零偏)。
 */
uint8_t Location_CalibrateGyro(void);
uint8_t Location_IsCalibrated(void);			// 是否已完成过一次有效校准
float   Location_GetGyroBias(void);				// 当前陀螺Z零偏(度/秒)
float   Location_GetGyroNoise(void);			// 校准时测得的角速度标准差(度/秒)

/*---------------------- 导航辅助(航点/轨迹绘图用) ---------------*/
float Location_DistanceTo(float lat, float lon);// 当前位置到目标点的距离(m)
float Location_BearingTo(float lat, float lon);	// 当前位置到目标点的方位角(度)

/*
 * 把两个E7定点经纬度之差换算成局部平面米(小范围近似, 校园/赛道足够):
 *   northM - 北向分量(m), 正=目标在北
 *   eastM  - 东向分量(m), 正=目标在东
 * 用int32定点做差再乘系数, 不经过float经纬度, 所以没有0.8m的量化台阶。
 */
void  Location_E7ToLocalM(int32_t refLatE7, int32_t refLonE7,
                          int32_t latE7, int32_t lonE7,
                          float *northM, float *eastM);

/* 两个E7定点经纬度之间的平面距离(m) */
float Location_E7DistanceM(int32_t lat1E7, int32_t lon1E7,
                           int32_t lat2E7, int32_t lon2E7);

#endif /* _LOCATION_H_ */

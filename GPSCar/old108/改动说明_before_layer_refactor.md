# 导航智能小车 改动说明（K230 通信 / 轨迹地图存 SD / 阻塞问题）
//2026年10月7
> 本文档对应本次代码改动。所有源码保持原有编码（工程内大部分是 GB2312，`K230.c` 等少数是 UTF-8）。
> 编译验证：ARMCC 5.06 update 6，**0 Error 0 Warning**；
> 占用：Code 49214 B + RO 13402 B + RW 624 B + ZI 25768 B（RAM 约 25.8 KB / 128 KB，Flash 约 61.8 KB / 1 MB）。

---

## 一、先看这里：必须在硬件上确认的 3 件事

| # | 事项 | 怎么确认 |
|---|------|----------|
| 1 | **上电要静止** | 上电后约 0.6 s（最长 1.8 s）在做 MPU6050 静止调零，这期间别碰车。MPU 页看 `Zero: OK/FAIL`，`FAIL` 说明校准时有晃动 → 重新上电。 |
| 2 | **惯导安装方向** | `location.h` 里 `LOC_ACC_AXIS / LOC_ACC_SIGN / LOC_GYRO_SIGN`。**手推车头左转**，GPS 页 `Head` 应该增大；如果减小就把 `LOC_GYRO_SIGN` 改成 `-1`。 |
| 3 | **SD 卡** | 插卡上电，串口应打印 `[NAV] SD ok, log -> 0:/CARLOG/TRACK.CSV`。地图页信息行 `SD:` 显示 `OK`。没卡也能用，只是不落盘。 |

---

## 二、解决的阻塞 / 冲突问题

### 1）SD 卡一读写就整车死机（最严重）
`FatFs/sdio_sd.c` 在 `SD_Init()` 和块读写时会打开 `SDIO_IT_DATAEND` 等中断源并使能 `SDIO_IRQn`、`DMA2_Stream3_IRQn`，但工程里**从来没有实现这两个中断函数**，启动文件里它们是 `[WEAK]` 的 `Default_Handler`（内容是一条 `B .` 死循环）。结果是第一次读写 SD 卡就跳进死循环，整机卡死。

→ 在 `MY/stm32f4xx_it.c` 里补上：

```c
void SDIO_IRQHandler(void)        { SD_ProcessIRQSrc(); }   /* 置传输完成/错误标志 */
void DMA2_Stream3_IRQHandler(void){ SD_ProcessDMAIRQ();  }
```

### 2）GPS 语句被大量丢弃 → GPS 定位长期不生效
原来 `gps.c` 只有一个 `s_msgReady` 缓冲：一次突发里第一条语句收完后，**后面所有字节都被 `if (s_msgReady) continue;` 丢掉**。定位模块 1 Hz 一次连续吐 GGA/GSA/GSV/RMC/VTG 共 5~8 条，而主循环 100 ms 才消费一次 → 每次突发只解析出 1 条。而 `valid` 只在 RMC 里置 1，一旦 RMC 被丢，`gps->valid` 永远是 0，融合定位永远进不了 GPS 模式。

→ 改成**完整语句环形队列**（10 条，静态），突发全部入队，`GPS_Process()` 每次批量解析最多 4 条。
→ 另外 `GGA` 的定位质量位也作为 `valid` 的兜底（有些模块默认不开 RMC）。
→ 新增诊断：GPS 页 `RMC:`（应每秒 +1）、`Drop:`（应为 0）。

### 3）K230 与蓝牙摇杆抢底盘控制权
K230 原本在 `MODE_MANUAL` 下直接写电机，而摇杆（`lanyacon.c`）和语音在手动模式下也写电机 → 两套控制互相覆盖轮速。而且两者的"掉线保护"都是**无条件** `Car_Stop()`：只要收到过一帧，500 ms 后就会在避障 / 雷达模式下乱踩刹车。

→ 新增独立 **`MODE_VISION`**（KEY3 循环：MANUAL → AVOID → RADAR → VISION），只有该模式下 K230 接管底盘。
→ 两处掉线保护都增加 `s_wasControlling` 判断，只在"确实是自己刚才在驱动底盘"时才刹车。
→ 进入/离开 VISION 模式时自动给 K230 下发 `TRACK_START` / `TRACK_STOP`。

### 4）按键 `while(按下);` 死等
`car_key.c` 原来是 `Delay_ms(10); while(读引脚==0);` —— 按住按键不放，主循环里的 GPS 解析 / K230 组帧 / 遥控 / 避障全部停摆；按键卡住或引脚短路则**永久死循环**。

→ 改为非阻塞扫描：`CarKey_Poll()` 每 50 ms 调一次（消抖 + 长按判定），事件进小队列，`CarKey_GetEvent()` 取走。同时新增**长按**事件。

### 5）蜂鸣器阻塞
`Buzzer_Beep()` 内部是 `Delay_ms()`，切换模式 / 避障时会把主循环卡住 60~100 ms。
→ 新增 `Buzzer_BeepAsync()` + `Buzzer_Task()`（主循环调用，到点自动静音），周期性提示音已全部改用它。`Buzzer_Beep()` 保留给开机那一次。

### 6）超声波把主循环占掉 30%
HC-SR04 无回波时（空旷场地是常态）要等满 30 ms 超时才返回，100 ms 周期下等于 30% 的时间被占住，GPS/K230/遥控都得排队。
→ 车头超声波采样率改为自适应：**避障模式 100 ms**（安全相关），其它模式 300 ms。

### 7）栈太小（隐藏的溢出风险）
启动文件 `Stack_Size` 只有 1 KB。FatFs 开 LFN（`FF_USE_LFN=2`）时 `create_name()` 会在**栈上**放 512 B 名字缓冲，`f_open()` 极易溢出。
→ 用 `armlink --callgraph --info=stack` 实测最深调用链：
`main → Menu_KeyProcess → Menu_MapSave → MapView_SaveToSd → NavLog_WriteBmp1bpp → f_open → … → DMA_DeInit` = **0x6F4（1780）字节**，而且中断还会在同一栈上嵌套。
→ `Stack_Size` 改为 **0x1000（4 KB）**（RAM 富余，RW+ZI 仅约 26 KB）。

### 8）其它
- `LCD_DrawPoint(x,y,color)` 原来**忽略 color 形参**、只画当前画笔色，与函数说明不符 → 已按说明正确转换 RGB565 后写入（原有调用方传的都是 `LCD.Color`，行为不变）。
- `ffconf.h`：`FF_MIN_SS/FF_MAX_SS` 由 512/4096 改为 **512/512**。`diskio.c` 的 `GET_SECTOR_SIZE` 本来就固定返回 512，固定成 512 后 `FATFS.win` 与 `FIL.buf` 各从 4 KB 降到 512 B，**省下约 7 KB RAM**。
- `ffsystem.c` 第 43 行注释漏了 `*/`，导致 "nested comment" 警告 → 已修正（第三方示例文件的笔误）。
- 掉线保护只在真正接管过底盘后才刹车（见 3）。

---

## 三、GPS 定位精度相关

| 措施 | 说明 |
|------|------|
| 不丢语句 | 见二.2，这是之前定位不生效的根因 |
| **定点经纬度** | 新增 `latE7/lonE7`（度 × 1e7，约 1.1 cm）。113°E 附近 `float` 的最小分辨率约 2⁻¹⁷ 度 ≈ **0.8 m**，直接拿 float 画轨迹会有明显台阶。NMEA 解析全程用 `double` 中间量，落库用定点。 |
| 静止调零更可靠 | `Location_CalibrateGyro()` 采样 120 点（约 0.6 s）求均值，**同时算标准差**判断"采样期间车是否真的静止"；超阈值自动重试（最多 3 次）。成功/失败都会打印，MPU 页显示 `Zero/BiasZ/Noise`。 |
| 航向互补滤波 | 原来注释说"用 GPS 航向校正惯导航向"，但代码里没做。现在 GPS 有效且车速 > 3 km/h 时，每秒把陀螺积分航向朝 GPS 航向修正 0.5（单次限幅 20°），兼顾"陀螺短时平滑"和"GPS 长期不漂"。 |
| 轨迹记录做精度过滤 | 无效定位 / 惯导推算点 / 卫星数 < 4 / HDOP > 3.0 / 定位质量为 0 **都不记**；与上一点距离 < 1.0 m 不记（静止时 GPS 抖动不会画成一团）；相邻点隐含速度 > 8 m/s 判为野值丢弃；连续丢 5 次后强制接受，避免整体漂移时永久锁死。 |

> PID 没有动（按你的要求），`LOC_ZUPT_*`、`LOC_GPS_HDG_*`、`NAVLOG_*` 这些阈值都集中在头文件里，需要调时改宏即可。

---

## 四、K230 双向通信协议

物理层 **USART1，PA9(TX)/PA10(RX)，115200 8N1**。

> ⚠️ **USART1 同时是调试串口（printf）**，所以 K230 会收到主控打印的普通文本。
> K230 端请按下面的"帧头 + 长度 + 校验"来找帧，忽略文本即可（调试文本都是 ASCII，不会出现 0xAA）。

### 4.1 K230 → 主控（**保持原样，K230 端不用改**）

固定 8 字节，无校验：

```
0x55 | LR | X高 | X低 | UD | Y高 | Y低 | flag
  LR   = 1 目标在左侧   0 目标在右侧
  UD   = 1 目标在上侧   0 目标在下侧
  X/Y  = 目标与画面中心的距离（像素）
  flag = 1 请求夹取     0 继续前行
```

### 4.2 主控 → K230（**新增，K230 端需要实现**）

变长帧，带校验：

```
0xAA | CMD | LEN | payload[LEN] | CHK
  CHK = 0xAA ^ CMD ^ LEN ^ payload[0] ^ ... ^ payload[LEN-1]
```

| CMD | 名称 | LEN | 含义 |
|-----|------|-----|------|
| 0x01 | `TRACK_START` | 0 | 进入 VISION 模式，请开始输出目标 |
| 0x02 | `TRACK_STOP` | 0 | 离开 VISION 模式，请停止输出目标 |
| 0x03 | `GRAB_ACK` | 0 | 已收到夹取请求，小车已停车 |
| 0x04 | `GRAB_DONE` | 0 | 夹取流程结束，可以继续输出目标 |
| 0x05 | `RECALIB` | 0 | 要求重新标定/重置目标（协议保留，暂未主动下发） |
| 0x06 | `HEARTBEAT` | 0 | 主控心跳，**每 1 s 一次**（K230 可据此判断主控在线） |
| 0x07 | `SET_SPEED` | 1 | payload[0] = 基础速度 0~100（协议保留，暂未主动下发） |

### 4.3 夹取握手流程

```
K230 发 flag=1
   → 主控 Car_Stop()，回 GRAB_ACK，蜂鸣提示，进入"夹取挂起"
   → 机械臂动作（目前未接入）
   → 3 s 超时后主控自动 K230_GrabDone()：回 GRAB_DONE，恢复流程
```

> 机械臂接入后，在动作完成处调用 `K230_GrabDone()` 即可**提前结束**挂起，不必等超时。

### 4.4 链路状态

地图页信息行显示 `K2: ON / LOST / WAIT`。
菜单外的完整统计在 `K230_GetStatus()`：`rxFrames / rxBadFrames / txFrames / overflow / lastFrameMs`。

---

## 五、SD 卡文件

新建目录 `0:/CARLOG/`。

### `TRACK.CSV`（追加，每次上电续写）

```
idx,lat,lon,speed_kmh,course_deg,sats,hdop,fix,source,utc,t_ms
0,25.7916670,113.1038890,3.2,145.6,9,0.9,1,1,2025-01-06T08:31:22,15240
```

- `idx`：**本次上电内**的行号（文件跨上电续写，所以会重新从 0 开始）
- `lat/lon`：定点经纬度还原的十进制度，**7 位小数**（不需要 float 精度）
- `source`：0=初始位置 1=GPS 2=惯导推算
- `utc`：GPS 时间（板子没有 RTC，用 GPS 时间让跨上电的日志也能对上）
- 每写 10 个点 `f_sync()` 一次，兼顾掉电丢失量与写卡寿命

### `MAP.BMP`（每次保存覆盖）

236×192 24 位 BMP（正好是 LCD 地图区域大小），**北朝上**，含网格、轨迹折线、起点（方块）、当前位置（圆点）、指北箭头、比例尺。
双击即可在电脑上看；`TRACK.CSV` 是同一份数据，可以导进 Excel / 地图工具。
文件约 136 KB，保存瞬间界面停顿一两百毫秒（KEY4 主动触发，可接受）。

---

## 六、菜单与按键

分页变成 **7 页**：OVER / MOTR / SENS / MPU / GPS / IO / **MAP**。

| 按键 | 通用 | 地图页 |
|------|------|--------|
| KEY1 短按 | 上一页 | 上一页 |
| KEY2 短按 | 下一页 | 下一页 |
| KEY3 短按 | 切换工作模式（4 模式循环） | 同左 |
| KEY4 短按 | 电机自检 | **保存地图到 SD 卡**（BMP + CSV 刷盘） |
| KEY1 长按 | — | 视野缩小 |
| KEY2 长按 | — | 视野放大 |
| KEY4 长按 | — | **清空当前轨迹并恢复自动缩放**（不删除已写入 CSV 的历史数据） |

地图页信息行：`Pts:轨迹点数 / 累计里程 m / SD:状态 / K2:K230链路`。
地图**只在数据变化时重绘**，300 ms 刷新不会闪屏；离开页面时会通知地图模块，回来必定重绘。

GPS 页新增精度诊断行：`RMC:`（每秒 +1 才说明定位真的生效）、`Drop:`（队列丢弃数，应为 0）、`Tk:`/`SD:`。
MPU 页新增：`Zero: OK/FAIL`、`BiasZ`、`Noise`。

---

## 七、新增 / 修改文件清单

**新增**

| 文件 | 作用 |
|------|------|
| `Drive/Include/navlog.h`、`Drive/Source/navlog.c` | SD 挂载、轨迹记录与精度过滤、TRACK.CSV、BMP 写出 |
| `Drive/Include/mapview.h`、`Drive/Source/mapview.c` | 轨迹投影、LCD 地图绘制、1bpp 位图渲染（236×192，静态 5760 B） |

**修改**

| 文件 | 改动 |
|------|------|
| `MY/main.c` | 初始化顺序、任务接线、模式切换通知 K230、超声波自适应采样 |
| `MY/stm32f4xx_it.c` | **补上 SDIO / DMA2_Stream3 中断服务函数** |
| `Drive/Source/gps.c`、`gps.h` | NMEA 语句队列、double 解析 + E7 定点、GGA valid 兜底、诊断计数 |
| `Drive/Source/location.c`、`location.h` | 静止调零（含标准差重试）、航向互补滤波、E7、局部平面米换算 |
| `Drive/Source/car_key.c`、`car_key.h` | 非阻塞扫描 + 长按事件 |
| `Drive/Source/buzzer.c`、`buzzer.h` | 非阻塞鸣叫 |
| `Drive/Source/K230.c`、`K230.h` | 双向协议、VISION 模式接管、夹取握手、心跳、链路统计 |
| `Drive/Source/menu.c`、`menu.h` | MODE_VISION、地图页、按键事件处理、精度诊断显示 |
| `Drive/Source/lanyacon.c` | 掉线保护按模式收敛 |
| `Drive/Source/lcd_spi_154.c` | `LCD_DrawPoint` 使用传入颜色 |
| `FatFs/ffconf.h` | 扇区尺寸固定 512（省约 7 KB RAM） |
| `FatFs/ffsystem.c` | 修正注释笔误 |
| `STM32/.../startup_stm32f40_41xxx.s` | 栈 1 KB → 4 KB |
| `Project/STM32F407.uvprojx`、`Project/.eide/eide.yml` | 加入 4 个新文件 |

---

## 八、已知未处理 / 后续可做

- **PID 未动**（按你的要求）。`car.h` 里 PID 参数与 `Car_PID_Update()` 仍是注释状态。
- **雷达模式仍有阻塞**：`App_RadarTask()` 每步 `Delay_ms(15)` 等舵机稳定，单次约 15~45 ms。这是舵机扫描演示的固有节奏，只在该模式下发生，未改。
- **超声波仍是轮询式**：想彻底不阻塞需要改成输入捕获 / EXTI 时间戳，改动较大，本次只做了采样率自适应。
- **惯导长时间推算不可靠**：纯 MPU6050 只能维持几秒~十几秒，`location.c` 顶部注释已说明；要更准建议用编码器里程替代加速度积分（代码里已留提示）。
- **夹取机械臂未接入**：`K230_GrabDone()` 是预留的接入点。
- **地图只有 1024 个点**（约 1 km @1 m 间距）。满了以后 LCD 不再增长（显示 `SD:FULL`），但 CSV 仍继续追加。需要更长轨迹就调 `NAVLOG_MAX_POINTS`（每点 8 字节）。

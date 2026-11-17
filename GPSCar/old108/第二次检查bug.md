# 第二次检查 · 工程逻辑 / RTOS / 定时器 / 冲突 + M1·M2 编码器专项

> 工程路径：`D:\table2\GPSCar_PRO\GPSCar\old108`
> 目标芯片：STM32F407ZGTx（LQFP144，HSE 12 MHz → SYSCLK 168 MHz）
> 构建系统：Keil `Project/STM32F407.uvprojx`（ARMCC 5.06，MicroLIB）
> 审查方式：全量读源码（`MY/BSP/Drive/Program/Mid` + `RTOS` + `FatFs`）+ 工程文件（uvprojx / eide.yml）+ 启动文件
> 文中所有结论都带 `文件:行号`，可逐条复核。
>
> ⚠️ **本文档分成两部分**：
> **第 0~7 节 = 检查当时的原始结论（保留不改）**；
> **第 8 节 = 已实际实施的修复记录**：C1~C15、C17、C18、C20 已改（含 8.1 里那处自检的附加修正），
> C16（`K230_test` 400 B）与 C19（整页重绘）按 8.2 节说明**故意保留**。
> 全量编译 + 链接：**0 Error(s) 0 Warning(s)**。
> 阅读时请对照第 8 节看"哪些已经改了"。

---

## 0. 摘要（先看这 5 条）

| # | 结论 | 依据 | 严重度 |
|---|------|------|--------|
| **1** | **M1/M2 编码器“没示数”不是驱动代码的问题**。四路编码器走的是**同一段代码、同一张参数表**，M3/M4 用同一段代码能读到数，说明 `TIM_EncoderInterfaceConfig / TIM_ICInit / Encoder_ReadDelta` 这条链路本身是好的。**有区分度的变量只剩“PA0/PA1/PA6/PA7 这四个引脚上到底有没有边沿信号”**（以及这几个引脚是否被板载外设占死）。→ 见第 5 节，有 4 步 10 分钟定位法。 | `Drive/Source/encoder.c:38-52/62-126` | ✅ 已恢复（偶发/接触类，根因未确认，见 **5.4**） |
| **2** | **另外发现 1 个致命冲突**：`Lidar_Init()` 打开了 USART3 的 IDLE 中断并使能了 NVIC，但**整个 Keil 工程里没有任何 `USART3_IRQHandler`**（唯一的实现 `asrpro.c` 没有加进工程）。只要 LD19 雷达插上并开始吐数据，就会跳进启动文件的 `Default_Handler`（`B .`）→ 整车卡死/严重降速。 | `Program/Source/lidar.c:71-76`；`asrpro.c:84`（未编译）；启动文件 `startup_stm32f40_41xxx.s:284/367` | 🔴 P0 |
| **3** | **RTOS 已经从“单任务搬迁”扩成了 9 个任务，但共享资源没有按“单写者”原则隔离**：FatFs 被 `UiTask` 和 `LogTask` 同时访问（`FF_FS_REENTRANT=0`）、I2C1/MPU6050 被 `NavTask` 和 `UiTask` 同时访问、电机输出有 4 个写者、`g_mode` 有 4 个写者且无临界区。这是当前最大的架构风险，也是“偶发死机/偶发乱动”的最可能来源。 | 第 4 节 C3~C6 | 🔴 P0/P1 |
| **4** | **定时器分工是干净的**：TIM1=PWM，TIM2=1 MHz 时基，TIM3/4/5/8=四路编码器，TIM11=蜂鸣器，TIM12=舵机(已禁用)。**没有任何一个定时器被两个模块同时使用**。冲突不在定时器，在“引脚”、“中断向量”、“共享数据”。 | 第 3 节全表 | 🟢 |
| **5** | **模式状态机有 14 个枚举，只有 4 个真的会驱动底盘**（`MANUAL` / `AVOID` / `VISION` / `ESTOP`，`IDLE` 是空状态）：`MODE_SCAN/APPROACH/ALIGN/WAIT_ARM` 整条采摘链**没有入口（不可达死代码）**，`MODE_PATROL`、`MODE_RADAR` 没有任何运动实现。按 KEY3 切换的 6 个模式里有 2 个是“空转”。 | `main.c:62-63/178-199` | 🟠 P1 |

---

## 1. 整个工程的逻辑

### 1.1 分层与依赖方向

```
MY   (APP)      main.c（任务/模式机）  stm32f4xx_it.c（SDIO/DMA 中断）  FreeRTOSConfig.h
  │                     ↓ 只向下
BSP             bsp.c → BSP_Init()：唯一的上电 bring-up 入口
  │
Mid  (算法)     location.c（GPS+惯导融合）  navlog.c（轨迹/CSV/BMP）
                mapview.c（投影/绘图）     map_collent.c（拓扑图+寻路）
                menu.c（按键菜单 + LCD 分页 + 电机自检）
  │
Program (模块)  car.c（底盘合成）  gps.c  bluetooth.c  K230.c  lanyacon.c
                lidar.c  estop.c  uart_host.c   （asrpro.c = 未编译）
  │
Drive (外设)    motor/encoder/timer/delay/usart/lcd_*/mpu6050/sensor_adc/
                servo/ultrasonic/buzzer/car_led/car_key/led/key
  │
STM32 标准外设库 + CMSIS  +  FatFs(SDIO)  +  FreeRTOS(port ARM_CM4F + heap_4)
```

**依赖检查结果（本次实测）**：

- Drive/Program 层没有一个文件反向 `#include` 上层头文件，方向是干净的。
- `Menu_*`（Mid）反向调用了 `App_SetMode/App_SwitchMode`（MY），属于“Mid → APP”的逆向依赖，靠 `menu.h:35-36` 的声明绕过去了。**编译能过，但违反分层约定**；要修就把“模式切换”收成一个 `App_*` 回调，或把模式机整体上移到 MY。
- `car.c:9` 注释写 `M1=左前 M2=右前 M3=左后 M4=右后`，而 `motor.h:7-10` 与 `car.h:37-38`（`CAR_LEFT_MOTORS {MOTOR_1, MOTOR_2}`）写的是 **M1/M2=左侧、M3/M4=右侧**。两份注释**互相矛盾**，见 4 节 C12 —— 这条和你的编码器故障直接相关（先确认你说的“M1、M2”到底是左侧两个还是前面两个）。

### 1.2 上电初始化顺序（`BSP/Src/bsp.c:58-95`）

| 步 | 调用 | 做的事 | 关键依赖 / 注意 |
|---|---|---|---|
| 1 | `Delay_Init()` | 内部只调 `Timer_Init()`（`delay.c:32-35`） | SysTick 已归 FreeRTOS，不再配 |
| 2 | `Timer_Init()` | **TIM2** = 1 MHz 32 位自由计数器 | 后面所有 `Get_Ms()/Get_Us()/Delay_us()` 靠它 |
| 3 | `LED_Init()` | PC13 板载灯 | — |
| 4 | `Usart_Config()` | USART1 = **9600** 8N1，PA9/PA10；创建 printf 互斥量 | ⚠️ 见 C7：K230 协议是 115200 |
| 5 | `K230_Init()` | **再调一次 `Usart_Config()`**，开 USART1 RXNE + NVIC(7) | 重复配置无害 |
| 6 | `BT_Init()` | USART2 = 115200，PD5/PD6，RXNE + NVIC(5) | 蓝牙/WiFi/ESP32 共用 |
| 7 | `UARTHost_Init()` | 只清状态，不碰硬件 | — |
| 8 | `Lidar_Init()` | USART3 = 230400，PD8/PD9，DMA1_Stream1 + **IDLE 中断 + NVIC(5)** | 🔴 见 C1：没有 ISR |
| 9 | `Estop_Init()` | PE0 急停输入；**PB1 = 驱动器 STBY**，拉低 2 ms 再拉高 | 之后所有电机才受控 |
| 10 | `Car_Init()` | `Motor_Init()`（TIM1 四路 PWM）+ **`Encoder_Init()`（TIM5/TIM3/TIM4/TIM8）** | ← 编码器在这里初始化 |
| 11 | `SensorADC_Init()` | ADC1 + DMA2_Stream0 循环扫描 PA3/PA4/PB0/PC4 | — |
| 12 | `Servo_Init()` | **空函数**（`servo.c` 函数体整体注释，省电） | TIM12 没开时钟，PB14 没初始化 |
| 13 | `Ultrasonic_Init()` | 雷达位 PB15/PD14，车头 PD0/PD1 | — |
| 14 | `Buzzer_Init()` | PF10 推挽 + TIM11 时基（**中断默认关=静音**） | — |
| 15 | `CarLED_Init()` / `CarKey_Init()` | PG0~PG3；PE2/PE3/PG5/PG7 | — |
| 16 | `MPU6050_Init()` | I2C1 PB8/PB9，结果存 `s_mpuReady` | — |
| 17 | `SPI_LCD_Init()` | SPI3：PB3(SCK)/PB5(SDA)/PD11(CS)/PD12(DC)/PD13(BL) | — |
| 18 | `GPS_Init()` | USART6 9600，PG9/PG14，DMA2_Stream2 + IDLE + NVIC(6) | — |
| 19 | `Location_Init()` | **静止调零 0.6~1.8 s（阻塞，此时是裸机忙等）** | 上电别碰车 |
| 20 | `NavLog_Init()` | `f_mount` → `f_mkdir 0:/CARLOG` → `f_open TRACK.CSV` | 没卡也能跑 |
| 21 | `MapView_Init()` / `MapCollent_Init()` | 地图投影 / 200 节点拓扑 | — |

> **初始化顺序的安全性（本次已复核）**：`Car_Init()`（第 10 步，配 PA0/PA1/PA6/PA7 复用）之后，**再没有任何一行代码重新配置 GPIOA**。全工程 `GPIOA` 只出现在 `key.h`(PA15，从未调用)、`usart.h`(PA9/PA10)、`encoder.c`(PA0/1/6/7)、`sensor_adc.c`(PA3/PA4)。所以“编码器引脚被后面某个 init 覆盖”这个假设**在代码层面可以排除**。

### 1.3 运行期数据流

```
                     ┌──────────── 中断层（只搬字节，不做业务） ────────────┐
 GPS(USART6+DMA2_S2) │ ISR: IDLE → 组帧 → 语句队列[10]        NVIC 6      │
 K230(USART1)        │ ISR: RXNE → 环形缓冲[100]              NVIC 7      │
 蓝牙/ESP32(USART2)  │ ISR: RXNE → FIFO + 行缓冲              NVIC 5      │
 LD19(USART3+DMA1_S1)│ ISR: **缺失**  ← C1                                │
 SD卡(SDIO+DMA2_S3)  │ ISR: 置完成标志                        NVIC 5      │
 蜂鸣器(TIM11)       │ ISR: 翻转 PF10                          NVIC 6      │
                     └──────────────────────────────────────────────────────┘
        │                    │                     │                    │
   ┌────▼─────┐        ┌─────▼──────┐        ┌─────▼──────┐      ┌──────▼──────┐
   │ NavTask  │        │ CommTask   │        │  UiTask    │      │  LidarTask  │
   │ 50 ms    │        │ 10 ms      │        │ 10 ms 基   │      │  100 ms     │
   └────┬─────┘        └─────┬──────┘        └─────┬──────┘      └──────┬──────┘
        │GPS_Process         │BT字节→LanyaCon/命令   │按键/自检/LCD        │DMA→点云
        │MPU读+I2C1          │K230_Process→电机     │超声波×2/MPU读(I2C1!)│→ sectors[360]
        ▼                    ▼                     ▼                     ▼
  Location(融合)      Car_Move/Turn/Stop      电机PWM(TIM1)         Lidar_GetMinDistance
  latE7/lonE7         (MANUAL/VISION)          (自检时直写 TIM1)         │
        │                    │                                           │
        ▼                    ▼                                           ▼
  ┌──────────┐        ┌──────────────┐                          ┌────────────────┐
  │ LogTask  │        │ SafetyTask   │◄── g_distFixed<20cm ─────│  MissionTask   │
  │ 100 ms   │        │ 5 ms         │                          │  100 ms        │
  │ NavLog   │        │ Estop_Task   │── E-STOP ──► Car_Stop()  │ AVOID: 雷达避障 │
  │ →FatFs   │        │ (PE0 + 软件) │      + PB1=0(STBY)       │ 电机由本任务写  │
  └────┬─────┘        └──────────────┘                          └────────────────┘
       │
       ▼  SD: 0:/CARLOG/TRACK.CSV（每 10 点 f_sync）、0:/CARMAP/MAP.BIN、MAP.BMP
       ▼  LCD: MAP 页由 UiTask 每 300 ms 调 MapView_Refresh(1) 绘制
```

### 1.4 底盘控制权归属（**这是全工程最容易打架的地方**）

| 模式 | 谁在写电机 | 位置 |
|---|---|---|
| `MODE_IDLE` | 没人 | — |
| `MODE_MANUAL` | 蓝牙摇杆 `LanyaCon_Apply()`；ESP32/WiFi 文本命令 `Command_Exec` | `lanyacon.c:83`、`uart_host.c:34-41`（都在 **CommTask**） |
| `MODE_AVOID` | 雷达避障（左/右转或前进） | `main.c:167-177`（**MissionTask**） |
| `MODE_VISION` | K230 视觉帧 → 四轮差速 | `K230.c:257-273` → `K230_Process()`（**CommTask**） |
| `MODE_ESTOP` | `Estop_Task` 每 5 ms 强停 + 拉低 STBY | `estop.c:56-70`（**SafetyTask**） |
| 任意模式 | KEY4 电机自检直接 `Motor_SetSpeed()` | `menu.c:727/765/803`（**UiTask**） |
| 任意模式 | `App_SetMode()` 一进来就 `Car_Stop()` | `main.c:51`（UiTask / MissionTask / CommTask 都会调） |

→ **一共 4 个任务会写电机状态，没有任何互斥**（C4）。

### 1.5 安全（E-STOP）链路

- 4 个来源：物理急停 `PE0`（`ESTOP_SRC_GPIO`）、ESP32 命令 `estop`（`SOFT`）、车头超声 < 20 cm（`ULTRA`）、`LINK`（已定义未使用）。
- 触发动作：`Car_Stop()` + **PB1(STBY) 拉低**（硬件级封锁电机）。
- 复位：只有 `Estop_Reset()`（ESP32 `reset` 命令或 KEY3，见 `main.c:66-70`），并且**要求 PE0 已经回到高电平**。
- 逻辑缺陷：`Estop_Task` 每 5 ms 读一次 PE0，连续 3 次低才确认（15 ms 消抖，`estop.c:59-63`），但 `s_lowCount` **永远不会被降到 0 以外的值之后又重置**——`else s_lowCount = 0;` 有，逻辑是对的。唯一问题是 `Estop_Request()` 在 `SafetyTask` 里被调用时会调 `Car_Stop()`，而同时别的任务可能正在 `Car_Move()`（优先级 6 > 4，会打断）→ 见 C4。

### 1.6 定位 / 轨迹链路

- `GPS_Process()`（在 `Location_Update` 里被调，`location.c:234`）从语句队列取 NMEA，解析成 `lat/lon` + **定点 `latE7/lonE7`**（0.8 m 的 float 量化问题已是历史，见 `location.h:20-23`）。
- `Location_Update()`：GPS 有效就用 GPS（`fixSeq++` 通知下游）；GPS 失效（`LOC_GPS_TIMEOUT_MS=2500`）就用 MPU6050 惯导推算（`location.c:292-326`），并做 ZUPT 零速修正 + 陀螺/GPS 航向互补。
- `NavLog_Update()`（LogTask，100 ms）只在 `fixSeq` 变化时处理，做 5 道精度过滤（sat<4 / hdop>3 / 距离<1 m / 隐含速度>8 m/s / 无效定位），合格的写 CSV 并进 RAM 数组。
- `MapView_Refresh(1)`（UiTask，300 ms）把 E7 定点投影成 LCD 像素画折线；`MapView_SaveToSd()` 在 KEY4 时写 BMP。
- **注意：编码器完全没有参与定位。** `location.c` 里没有任何 `Encoder_*` 调用，里程计是“MPU 加速度积分”。这也是本次编码器故障目前“只影响显示、不影响功能”的原因，但一旦启用 PID 或启用“编码器里程替代加速度积分”，M1/M2 归零会立刻变成控制问题。

---

## 2. RTOS 是怎么设计的

### 2.1 内核配置（`MY/FreeRTOSConfig.h`，FreeRTOS V9.0.0）

| 配置 | 值 | 说明 |
|---|---|---|
| `configUSE_PREEMPTION` | 1 | 抢占式 |
| `configUSE_TIME_SLICING` | 1 | 同优先级时间片轮转 |
| `configTICK_RATE_HZ` | **1000** | 1 tick = 1 ms |
| `configMAX_PRIORITIES` | 32 | 0~31 |
| `configMINIMAL_STACK_SIZE` | 256 word = 1 KB | 空闲任务 |
| `configUSE_PORT_OPTIMISED_TASK_SELECTION` | 1 | 用 CLZ 指令选下一个任务 |
| `configUSE_MUTEXES / RECURSIVE / COUNTING` | 1/1/1 | 只有 printf 用了一个 mutex |
| `configUSE_TIMERS` | 1 | 软件定时器任务存在但**一个定时器都没用**，一直阻塞 |
| `configTIMER_TASK_PRIORITY` | 31 | 最高优先级（模板默认；真用软件定时器时要降） |
| `configTOTAL_HEAP_SIZE` | **36 KB** | heap_4 |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 溢出→钩子→关中断死循环 |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 同上 |
| `configASSERT` | → `vAssertCalled()` 关中断死循环 | 不再 printf（避免死等） |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | **优先级数值 ≥5 的 ISR 才允许调 FromISR API** |
| `xPortPendSVHandler / vPortSVCHandler / xPortSysTickHandler` | 映射到 `PendSV_Handler / SVC_Handler / SysTick_Handler` | 三个向量已还给 `port.c` |
| `NVIC_PriorityGroup` | Group_4（`main.c:78`） | 4 位全是抢占位 |

### 2.2 九个任务 + 两个系统任务（`main.c:98-262`）

| 任务 | 优先级 | 栈(word) | 栈(B) | 周期 | 干什么 | 风险点 |
|---|---|---|---|---|---|---|
| `Safety` | **6** | 256 | 1024 | 5 ms | `Estop_Task()`；`g_distFixed<20` → `Estop_Request()`；把 `g_mode` 强改 `MODE_ESTOP` | 栈最小；直接写 `g_mode`（无临界区） |
| `Control` | **5** | 384 | 1536 | 20 ms | **`Car_SpeedUpdate()` = 唯一读编码器的地方**（`Encoder_ReadDelta(0..3)`） | 只做显示用，不闭环 |
| `Nav` | **5** | 768 | 3072 | 50 ms | `Location_Update()`：GPS 解析 + **自己读 MPU6050(I2C1)** + 惯导推算 | 与 UiTask 抢 I2C1（C3） |
| `Lidar` | 4 | 512 | 2048 | 100 ms | `Lidar_Process()`：消费 DMA1_Stream1 环形缓冲，解 LD19 包 | 依赖 C1 修好才有意义 |
| `Mission` | 4 | 512 | 2048 | 100 ms | AVOID 避障决策；`MODE_SCAN→APPROACH→ALIGN→WAIT_ARM` 采摘状态机（**不可达**） | 写电机；`App_SetMode()` 写 `g_mode` |
| `Comm` | 3 | 512 | 2048 | 10 ms | `UARTHost_Process()`（蓝牙字节→摇杆/命令）、**`K230_Process()`（VISION 下直接写电机）**、`K230_GrabTask()`、100 ms 遥测 | 写电机；`printf` 多 |
| `UI` | 2 | 512 | 2048 | 10 ms 基 / 50·100·200·300 ms | 按键菜单、**电机自检**、超声波×2、**读 MPU6050(I2C1)**、LCD 刷新、蜂鸣器 | **2 KB 栈里跑 FatFs(LFN)** ← C5；阻塞 30 ms×2 |
| `Log` | **1** | 768 | 3072 | 100 ms | `NavLog_Update()`：过滤 + `f_write/f_sync` | 优先级最低 → 可能被饿；与 UiTask 抢 FatFs（C2） |
| `LED` | 3 | 128 | 512 | 500 ms | 翻转 PC13（`LED1_Toggle`） | — |
| `IDLE` | 0 | 256 | 1024 | — | 空闲 | — |
| `Tmr Svc` | 31 | 512 | 2048 | — | 软件定时器服务（未使用） | — |

**栈/堆预算（实测参数算出来的）**

```
用户任务：256+384+768+512+512+512+512+768+128 = 4352 word = 17 408 B
IDLE + Tmr：256+512                            =  768 word =  3 072 B
TCB/队列/互斥量等内核对象                      ≈  1 200 B
合计                                           ≈ 21.7 KB  / 36 KB 堆
→ 剩余约 14 KB（Debug 页 "Heap" 那一行可以直接看着验证）
```

**优先级设计评价**

- ✅ 合理的地方：安全最高（6）、控制次之（5）、UI 偏低（2）、日志最低（1）。Cortex-M4 @168 MHz 跑这套负载绰绰有余。
- ⚠️ 不合理的地方：
  1. **`Nav`(5) 和 `Control`(5) 同优先级**，靠时间片轮转。`Nav` 一次 `Location_Update()` 可能包含 GPS 批量解析 + 一次 I2C 读（几 ms），会把 `Control` 的 20 ms 节拍往后推 —— 现在只影响显示无所谓，**将来上 PID 就会变成控制抖动**。
  2. **`Log`(1) 只比 IDLE 高**。UI(2)+Comm(3)+Mission(4)+Lidar(4)+Nav(5)+Safety(6) 任意一个忙，`LogTask` 就可能错过 100 ms 节拍 → 轨迹点会“按 fixSeq 补”，不丢数据但时间戳会偏。
  3. **没有任何任务间通信对象**（队列/信号量/事件组/任务通知一个都没用），九个任务全靠全局变量裸共享 → C3~C6 的根因。
  4. `Safety` 栈 256 word（1 KB）是全场最小，而它调 `Estop_Task → Car_Stop → Motor_StopAll → GPIO + TIM_SetCompare` 一层套一层，余量不大。建议用 `uxTaskGetStackHighWaterMark` 实测后调。

### 2.3 双时间基准（一个容易被忽略的设计）

| 时基 | 谁提供 | 谁在用 | 回绕 |
|---|---|---|---|
| **SysTick**（1 ms tick） | FreeRTOS 内核 | `vTaskDelay/vTaskDelayUntil`（所有任务节拍） | 32 位 tick，49.7 天 |
| **TIM2**（1 MHz 自由计数） | `timer.c:25-37` | `Get_Ms()`（**所有业务周期判定**）、`Get_Us()`、`Delay_us()`、`NB_DelayMs()`、超声波测距 | **2³² µs = 4294.97 s ≈ 71.6 分钟** |

- 两者**不同步**：任务按 SysTick 醒，业务周期按 TIM2 判。两者都是无符号差值比较，短周期没问题。
- **但 `Get_Ms()` 每 71.6 分钟归零一次**。凡是写成“绝对时间比较”的地方都会在那 1~3 秒窗口里出错，例如 `menu.c:302` 的 `s_mapMsgUntil = Get_Ms() + 3000`（若用 `now < s_mapMsgUntil` 判断，回绕瞬间会立刻失效或永远不失效）。**建议统一改成 `(uint32_t)(now - start) >= period` 的相对写法。**
- 另外 `delay.c:43-67` 的 `Delay_ms()` 是双路径：调度器在跑→`vTaskDelay()`；启动阶段/中断里→TIM2 忙等。**在中断里调 `Delay_ms()` 会让整个系统卡住 n ms**（目前只有 `Estop_Init` 在启动阶段用过）。

### 2.4 中断优先级表（已全部下移到 syscall 边界 5 之下）

| 中断源 | 抢占优先级 | 使能位置 | 处理函数 | 清标志 | 备注 |
|---|---|---|---|---|---|
| `SysTick` (1 ms) | 15 | FreeRTOS | `port.c` | ✓ | 内核 |
| `PendSV` / `SVC` | 15 | FreeRTOS | `port.c` | — | 内核 |
| `USART1` (K230) | 7 | `K230.c:128-132` | `K230.c:95` | 读 DR ✓ | 与 printf 共用串口 |
| `USART2` (蓝牙/ESP32) | 5 | `bluetooth.c:70-74` | `bluetooth.c:85` | 读 DR ✓ | — |
| **`USART3` (LD19)** | **5** | `lidar.c:72-76` | **不存在** 🔴 | **没清 IDLE** | **C1 致命** |
| `USART6` (GPS) | 6 | `gps.c:339-341` | `gps.c:104` | 读 SR+DR ✓ | 正确的 IDLE 写法（可作为修 C1 的模板） |
| `SDIO` | 5 | `sdio_sd.c:581` | `stm32f4xx_it.c:139` | `SD_ProcessIRQSrc()` | — |
| `DMA2_Stream3` | 5 | `sdio_sd.c:587` | `stm32f4xx_it.c:144` | `SD_ProcessDMAIRQ()` | — |
| `TIM1_TRG_COM_TIM11` | 6 | `buzzer.c:59-63` | `buzzer.c:74` | `TIM_ClearITPendingBit` ✓ | 蜂鸣器方波 |
| ADC1 / DMA2_Stream0 | — | 无中断（纯 DMA 循环） | — | — | 直接读 `s_adcBuf` |
| DMA1_Stream1 (LD19) | — | 没开中断 | — | — | 靠 USART3 IDLE 搬数据 |
| DMA2_Stream2 (GPS) | — | 没开中断 | — | — | 靠 USART6 IDLE 搬数据 |

> 全部 ISR 优先级 ≥5（数值），**都允许调 `FromISR` API**；目前没有一个 ISR 真的调了 RTOS API，所以是安全的。

---

## 3. 每个 TIM 的使用（全表）

| TIM | 总线 | 定时器时钟 | PSC | ARR | 模式 | 引脚 / AF | 用途 | 初始化位置 | 冲突风险 |
|---|---|---|---|---|---|---|---|---|---|
| **TIM1** | APB2 | 168 MHz | 0 | 16799 | PWM1 ×4 | PE9/PE11/PE13/PE14，**AF1** | 四路电机 10 kHz PWM | `motor.c:84-133` | 🟢 独占。注意 `TIM_CtrlPWMOutputs(ENABLE)` 已开主输出 |
| **TIM2** | APB1 | 84 MHz | 83 | 0xFFFFFFFF | 自由计数 | **无引脚** | 1 MHz 时基：`Get_Us/Get_Ms/Delay_us` | `timer.c:25-37`（被 `Delay_Init` + `BSP_Init` **各调一次**，幂等无害） | 🟢 独占；⚠️ 71.6 min 回绕（C8） |
| **TIM3** | APB1 | 84 MHz | 0 | 0xFFFF | **编码器 TI12 ×4** | PA6(TIM3_CH1)/PA7(TIM3_CH2)，**AF2** | ENC2（M2） | `encoder.c:43-45` | 🟠 **本次故障点之一**（见 5 节） |
| **TIM4** | APB1 | 84 MHz | 0 | 0xFFFF | 编码器 TI12 ×4 | PB6/PB7，**AF2** | ENC3（M3） | `encoder.c:46-48` | 🟢 实测正常 |
| **TIM5** | APB1 | 84 MHz | 0 | 0xFFFF | 编码器 TI12 ×4 | PA0(TIM5_CH1)/PA1(TIM5_CH2)，**AF2** | ENC1（M1） | `encoder.c:40-42` | 🟠 **本次故障点之一**（见 5 节） |
| **TIM6** | APB1 | 84 MHz | — | — | 未使用 | 无引脚 | — | — | 🟢 空闲（只能做基础定时/触发） |
| **TIM7** | APB1 | 84 MHz | — | — | 未使用 | 无引脚 | — | — | 🟢 空闲 |
| **TIM8** | APB2 | 168 MHz | 0 | 0xFFFF | 编码器 TI12 ×4 | PC6(TIM8_CH1)/PC7(TIM8_CH2)，**AF3** | ENC4（M4） | `encoder.c:49-51` | 🟢 实测正常（高级定时器做编码器无需开 MOE） |
| **TIM9** | APB2 | 168 MHz | — | — | 未使用 | 可用 PE5/PE6(AF3) | 备用（唯一一对“双通道全空”的编码器候选） | — | 🟢 空闲 |
| **TIM10** | APB2 | 168 MHz | — | — | 未使用 | 可用 PF6(AF3) | 只有 1 通道，做不了正交 | — | 🟢 空闲 |
| **TIM11** | APB2 | 168 MHz | 167 | 999（动态改） | 更新中断 | PF10（**无复用，纯 IO 翻转**） | 无源蜂鸣器方波 | `buzzer.c:31-66` | 🟢 独占 |
| **TIM12** | APB1 | 84 MHz | — | — | **未初始化** | PB14(TIM12_CH1,AF9) | 舵机 50 Hz（`servo.c` 函数体全注释，省电） | 无 | 🟠 空实现（C14） |
| **TIM13/TIM14** | APB2 | 168 MHz | — | — | 未使用 | PF8/PF9(AF9) | 备用（各 1 通道） | — | 🟢 空闲 |

**结论：定时器本身没有任何冲突。** 8 个被用的 TIM 各管一件事，引脚也不重叠。工程里真正的“冲突”在别处（第 4 节）。

**各外设 DMA 通道分配（也复核过，不冲突）**

| 外设 | DMA | Stream | Channel | 模式 | 位置 |
|---|---|---|---|---|---|
| ADC1（4 通道扫描） | DMA2 | Stream0 | 0 | 循环 | `sensor_adc.c:54-67` |
| USART6_RX（GPS） | DMA2 | Stream2 | 5 | 循环 | `gps.h:29-30`、`gps.c:317-331` |
| SDIO | DMA2 | Stream3 | 4 | Normal（每块重装）+ TC 中断 + FIFO | `sdio_sd.h:259-269`、`sdio_sd.c:488-505` |
| USART3_RX（LD19） | DMA1 | Stream1 | 4 | 循环 | `lidar.c:51-69` |
| LCD（SPI3） | 无 DMA，纯轮询 | — | — | — | `lcd_spi_154.c` |

---

## 4. 冲突与隐患清单

> 分级：🔴 P0 = 会死机/数据损坏；🟠 P1 = 功能错误或时序不可靠；🟡 P2 = 整洁性/维护性。

### 🔴 C1（P0）USART3 的 IDLE 中断没有处理函数 → 插上 LD19 就卡死

**证据链**

1. `lidar.c:71-77`：`USART_ITConfig(USART3, USART_IT_IDLE, ENABLE)` + `NVIC_Init(USART3_IRQn, 抢占 5, 使能)`。
2. 全工程 `USART3_IRQHandler` **只在 `Program/Source/asrpro.c:84` 出现一次**。
3. 但 `Project/STM32F407.uvprojx` 的文件列表里**没有 `asrpro.c`**（只有 car/gps/bluetooth/K230/lanyacon/uart_host/estop/lidar）。`Project/.eide/eide.yml` 里有，但那份配置是**旧版**（没有 RTOS/lidar/estop/uart_host/map_collent），实际不用它编译。
4. 启动文件里 `USART3_IRQHandler` 是 `[WEAK]`（`startup_stm32f40_41xxx.s:284`），弱符号落到 `Default_Handler`（`:243`，内容是一条 `B .`）。

**后果**：LD19 每 ~3 ms 一个包，包间一定有总线空闲 → IDLE 置位 → 进 `Default_Handler` 死循环，而且 IDLE 标志**永远没人清**，所以中断会一直挂着。表现是“雷达一插上，整车像卡住一样（LCD 不刷新、按键无响应、超声卡住）”。**现在的车能跑，很可能只是因为 LD19 根本没插。** 这和你们上次修掉的 SDIO/DMA2_Stream3 是同一类坑。

**修法（二选一，推荐 A）**

```c
/* A. 在 lidar.c 里补上（照抄 gps.c:104-111 的写法） */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART3);      /* 读 SR+DR 清 IDLE：这一步不能省 */
        s_idleFlag = 1;                 /* 只置标志，搬运放到 Lidar_Process() */
    }
}

/* B. 如果不装雷达：注释掉 lidar.c:71 的 USART_ITConfig 和 72-76 的 NVIC_Init */
```

### 🔴 C2（P0）FatFs 被两个任务同时访问，而 `FF_FS_REENTRANT = 0`

| 任务 | 调用的 FatFs API | 位置 |
|---|---|---|
| `LogTask`(优先级 1) | `f_write` / `f_sync` / `f_close` / `f_open`（重试） | `navlog.c:269/283/272/65` |
| `UiTask`(优先级 2) | `f_stat` / `f_getfree`（TF 页） | `menu.c:338-342` |
| `UiTask`(优先级 2) | `f_open` / `f_write` / `f_sync`（保存地图） | `navlog.c:409-447`（`Menu_MapSave → MapView_SaveToSd`）、`map_collent.c:330` `map_write_SD` |

`FatFs/ffconf.h:249` 是 `#define FF_FS_REENTRANT 0`。`兼容解决方案.md` 里“所有 SD 访问都在同一个任务里，天然串行”的前提**在拆成 9 个任务之后已经不成立了**。
后果：切到 TF/MAP 页按 KEY4、或正好在 `f_sync` 中间被 `UiTask` 抢占时，可能破坏 FAT 表、`FIL` 结构体、或让 `s_csv` 处于半写状态 → “偶发 SD 卡损坏/白卡”。

**修法**：①推荐——所有 FatFs 调用收进 `LogTask`，其它任务用队列发请求（“存地图”“查容量”）；②或 `FF_FS_REENTRANT=1` + 在 `ffsystem.c` 里实现 `ff_mutex_create/take/give/delete`（`configUSE_MUTEXES` 已经=1），并保证**同一时刻只用一个 FATFS/FIL 对象**。

### 🔴 C3（P0）MPU6050(I2C1) 被两个任务同时读

- `NavTask`(5)：`location.c:260` 每 50 ms `MPU6050_ReadData(&mpu)`；
- `UiTask`(2)：`main.c:234-235` 每 200 ms `MPU6050_ReadData(&g_mpu)`。

`mpu6050.c` 的读写是**多步 I2C 时序 + 超时轮询**（`MPU_WaitEvent/MPU_WaitFlagClear`），不是原子操作。`NavTask` 优先级更高，可以**在 UiTask 一次传输的中间抢占**，两边交替操作同一组 `I2C_CR1/DR` 寄存器 → 偶发 NACK、偶发读到半新半旧数据、偶发总线卡死（I2C 的 BUSY 位不复位只能靠 `I2C_DeInit` 恢复，而代码里没有恢复逻辑）。
**修法**：给 I2C1 加一个 mutex（`s_i2cMutex`），或规定只由 `NavTask` 读，`g_mpu` 改成从 `location` 暴露的只读快照。

### 🔴 C4（P0）电机输出有 4 个写者，没有互斥、没有“控制权令牌”

| 写者 | 触发 | 位置 |
|---|---|---|
| `MissionTask`(4) | `MODE_AVOID` | `main.c:167-177` |
| `CommTask`(3) | `MODE_MANUAL`（摇杆/命令）、`MODE_VISION`（K230） | `lanyacon.c:83-129`、`uart_host.c:34-41`、`K230.c:257-273` |
| `UiTask`(2) | KEY4 电机自检、`App_SetMode()` 里的 `Car_Stop()` | `menu.c:724-730/765/803`、`main.c:51` |
| `SafetyTask`(6) | 急停 `Car_Stop()` | `estop.c:41/67` |

后果：`Car_Move()` 内部是**四次独立的 `Car_SetWheelSpeed`**（`car.c:93-125`），中途被高优先级任务打断就是“一个 20 ms 周期里左右轮目标不一致”（车会抽一下）；自检时 `Motor_SetSpeed(300)` 与 K230/避障指令互相覆盖，自检数据也不可信。
**修法**：确立“**只有 `ControlTask` 能写电机**”，其它任务只投递 `{dir, speed}` 请求到队列；`ControlTask` 每 20 ms 取最新请求 → 统一输出。这也是原 `智能采摘小车方案.md` 第 9 节的方案。

### 🔴 C5（P0）`UiTask` 2 KB 栈里跑 FatFs（LFN 用栈缓冲）

- `ffconf.h:100`：`FF_USE_LFN = 2` = **长文件名工作缓冲放在栈上**（`FF_MAX_LFN=255`，约 256~512 B），而 `f_open → create_name() → get_fileinfo()` 还有多层嵌套。
- `UiTask` 栈 = 512 word = **2048 B**（`main.c:106`）。
- `UiTask` 会走到：KEY4 → `Menu_MapSave()` → `map_write_SD()` + `MapView_SaveToSd()` → `NavLog_WriteBmp1bpp()` → `f_open/f_write/f_sync`（`navlog.c:366-450`）；以及 TF 页每 300 ms 的 `f_stat/f_getfree`。
- 参照 `改动说明.md` 里 armlink `--callgraph` 实测：`main → … → f_open → DMA_DeInit` 最深链 **1780 B**。裸机时代 `Stack_Size` 因此从 1 KB 提到 4 KB。

→ **2048 B 装 1780 B 的链，只剩 268 B 余量，任何一点嵌套加深就触发 `vApplicationStackOverflowHook`（关中断死循环）**，现象是“一按保存地图就死”。
**修法**：把 `UiTask` 提到 768~1024 word；或（更好）C2 的修法一起做——把 FatFs 全部搬到 `LogTask`（栈 768 word=3072 B）。

### 🟠 C6（P1）`g_mode` 有 4 个写者，切换不是原子操作

- `main.c:52`（`App_SetMode`，可能来自 Ui/Comm/Mission）、`main.c:68`（KEY3 复位急停）、`main.c:119`（SafetyTask 强改 ESTOP）、`main.c:183/186/190/191/194/196`（MissionTask 内部流转）。
- “切模式”这一套动作是 `Car_Stop() → g_mode = mode → s_missionMs = Get_Ms() → K230_StartTrack/StopTrack() → printf`（`main.c:47-58`）——**中间任何一步被打断都会留下不一致状态**（例如已经停车了但模式还是旧的，或 K230 收到了 TRACK_START 但模式没切过去）。
**修法**：`g_mode` 只允许一个任务（建议 `ControlTask` 或新建 `ModeTask`）改写，其它任务发请求；或至少 `taskENTER_CRITICAL()` 包住。

### 🟠 C7（P1）USART1 波特率 9600，但 K230 协议写的是 115200

| 位置 | 值 |
|---|---|
| `Drive/Include/usart.h:11` | `#define USART1_BaudRate 9600` |
| `Program/Include/K230.h:8`（注释） | “USART1, PA9(TX)/PA10(RX), **115200** 8N1” |
| `K230_Init()` | 只是**再调一次** `Usart_Config()`，没有改波特率 |

后果：①K230 用 115200 说话 → 主控收到的全是乱码（`rxBadFrames` 会一直涨）；②`printf` 在 9600 下 **1 个字节约 1.04 ms**，`Menu_SelfTestTick` 里那股报文（`menu.c:751-824`，每秒 40+ 字符）会**让 UiTask 阻塞 40 ms 以上**，自检阶段屏幕会明显卡。
**修法**：明确一个值。如果 K230 端确实 115200，把 `usart.h` 改成 115200（顺带 printf 快 12 倍）；如果 K230 端是 9600，就改 `K230.h` 的注释和协议文档。

### 🟠 C8（P1）TIM2 每 71.6 分钟回绕 + 绝对时间比较

- `Get_Ms() = TIM2->CNT / 1000`，32 位 µs 计数器 4294.97 s 归零。
- 绝大多数地方用的是 `(uint32_t)(now - last) >= period`，**能容忍回绕**（写得很规范）。
- 但 `menu.c:302` 的 `s_mapMsgUntil = Get_Ms() + 3000;` 属于“未来时间戳”，跨回绕的 3 秒窗口内行为不对（第 71 分钟附近按保存地图，提示可能立刻消失或一直不消失）。
**修法**：改成记录 `s_mapMsgStart`，判断 `(uint32_t)(Get_Ms() - s_mapMsgStart) < 3000`。

### 🟠 C9（P1）超声波在 `UiTask` 里阻塞 30 ms ×2

`ultrasonic.c:79-107`：无回波时 `while(ECHO==0)` 要等满 `ULTRA_TIMEOUT_US=30000`。`main.c:230-233` 每 100 ms 测车头、每 300 ms 测雷达位 → **平均占用 UiTask 30%~40%**，最坏 60 ms/100 ms。期间 LCD 不刷新、按键不响应（虽然不会影响更高优先级任务）。另外 `SafetyTask`（5 ms）用的 `g_distFixed` 就是这里更新的 → 车头超声的“20 cm 急停”最坏有 100 ms + 30 ms 的延迟。**这是当前安全链路的真实响应上限**，需要知道。

### 🟠 C10（P1）`MissionTask` 的采摘状态机是死代码

- `MODE_SCAN / MODE_APPROACH / MODE_ALIGN / MODE_WAIT_ARM`（`main.c:178-199`）只有从 `MODE_SCAN` 才能进入，而**全工程没有任何一处 `App_SetMode(MODE_SCAN)`**：`App_SwitchMode` 的循环表只有 `IDLE/MANUAL/PATROL/AVOID/RADAR/VISION`（`main.c:62-63`），`uart_host.c` 也没有。
- `MODE_PATROL`、`MODE_RADAR`、`MODE_UNLOAD/INSPECT/RETURN` 没有任何运动实现（`MODE_RADAR` 用的舵机 `Servo_SetAngle()` 是空函数）。
→ 14 个模式里真正有行为的只有 `MANUAL / AVOID / VISION / ESTOP`（+ `IDLE` 什么都不做）。**“采摘”这条主线目前等于不存在**，KEY3 循环到 PATROL/RADAR 时车是不动的，容易被误判为“死机”。

### 🟠 C11（P1）`printf` 是“持锁忙等”，且被多任务共用

`usart.c:107-122`：拿 mutex → 逐字符 `while(TXE==RESET)` → 放锁。9600 下一条 40 字符日志 = 41 ms 的忙等。Comm/Ui/Mission 都会打印，**持锁期间别的任务想打印会阻塞**（`portMAX_DELAY`）。加上 USART1 还要给 K230 收帧（同一根线），高频打印会挤占 K230 的下行带宽。

### 🟠 C12（P1）电机编号 M1~M4 的“左右/前后”定义在工程里自相矛盾

| 位置 | 说法 |
|---|---|
| `Drive/Include/motor.h:7-10` | M1 左前、**M2 左后**、**M3 右前**、M4 右后 |
| `Program/Source/car.c:9`（注释） | M1 左前、**M2 右前**、**M3 左后**、M4 右后 |
| `Program/Include/car.h:37-38` | `CAR_LEFT_MOTORS {MOTOR_1, MOTOR_2}`、`CAR_RIGHT_MOTORS {MOTOR_3, MOTOR_4}`（= M1/M2 左侧） |
| `智能采摘小车方案.md:92` | “左=(M1+M2)/2、右=(M3+M4)/2”（= M1/M2 左侧） |
| `car.c` 实际代码 | 硬编码 `MOTOR_1/MOTOR_2` 当左侧、`MOTOR_3/MOTOR_4` 当右侧（`car.c:112-115`）；`CAR_LEFT_MOTORS`、`CAR_RIGHT_MOTORS`、`CAR_TURN_INNER_PCT` 这些宏**定义了但从来没被使用** |

→ 代码按“M1M2=左”实现。但如果你车上的线序其实是“M1M2=前轴”，那 `Car_Move(FORWARD)` 就是对的（同向前进），而 `Car_Turn` 的差速逻辑就完全是错的（它会按左右分，实际按前后分）。**先确认这一条，再谈编码器。** 这正是 5 节 Step 0 要做的事。

### 🟡 C13 ~ C20（P2，整洁性/维护性）

| # | 问题 | 位置 | 建议 |
|---|---|---|---|
| C13 | `asrpro.c` 在两个工程配置里状态不一致（Keil 无、EIDE 有），是 C1 的根源之一 | `uvprojx` / `eide.yml` | 统一工程配置，删掉不用的 asrpro.c 或正式加回并处理 USART3 冲突 |
| C14 | `servo.c` 三个函数体全注释，`Servo_Init/SetAngle/SetPulse` 是空函数，但雷达/避障仍会调 | `servo.c:31-96` | 加 `#warning` 或在菜单上显示 `Servo: OFF`，别让人以为在动 |
| C15 | `key.c` 的死代码：`KEY_Scan()` 里 `Delay_ms(10)` + `while(按下);`，且 `KEY_Init()` 全工程从未调用 | `key.c:51-71` | 直接删除 `key.c/h`（PA15 也就能回收做别的用） |
| C16 | `kmain` 里 `K230_test[400]` 的测试帧常量（400 B RAM）永远不被使用 | `K230.c:60-77` | 用 `#if 0` 包起来或删掉 |
| C17 | `menu.c` 的 `Menu_GetModeName()` 用于 `printf("Mode -> %s")`，但 `s_modeName` 只有 14 项、`g_mode` 若越界会读飞 | `menu.c:58-60/118-121`、`main.c:57` | 加 `if (g_mode >= MODE_NUM) g_mode = MODE_IDLE;` |
| C18 | 无独立看门狗（IWDG 未启用）；`HardFault_Handler` 只是 `while(1)`，现场无法定位 | `stm32f4xx_it.c:44-50` | HardFault 里把 `SCB->CFSR/HFSR/MMFAR/BFAR` 存全局变量，复位后用 `__attribute__((noinit))` 保留 |
| C19 | `MapView_Refresh(0)` 在非地图页每 300 ms 被调一次（只是清标志），`Menu_Refresh` 每 300 ms 全页重画（无脏区判断），SPI3 上刷满屏耗时不小 | `menu.c:236-273`、`mapview.c:538` | 只在数值变化时重画该字段 |
| C20 | `Estop_Request()` 会在 `SafetyTask`(5 ms) 里反复调 `Car_Stop()`（每次都 `Motor_StopAll()`），急停激活期间每 5 ms 写 4 路 TIM1 比较寄存器 | `estop.c:56-70` | 加 `if (s_active && !s_stoppedOnce)` 只停一次 |

---

## 5. 专项：M1/M2 编码器“转轮子没示数”（本次故障）

### 5.1 先建立一个可以被证明的事实：代码路径是**对称的**

`Drive/Source/encoder.c` 里，四路编码器用的是：

- **同一张参数表**（`encoder.c:38-52`，四行，每行 11 个字段）；
- **同一个初始化循环**（`encoder.c:62-126`，`for (i = 0; i < ENCODER_NUM; i++)`）；
- **同一个读取函数**（`encoder.c:128-152`）。

四行参数之间**只有 4 个字段不同**，其余全一样：

| 字段 | ENC1(M1) | ENC2(M2) | ENC3(M3) | ENC4(M4) | 是否已核对 |
|---|---|---|---|---|---|
| 定时器 | TIM5 | TIM3 | TIM4 | TIM8 | ✓ 都对 |
| 时钟位 | `RCC_APB1Periph_TIM5` | `…TIM3` | `…TIM4` | `RCC_APB2Periph_TIM8` | ✓ 都对 |
| 端口 | GPIOA | GPIOA | GPIOB | GPIOC | ✓ |
| 引脚 | PA0/PA1 | PA6/PA7 | PB6/PB7 | PC6/PC7 | ✓ 见下 |
| AF 号 | 2 | 2 | 2 | 3 | ✓ 见下 |
| 是否 APB2 | 0 | 0 | 0 | 1 | ✓ |

**引脚复用对照 STM32F407 数据手册（AF 表）逐条核对：**

| 引脚 | 本工程用的 | 数据手册支持 | 结论 |
|---|---|---|---|
| PA0 | TIM5_CH1, AF2 | TIM2_CH1 / **TIM5_CH1 (AF2)** / TIM8_ETR | ✅ |
| PA1 | TIM5_CH2, AF2 | TIM2_CH2 / **TIM5_CH2 (AF2)** | ✅ |
| PA6 | TIM3_CH1, AF2 | **TIM3_CH1 (AF2)** / TIM1_BKIN / TIM13_CH1(AF9) / SPI1_MISO | ✅ |
| PA7 | TIM3_CH2, AF2 | **TIM3_CH2 (AF2)** / TIM1_CH1N / TIM8_CH1N / TIM14_CH1(AF9) / SPI1_MOSI | ✅ |
| PB6/PB7 | TIM4_CH1/2, AF2 | TIM4_CH1/CH2 (AF2) | ✅ |
| PC6/PC7 | TIM8_CH1/2, AF3 | TIM8_CH1/CH2 (AF3) | ✅ |

**由此得到唯一合理的推论：**

> 既然 `TIM_EncoderInterfaceConfig + TIM_ICInit + TIM_GetCounter` 这套代码让 M3/M4 出了数，
> 说明**这段代码本身没有 bug**；
> 既然 M1/M2 与 M3/M4 在代码上只差“定时器实例 + 引脚”，而定时器实例的时钟使能/启动是同一段循环，
> 那么**故障只能落在“PA0/PA1/PA6/PA7 这四个引脚上有没有边沿信号”这一件事上**。

可能的物理原因只剩 4 类：

| 类别 | 具体情形 | 为什么“正好是左侧两个” |
|---|---|---|
| **① 编码器本体/线束** | 左侧两个编码电机的霍尔板没供电（VCC/GND 断）、A/B 线断、插头插反/错位、线序与右侧不同 | 左侧两根线走同一侧，共用同一个 5V 分支或同一段扎带，最容易一起坏 |
| **② 板载外设占死引脚** | 核心板把 PA0 做 WK_UP 按键（外部下拉）、PA6/PA7 接板载 SPI Flash / OLED 排针（MISO 被芯片驱动） | 这两个引脚**恰好都在 GPIOA**，而 M3/M4 在 GPIOB/GPIOC 上，那两组通常不带板载器件 |
| **③ 上拉不足 / 边沿太慢** | 编码器是“集电极开路”输出，靠 MCU 内部 ~40 kΩ 上拉；再加上 `TIM_ICFilter=0x0F`（8 次采样 + fDTS/32），慢沿会被完全滤掉 | 如果左侧线更长/更细，RC 更大，就先在这两路失效 |
| **④ 5 V 电平/地** | 左侧编码器由驱动器板 5 V 供电，而该路 5 V 没接或地虚接 | 同上，同一条供电分支 |

> ⚠️ 注意：**③ 里那个 `TIM_ICFilter = 0x0F` 四路是一样的**（`encoder.c:106`，一个结构体同时喂给 CH1 和 CH2），所以它本身不能解释“只有 M1/M2 坏”；但它是“线材稍差就丢脉冲/完全收不到”的放大器。等定位到具体原因后，建议把滤波降到 `0x03`（只改 `encoder.c:106` 这一行，两路同时生效）。

### 5.2 四步定位法（10 分钟，按顺序做，每步都能“排除一半可能”）

#### Step 0（2 分钟）：先确认“M1/M2”到底是哪两个轮子

**这一步同时解决 C12 的编号矛盾。**

1. 把车架空（轮子离地）。
2. 在**非地图页**短按 **KEY4** 启动电机自检（`menu.c:714`）。
3. 阶段 2 会依次只转 M1 → M2 → M3 → M4，串口（USART1，9600）会打印：

```
[TEST 2/3] each motor runs 1s @30%PWM (IN1/IN2 = actual pin level)
M1 run... IN1=1 IN2=0 (expect 1 0)
M1 stop, encoder delta = xxx
M2 run... ...
```

4. 记下“打印 M1 时，**物理上哪个轮子**在转”。四种可能：

| 现象 | 结论 |
|---|---|
| M1=左前、M2=左后 | 代码与 `motor.h` 一致，`car.c` 注释错 → 你坏的是“左侧两个” |
| M1=左前、M2=右前 | `car.c` 注释才是对的 → 你坏的是“前轴两个”，并且 `Car_Turn` 的差速逻辑**当前是错的** |
| M1/M2 都不是你手转的那两个 | 线序与编号完全对不上，先重新贴标签 |

> 顺带看 `encoder delta`：如果 M1/M2 阶段打印的 delta 是 0、而 M3/M4 是几百，**故障就复现了**，而且是在“电机自己转”而不是“手转”的条件下复现的 —— 这比手转更可靠（手转可能太慢，被 0x0F 滤波滤掉）。

#### Step 1（2 分钟）：换线，一次分清“编码器/线束”还是“MCU 引脚”

1. 断电，把**左前轮编码器插头**和**右前轮编码器插头**在驱动器板上**互换**（只换编码器那 4 根：VCC/GND/A/B，别动电机功率线）。
2. 上电，进电机自检，等到**阶段 3**（`[TEST 3/3] hand-turn each wheel in 10s`），这 10 秒里**慢慢**转左前轮，同时看串口每秒打印的：

```
  total: ENC1=xxx ENC2=xxx ENC3=xxx ENC4=xxx
```

| 结果 | 结论 | 下一步 |
|---|---|---|
| `ENC3` 开始跟着左前轮变、`ENC1` 不动 | **MCU 的 TIM4/PB6/PB7 通道是好的，坏的是“左前编码器本体或它的线”** | 换编码器电机 / 查那 4 根线，结束 |
| `ENC1` 仍然不动（接的是右前的好编码器） | **MCU 的 PA0/PA1 通道有问题**（引脚被占/AF 没生效/信号没进来） | 走 Step 2 |
| 两个都动了 | 说明原来是接触不良，已经好了；把滤波降到 0x03 继续观察 | 结束 |

对 M2 重复一次（和右后互换）。

#### Step 2（5 分钟）：寄存器取证，区分“没信号”还是“没复用”

在 `UiTask` 的 300 ms 分支里（`main.c:236` 那行后面）临时加一段，或者干脆在 `main()` 里 `vTaskStartScheduler()` 之前跑一次：

```c
/* ===== 临时诊断：编码器 4 路寄存器快照 ===== */
static void Diag_EncoderRegs(void)
{
    printf("GPIOA MODER=%08lX AFR0=%08lX IDR=%04X\r\n",
           (unsigned long)GPIOA->MODER, (unsigned long)GPIOA->AFR[0],
           (unsigned)GPIOA->IDR);
    printf("TIM5 CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04X CNT=%lu\r\n",
           (unsigned long)TIM5->CR1, (unsigned long)TIM5->SMCR,
           (unsigned long)TIM5->CCMR1, (unsigned)TIM5->CCER,
           (unsigned long)TIM5->CNT);
    printf("TIM3 CR1=%08lX SMCR=%08lX CCMR1=%08lX CCER=%04X CNT=%lu\r\n",
           (unsigned long)TIM3->CR1, (unsigned long)TIM3->SMCR,
           (unsigned long)TIM3->CCMR1, (unsigned)TIM3->CCER,
           (unsigned long)TIM3->CNT);
    /* 对照用：正常的那两路 */
    printf("TIM4 CNT=%lu  TIM8 CNT=%lu\r\n",
           (unsigned long)TIM4->CNT, (unsigned long)TIM8->CNT);
    printf("RCC APB1ENR=%08lX APB2ENR=%08lX\r\n",
           (unsigned long)RCC->APB1ENR, (unsigned long)RCC->APB2ENR);
}
```

**期望值（正常的那两路一定满足，可拿 TIM4/TIM8 和自己对比）**

| 寄存器 | 期望 | 含义 |
|---|---|---|
| `GPIOA->MODER` 的 bit[1:0]=10、bit[3:2]=10、bit[13:12]=10、bit[15:14]=10 | 0b10 = 复用 | PA0/PA1/PA6/PA7 处于 AF 模式 |
| `GPIOA->AFR[0]` 的低 8 个半字节里，第 0/1/6/7 个 = `2` | AF2 | 复用到 TIM5/TIM3 |
| `TIM5->CR1` bit0 | 1 | CEN=1，计数器在跑 |
| `TIM5->SMCR` 的 SMS[2:0] | `011` | 编码器模式 TI1+TI2 双边沿 |
| `TIM5->CCMR1` 的 CC1S[1:0]=01、CC2S[9:8]=01 | 01 | 通道映射到 TI1/TI2 |
| `TIM5->CCER` bit0=1、bit4=1 | 1 | CC1E/CC2E 使能 |
| `RCC->APB1ENR` | bit3(TIM5)=1、bit1(TIM3)=1、bit2(TIM4)=1 | 时钟已开 |

**判读表**

| 寄存器 | 边沿信号 | 判读 | 处理 |
|---|---|---|---|
| 全对 | **有**（`IDR` 对应位在变） | 那就只剩“计数器值没被读出来”这种极低概率事件 | 检查 `Encoder_ReadDelta` 的调用者（`ControlTask` 是否真在 20 ms 跑；在 MOTR 页看 `F:` 是否更新） |
| 全对 | **没有**（`IDR` 恒高/恒低） | **信号没进 MCU**：编码器没供电 / A·B 断 / 板载外设拉着 | 万用表量：编码器插头 VCC 对 GND 应有 4.5~5.2 V；A、B 对 GND 在慢转时应在 0~VCC 之间跳变（若有示波器直接看） |
| `MODER`/`AFR` **不对** | — | 引脚被别人重配了（例如板载外设或某段代码隐式改 GPIOA） | 打印 `GPIOA->MODER` 在 `BSP_Init()` 末尾和 `UiTask` 里各一次，看是否后来被改 |
| `SMCR`/`CCMR1`/`CCER` 不对，或 `APB1ENR` 对应位=0 | — | 定时器没配好（和代码不一致） | 说明有 `TIM_DeInit`/RCC 复位在背后发生；此时 `CR1.CEN` 一般也是 0 |

> 如果不想动 `IDR` 的采样率，也可以用一个更省事的“边沿计数器”（放在 `SafetyTask` 里 5 ms 调一次就够手转）：

```c
/* 5 ms 采样, 只统计 PA0/PA1/PA6/PA7 的跳变次数, 每秒打印一次。
 * 优点: 不改变引脚模式(IDR 在 AF 模式下仍然反映引脚电平), 手转一圈应该看到几十次跳变。 */
static void Diag_PinEdges(void)
{
    static uint8_t  s_prev;
    static uint32_t s_cnt;
    static uint32_t s_t0;
    uint8_t now = (uint8_t)(GPIOA->IDR & 0xC3);   /* bit0=PA0 bit1=PA1 bit6=PA6 bit7=PA7 */
    uint8_t chg = now ^ s_prev;
    s_prev = now;
    if (chg & 0x01) s_cnt++;
    if (chg & 0x02) s_cnt++;
    if (chg & 0x40) s_cnt++;
    if (chg & 0x80) s_cnt++;
    if ((uint32_t)(Get_Ms() - s_t0) >= 1000)
    {
        s_t0 = Get_Ms();
        if (s_cnt) printf("[DIAG] PA edges/s = %lu\r\n", (unsigned long)s_cnt);
        s_cnt = 0;
    }
}
```

#### Step 3：如果确认是“板载外设占死引脚”，怎么救

| 方案 | 做法 | 代价 | 适用 |
|---|---|---|---|
| **A. 换到 TIM9**（推荐） | ENC1 → `TIM9_CH1=PE5(AF3)`、`TIM9_CH2=PE6(AF3)`，两个引脚都空（老版引脚表 PE5/PE6 是空闲）。改 `encoder.c:40-42` 那一行 + `isApb2=1`（TIM9 在 APB2） | 改 1 行表 + 飞 2 根线 | 只救回**一路** |
| **B. 软件 EXTI 正交解码** | 任意两个空闲 GPIO（PE0/PE1、PD3/PD4、PF0/PF1…）配 EXTI 双边沿中断，在 ISR 里 `A^B` 判方向累加 | 写 ~60 行；168 MHz 下几百 Hz 的脉冲毫无压力 | 救回多路，**推荐作为两路一起救的方案** |
| **C. 先不救** | `location.c` 目前**完全不用编码器**，PID 也是注释状态（`car.c:220-263`），所以 M1/M2 归零**当前不影响行驶和定位**，只影响 MOTR 页显示和自检 | 0 | 赶进度时可以先放着，但**上 PID 之前必须解决** |

### 5.3 结论与推荐动作（按顺序）

1. **先做 Step 0**（KEY4 自检），确认坏的是“左侧两个”还是“前面两个”，同时把 C12 的编号矛盾一次性钉死。
2. **再做 Step 1**（左右编码器插头互换）。这一步能立刻把“MCU 侧问题”和“编码器/线束问题”分开，是最省时间的一步。
3. 如果指向 MCU 侧，做 **Step 2** 打印寄存器（`MODER/AFR/SMCR/CCER/APB1ENR`），拿正常工作的 TIM4 当参照。
4. 定位后按 Step 3 选方案（TIM9 或 EXTI 软件解码）。
5. **不管结果如何，都建议顺手做两件事**：把 `encoder.c:106` 的 `TIM_ICFilter` 从 `0x0F` 降到 `0x03`；把 `Encoder_ReadDelta()` 的“回绕余量”写进注释 —— 16 位回绕差值只要两次读取之间 **< 32768 个脉冲**就是正确的，按 20 ms 周期允许到约 1.6 M count/s，而实测量级只有 2~3 万 count/s，余量约 50 倍以上，**当前 20 ms 周期是安全的**（如果有人把读取周期改到 100 ms 以上就需要注意）。

### 5.4 本次故障的最终结论（2026-11 更新）

| 项 | 内容 |
|---|---|
| 现象 | M1/M2（左侧两路，TIM5 + TIM3）手转轮子没有示数变化；M3/M4（TIM4 + TIM8）正常 |
| 代码层结论 | 四路配置**完全对称**、定时器/引脚复用**无冲突**（第 5.1 节已逐条核对数据手册）→ 故障只可能落在"PA0/PA1/PA6/PA7 引脚上有没有边沿信号"这一层 |
| 本轮固件侧改动 | ① `TIM_ICFilter` 由 `0x0F`（连续 8 次一致 + fDTS/32）降到 `0x03`（连续 4 次 + fDTS/16）；② 新增 `Encoder_DiagDump()`（KEY4 自检阶段 3 每秒打印 MODER/AFR/IDR + TIMx 寄存器） |
| 实测结果 | **新固件 + 重新接线/上电后，四路都恢复正常** |
| 定性 | **偶发/接触类问题，根因未确认**。最可能是编码器插头或线束接触不良（含 5V/GND 虚接），在重新插拔/重新上电后恢复；**也不能排除是 `0x0F` 强滤波把慢沿整段滤掉**——这两者在事后无法区分（两者都同时被"重新上电 + 换固件"覆盖了） |

> **为什么"根因未确认"要按"会复发"对待**：偶发性接触不良不会因为一次重插就消失，它只是在特定振动/温度/插拔角度下才出现。

### 5.4.1 复发时的第一手证据（照这个顺序抓，5 分钟定性）

1. **KEY4 跑电机自检，等到阶段 3**，把 `[ENC-DIAG]` 那几行**整段复制保存**（这是唯一能事后分析的东西）。
2. 手转对应轮子，看两个关键点：
   - `PA0/PA1/PA6/PA7 = x x x x` 在不在变 → **不变 = 信号没进 MCU**（线束/插头/编码器供电；若恒高/恒低也要怀疑板载外设拉着）
   - 在变但 `TIMx CNT` 不变 → **复用或定时器问题**（看 `MODER/AFR0` 是否 AF2、`CR1.CEN` 是否为 1、`SMCR.SMS` 是否 011）
3. 若同一路"时好时坏"，基本就是接触问题：重压端子 / 换插头 / 补焊 5V、GND，别再怀疑固件。

### 5.4.2 可选硬化（几十块钱，把"偶发"彻底消掉）

- 编码器 A/B 各加 **4.7 kΩ 外部上拉到 3.3 V**（MCU 内部上拉约 40 kΩ，长线 + 集电极开路输出时偏弱，慢沿容易被滤波吃掉）。
- 编码器 VCC/GND 走**独立的一对线**回到驱动板电源，不要和电机功率地共用一小段细线。
- 若换线后仍偶发丢脉冲，把 `encoder.c` 的 `TIM_ICFilter` 再往 `0x07`/`0x0B` 调（抗干扰 ↔ 吃慢沿的折中）。

### 5.4.3 ⚠️ 对后续开发的影响（重要）

**在编码器被确认稳定之前，不要打开 PID 闭环。**
原因：`Car_GetSpeed()` 就是 PID 的反馈量（`car.c` 的 `Car_PID_Update` 现在还是注释状态）。某一轮偶发掉成 0，PID 会把它当成"这个轮子不转"，把输出拉满 → 车突然跑偏。建议先连续跑几次自检、四路 `ENC-DIAG` 都稳定，再考虑开闭环。

---

## 6. 建议的修复顺序

| 阶段 | 内容 | 涉及文件 | 验证方式 |
|---|---|---|---|
| **P0-1** | 补 `USART3_IRQHandler`（C1） | `Program/Source/lidar.c` | 插上 LD19，Debug 页 `LD19: ON`、`Pkt` 递增、`CRC` 不涨，且系统不卡 |
| **P0-2** | 编码器定位（第 5 节 Step 0→3） | 视结论 | KEY4 自检阶段 3 四路都能变 |
| **P0-3** | FatFs 单写者化（C2）+ `UiTask` 栈加够（C5） | `main.c`、`menu.c`、`navlog.c`、`ffconf.h`/`ffsystem.c` | 地图页连按 KEY4 保存 10 次不挂；Debug 页看 `ui stack min` 余量 |
| **P0-4** | I2C1 加锁（C3） | `mpu6050.c` 或 `main.c` | 连续运行 30 分钟，`Zero/BiasZ` 不异常、LCD 姿态值不跳变 |
| **P1-1** | 电机单写者化：只有 `ControlTask` 写电机（C4），`g_mode` 单写者（C6） | `main.c`、`car.c` | 模式切换时轮速不抽；自检不受 K230/避障干扰 |
| **P1-2** | USART1 波特率统一（C7） | `usart.h` 或 `K230.h` | K230 帧能正常解析（`rxframes` 增长） |
| **P1-3** | 要么把采摘状态机接上入口，要么把 `MODE_PATROL/RADAR/SCAN…` 从 KEY3 循环里去掉（C10） | `main.c`、`menu.c` | 按键行为与文档一致 |
| **P1-4** | `s_mapMsgUntil` 改相对时间（C8） | `menu.c:302` | 静态检查 |
| **P2** | C13~C20 逐条清理 | — | — |

---

## 7. 附录

### 7.1 任务速查卡（贴在显示器上）

```
优先级 6  Safety   5ms   急停/PE0/STBY          ← 最高
优先级 5  Control  20ms  ★唯一读编码器★
优先级 5  Nav      50ms  GPS解析 + 读MPU(I2C1)
优先级 4  Lidar    100ms LD19 DMA
优先级 4  Mission  100ms 避障决策 + 写电机
优先级 3  Comm     10ms  蓝牙/K230 + 写电机
优先级 3  LED      500ms PC13 心跳
优先级 3  Sensor   10ms  超声波×2（第 8 节新增，原来是 UiTask 里阻塞调用）
优先级 2  UI       10ms  按键/LCD（FatFs 已移出，栈 512→768 word）
优先级 1  Log      100ms 轨迹写卡(FatFs)
优先级 0  IDLE
优先级 31 Tmr Svc       未使用
tick = 1ms，堆 36KB，已用 ≈21.7KB
```

### 7.2 关键宏一览（调参直接改这里）

| 宏 | 值 | 文件 |
|---|---|---|
| `DEFAULT_SPEED` | 30 | `main.c:22` |
| `CAR_OPENLOOP_SCALE` | 16（PWM‰ = 目标×16） | `car.h` |
| `CAR_TURN_INNER_PCT` | 40（**未使用**） | `car.h` |
| `LANYA_SPEED_MAX / TIMEOUT_MS` | 60 / 500 ms | `lanyacon.h` |
| `K230_BASE_SPEED / CAR_SPEED / TIMEOUT_MS` | 40 / 60 / 500 ms | `K230.c:44-51` |
| `K230_GRAB_TIMEOUT_MS` | 3000 ms | `K230.c:46` |
| `ULTRA_TIMEOUT_US / ULTRA_MAX_CM` | 30000 / 400 | `ultrasonic.h` |
| `LOC_GPS_TIMEOUT_MS` | 2500 ms | `location.h:32` |
| `LOC_CAL_SAMPLES / RETRY` | 120 / 3（≈0.6 s） | `location.h:56-57` |
| `LOC_ZUPT_ACC / GYRO` | 0.30 / 2.0 | `location.h:46-47` |
| `GPS_LINE_QUEUE / PARSE_BURST` | 10 / 4 | `gps.h` |
| `NAVLOG_MAX_POINTS / MIN_DIST_M / MAX_HDOP` | 1024 / 1.0 m / 3.0 | `navlog.h` |
| `CAR_KEY_DEBOUNCE_MS / LONG_MS` | 20 / 1000 | `car_key.c:22-23` |
| `configTOTAL_HEAP_SIZE` | 36 KB | `FreeRTOSConfig.h` |

### 7.3 本次审查覆盖的文件

`MY/`（main.c、stm32f4xx_it.c、FreeRTOSConfig.h）、`BSP/`（bsp.c/h）、
`Drive/`（motor、encoder、timer、delay、usart、usart.h、servo、ultrasonic、buzzer、car_key、car_led、sensor_adc、mpu6050、key、lcd_spi_154.h）、
`Program/`（car、gps、bluetooth、K230、lanyacon、lidar、estop、uart_host、asrpro—仅核对未编译）、
`Mid/`（location、navlog、menu、mapview、map_collent 的部分）、
`RTOS/`（配置与 port 映射）、`FatFs/`（ffconf.h、sdio_sd.h 的 DMA 定义）、
`Project/`（STM32F407.uvprojx 文件与包含路径、.eide/eide.yml）、
`STM32/.../startup_stm32f40_41xxx.s`（向量与弱符号）。

### 7.4 一句话总结

> **定时器分工是干净的，RTOS 也能跑；真正的问题是“共享资源没有主人”和“中断向量缺了人”。而 M1/M2 编码器这件事，代码层面已经证明四路完全对称 —— 请先做第 5 节的 Step 0/Step 1，别在 `encoder.c` 里找 bug。**

---
---

# 8. 修复记录（已实施）
//2026年11月 · 本轮改动

> **验证方式**：`tools/build_check.ps1` 用 ARMCC 5.06 update 6，按 `Project/STM32F407.uvprojx` 的**同一份文件列表 / 包含路径 / 预处理宏**全量编译并链接：
> **0 Error(s)，0 Warning(s)**（改前也是 0/0，可当回归基线用）。
> **编码**：改动的文件都保持原编码（多数 GB2312、`K230.c` 仍是 UTF-8），未出现乱码。
> **可回滚**：每个被改文件的原件都在 `tools/patch/backup/`（GB2312 原样字节）。

## 8.0 镜像变化（同一套编译参数下的对比）

| 项目 | 改前 | 改后 | 说明 |
|---|---|---|---|
| Code | 74,310 | 76,078 | +1,768（新增命令邮箱、编码器诊断、日志作业、HardFault 取证） |
| RO Data | 20,374 | 20,818 | +444 |
| RW Data | 828 | 824 | -4 |
| ZI Data | 77,924 | 77,992 | +68（多了一个 `Sensor` 任务栈：384 word = 1.5 KB 从堆里出，ZI 只多了几个静态变量） |

> 任务数：9 → **10**（新增 `Sensor`）。堆还是 36 KB，预估占用约 23.2 KB（Debug 页 `Heap` 那行可复核）。

## 8.1 逐条对照

| 编号 | 原问题 | 已做的修复 | 涉及文件 | 上机怎么验证 |
|---|---|---|---|---|
| **C1** 🔴 | USART3 开了 IDLE 中断但没有 `USART3_IRQHandler`，插雷达就跳 `Default_Handler` 死循环 | 新增 `USART3_IRQHandler()`：`USART_GetITStatus()` 读 SR + `USART_ReceiveData()` 读 DR（这才是清 IDLE 的正确顺序）；数据搬运仍在 `Lidar_Process()` | `Program/Source/lidar.c` | 符号表里 `USART3_IRQHandler` 已在 **0x080097e7**（真实代码，不再是 weak）。插上 LD19：Debug 页 `LD19:ON`、`Pkt` 递增、`CRC` 不涨，整机不卡 |
| **C2** 🔴 | FatFs 被 `UiTask` / `LogTask` 同时访问（`FF_FS_REENTRANT=0`） | 新增 **作业+快照** 机制：`NavLog_RequestSaveMap/RequestSdInfo/Sync/Clear` 只置标志，真正的 `f_open/f_write/f_stat/f_getfree/f_sync` 全部在 `NavLog_Service()` 里、由 `LogTask` 执行；结果放 `NavLog_Info_TypeDef` 快照给 UI 读 | `Mid/Inc/navlog.h`、`Mid/Src/navlog.c`、`Mid/Src/menu.c`、`MY/main.c` | 地图页连按 KEY4 保存 10 次不挂；TF 页的容量/文件大小 0.5 s 内刷新 |
| **C3** 🔴 | MPU6050(I2C1) 被 `NavTask` 和 `UiTask` 同时读 | **单读者**：只保留 `Location_Update()` 里的读取，读到的值发布到 `g_mpu` 供菜单显示；`UiTask` 里那段 200 ms 的 `MPU6050_ReadData(&g_mpu)` 已删除 | `Mid/Src/location.c`、`MY/main.c` | MPU 页姿态值仍刷新（由 NavTask 50 ms 更新）；连续跑 30 min 不再出现姿态值跳变/卡死 |
| **C4** 🔴 | 电机有 4 个任务写者，`Car_Move()` 中途被打断会出现"左轮改了右轮没改" | 新增 **底盘命令邮箱**：`Car_PostDir/PostTurn/PostLR/PostWheels/PostWheel/PostPwm/PostBrake`（任意任务投递，临界区保护，序号判新）；**只有 `ControlTask` 每 20 ms 调 `Car_ControlTask()` 才真正写 TIM1**。`Car_Stop()` 例外（全零幂等，任何任务可用），并且停车时会把挂起命令一起作废 | `Program/Include/car.h`、`Program/Source/car.c`、`MY/main.c`、`Program/Source/uart_host.c`、`Program/Source/lanyacon.c`、`Program/Source/K230.c`、`Mid/Src/menu.c` | Debug 页新增 `Src:` 显示当前是谁在开车（1=摇杆 2=避障 3=视觉 4=自检）；模式切换时轮速不再"抽一下" |
| **C5** 🔴 | `UiTask` 2 KB 栈里跑 FatFs(LFN 栈缓冲)，实测链 1780 B | FatFs 已移出 UI（见 C2）；同时把 `UiTask` 栈 **512 → 768 word（2 KB → 3 KB）** | `MY/main.c` | Debug 页 `UI stack min` 应稳定在 100 word 以上 |
| **C6** 🟠 | `g_mode` 多写者、越界无保护 | `App_SetMode()` 加 `mode >= MODE_NUM` 兜底；`Menu_GetModeName()` 同样兜底（原来直接拿 g_mode 当数组下标）；切模式统一走 `App_SetMode()`（`SafetyTask` 强制 ESTOP 的那一处保留，因为它是单条原子存储的最高优先级路径） | `MY/main.c`、`Mid/Src/menu.c` | 串口不会出现 `Mode -> ???`；按键/命令切模式行为一致 |
| **C7** 🟠 | USART1 实际 9600，K230 协议 115200 | `USART1_BaudRate` **9600 → 115200** | `Drive/Include/usart.h` | ⚠️ **串口助手必须同步改成 115200**，否则看到的全是乱码 |
| **C8** 🟠 | `Get_Ms()` 每 71.6 min 回绕 + 绝对时间比较 | 地图保存提示改成 `(Get_Ms() - saveMs) < 3000` 的相对比较 | `Mid/Src/menu.c`、`Mid/Src/navlog.c` | 静态检查即可 |
| **C9** 🟠 | 超声波在 `UiTask` 里阻塞 30 ms ×2 | 新增 **`SensorTask`（优先级 3、384 word）** 专管超声波（车头 100 ms / 雷达位 300 ms），用 `vTaskDelay(5)` 不用 `vTaskDelayUntil`（避免追赶式忙转）；UI 只剩按键 + 刷屏 + 蜂鸣 | `MY/main.c` | 按 KEY1/KEY2 翻页明显变跟手；`g_distFixed` 仍每 100 ms 更新 |
| **C10** 🟠 | 14 个模式里 9 个没实现，KEY3 会切到"车不动" | `App_SwitchMode()` 的循环表改成 **IDLE → MANUAL → VISION → AVOID**；`App_SetMode()` 对未实现模式打印 `Mode -> PATROL (no motion impl)`；`App_ModeImplemented()` 统一判定 | `MY/main.c` | 按 KEY3 只会在 4 个"真能用"的模式间循环 |
| **C12** 🟠 | M2/M3 的"左/右、前/后"注释互相矛盾 | `car.c` 顶部注释改正；新增 `s_leftWheel[2] = CAR_LEFT_MOTORS` / `s_rightWheel[2] = CAR_RIGHT_MOTORS`，`Car_Move()` 与邮箱的左右分配都走这张表（"哪几个轮子是左侧"从此只有 `car.h` 一处定义） | `Program/Source/car.c` | KEY4 自检阶段 2 打印 `M1..M4`，对照实际轮位即可确认 |
| **C11** 🟠 | `printf` 持锁忙等，USART 异常时永久卡死 | `fputc()` 的 `while(TXE==RESET)` 加了 20 万次上限（正常永远走不到）；配合 C7 提速 12 倍 | `Drive/Source/usart.c` | 拔掉调试线也不会卡住任务 |
| **C13** 🟡 | `asrpro.c` 不在 Keil 工程里，且它的 `USART3_IRQHandler` 与雷达冲突 | 文件顶部加了醒目说明：本文件未参与编译 / USART3 归 LD19 / 不要直接加回工程 | `Program/Source/asrpro.c` | 读注释即可 |
| **C14** 🟡 | 舵机是空实现但界面照常显示角度 | 新增 `Servo_IsEnabled()`（当前恒 0）；传感器页显示 `Servo: OFF (disabled)` | `Drive/Source/servo.c`、`Drive/Include/servo.h`、`Mid/Src/menu.c` | 传感器页第 3 行 |
| **C15** 🟡 | `key.c` 的 `while(按下);` 无上限死等 | 改成有上限轮询（最长约 1 s 后返回），并注明本文件未被工程引用 | `Drive/Source/key.c` | 静态检查 |
| **C17** 🟡 | `Menu_GetModeName()` 越界读飞 | 已加 `g_mode >= MODE_NUM` 兜底（见 C6） | `Mid/Src/menu.c` | 串口/页眉不会出现乱码模式名 |
| **C18** 🟡 | HardFault 只有 `while(1)`；无看门狗 | HardFault 里打印 `CFSR/HFSR/MMFAR/BFAR`（并拆出 MMFSR/BFSR/UFSR 三位字段）再死在原地给调试器；IWDG 以 `APP_USE_IWDG`（默认 0）形式提供，`LedTask` 里已接好 `App_IwdgFeed()` | `MY/stm32f4xx_it.c`、`BSP/Src/bsp.c`、`BSP/Inc/bsp.h`、`MY/main.c` | 想开看门狗：把 `bsp.c` 的 `APP_USE_IWDG` 改 1（LSI 30~60 kHz，标称 2 s，实际约 1.1~4 s） |
| **C20** 🟡 | 急停期间每 5 ms 重复写 TIM1 比较寄存器 | `Estop_Task()` 加了 `s_stopped` 闩锁：一次急停只 `Car_Stop()` 一次，之后只保证 STBY 为低 | `Program/Source/estop.c` | 急停时电机不再"每 5 ms 抖一下" |
| **编码器专项** | 缺一个"能上机自证"的手段 | 新增 `Encoder_DiagDump()`：打印 `GPIOA->MODER/AFR[0]/IDR` + `TIM3/4/5/8` 的 `CR1/SMCR/CCMR1/CCER/CNT` + `RCC->APB1ENR/APB2ENR` + 四路累计值；**电机自检阶段 3 每秒自动打印一次**；`TIM_ICFilter` 由 `0x0F` 降到 `0x03`（不吞慢边沿） | `Drive/Source/encoder.c`、`Drive/Include/encoder.h`、`Mid/Src/menu.c` | KEY4 自检 → 阶段 3 手转轮子，看 `PA0/PA1/PA6/PA7` 与 `TIM5/TIM3 CNT` 是否跟着变（判读表见第 5.2 节 Step 2）。→ **2026-11 实测：四路均已恢复**（偶发/接触类，根因未确认，见 **5.4**） |

## 8.2 改造过程中自查出的一处新问题（已一并修掉）

> 自检阶段 1 原来是 `for (i=0;i<4;i++) Motor_SetSpeed(i,300);` —— 直接写 4 个轮子。
> 改成"投递命令"以后，如果照抄成 `for (i=0;i<4;i++) Car_PostPwm(...)`，
> **邮箱里只会留下最后一条 → 只有 M4 会转**（这个坑是改完编译通过、但读代码时才发现）。
> 现在用 `Car_PostPwmAll()`（kind=8，一条命令带四个轮子），并把这个坑写进了注释。
>
> **由此得到一条改造规则**：凡是"一次要同时改多个轮子"的动作，必须**一条命令**，
> 不能拆成多次投递。目前全工程符合这条规则的调用点：`Car_PostDir/PostTurn/PostLR/PostWheels/PostPwmAll/PostBrake`。

## 8.3 故意没做的（保持原状）

| 项 | 原因 |
|---|---|
| `K230_test[400]` 测试帧常量（400 B RAM） | 是排查 K230 协议时很有用的样本，400 B 不紧张，保留 |
| `Menu_Refresh()` 每 300 ms 整页重画 | 属于观感/带宽优化，改它要动每个页面的绘制逻辑，风险大于收益 |
| PID 闭环 | 按你之前的要求保持注释状态（`car.c` 的 `Car_PID_Update` 仍未启用）。**注意：等你要开 PID 时，M1/M2 编码器必须先修好，否则闭环会把车带跑偏** |
| `MapView` 的跨任务读 | 现在只有 `UiTask` 读轨迹数组、`LogTask` 写；写入顺序是"先写点、后 `s_count++`"，读者最坏读到"少一个点"，不会读到错点，故未加锁 |
| 雷达/舵机的运动实现 | 属于功能开发（不是 bug），本轮不动 |

## 8.4 上机验证清单（按顺序做，约 10 分钟）

| # | 做什么 | 期望 |
|---|---|---|
| 1 | **串口助手改成 115200 8N1**，重新上电 | 开机 banner 正常；不再是乱码（C7 改了波特率） |
| 2 | 看 PC13 心跳灯 | 1 Hz 闪烁（调度器在跑） |
| 3 | 按 KEY3 循环 | 只在 IDLE/MANUAL/VISION/AVOID 之间切；串口打印 `Mode -> MANUAL` 等 |
| 4 | **KEY4 电机自检**（车架空）→ 阶段 1/2 | 4 个轮子依次转，串口打印每路的 `encoder delta`。**这一步就能确认 M1/M2 到底有没有脉冲** |
| 5 | 阶段 3（10 s 手转） | 每秒打印 `[ENC-DIAG]`：手转哪个轮子，就应看到对应 `PAx` 电平跳变 + 对应 `TIMx CNT` 变化。**按第 5.2 节 Step 2 的判读表定性**（IDR 不变=信号没进来；IDR 变 CNT 不变=AF/定时器问题） |
| 6 | 插上 LD19 雷达，进 Debug 页 | `LD19: ON`、`Pkt` 递增、整机不卡（C1） |
| 7 | 地图页连按 KEY4 保存 10 次 | 每次 3 s 内显示 `Saved BMP+MAP.BIN`，不死机、不白卡（C2/C5） |
| 8 | 蓝牙摇杆推一下 | 车动、松手 500 ms 自动停；Debug 页 `Src: 1`（C4） |
| 9 | 避障/视觉模式各试一次 | Debug 页 `Src:` 变成 2 / 3，切模式后自动停车 |

## 8.5 回滚方式

```
tools/patch/backup/            # 每个被改文件的原件（GB2312 原样字节，文件名里的 \ 用 __ 代替）
tools/patch/applied/           # 已应用的补丁（可读，记录每一处 old/new）
tools/build_check.ps1          # 全量编译+链接验证脚本（改完随时可跑）
```

- 想整体验证改动是否完整：跑 `powershell -ExecutionPolicy Bypass -File tools\build_check.ps1`，应为 `errors=0 warnings=0`。
- 想回退单个文件：把 `tools/patch/backup/<文件>` 拷回原位置即可（注意保持 GB2312 编码）。
- ⚠️ 回退 `lidar.c` 会**重新引入 C1**（雷达一插就卡死）；回退 `navlog.c`/`menu.c` 会重新引入"两个任务同时用 SD 卡"。


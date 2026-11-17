#include "stm32f4xx.h"
#include <stdio.h>
#include "timer.h"
#include "led.h"
#include "car.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "menu.h"
#include "location.h"
#include "K230.h"
#include "navlog.h"
#include "map_collent.h"
#include "uart_host.h"
#include "lidar.h"
#include "estop.h"
#include "bsp.h"
#include "lcd_spi_154.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEFAULT_SPEED 30

WorkMode_TypeDef g_mode = MODE_IDLE;
int16_t g_speed = DEFAULT_SPEED;
uint8_t g_mpuOK;
uint8_t g_basket;
float g_distFixed;
float g_distRadar;
float g_servoAngle = 90;
MPU6050_DataTypeDef g_mpu;

static uint32_t s_missionMs;
static uint8_t s_scanCount;

static void AppTaskCreate(void);
static void SafetyTask(void *arg);
static void ControlTask(void *arg);
static void NavTask(void *arg);
static void LidarTask(void *arg);
static void MissionTask(void *arg);
static void CommTask(void *arg);
static void UiTask(void *arg);
static void SensorTask(void *arg);
static void LogTask(void *arg);
static void LedTask(void *arg);

/* 这个模式到底有没有运动实现(只在切模式时打印提示用) */
static uint8_t App_ModeImplemented(WorkMode_TypeDef mode)
{
    switch (mode)
    {
        case MODE_IDLE:
        case MODE_MANUAL:
        case MODE_AVOID:
        case MODE_VISION:
        case MODE_ESTOP:
            return 1;
        default:
            return 0;
    }
}
void App_SetMode(WorkMode_TypeDef mode)
{
    WorkMode_TypeDef old = g_mode;
    if (mode >= MODE_NUM) mode = MODE_IDLE;     /* 越界保护: g_mode 会被当成 s_modeName[] 的下标 */
    if (Estop_IsActive() && mode != MODE_ESTOP && mode != MODE_IDLE) return;
    Car_Stop();
    g_mode = mode;
    s_missionMs = Get_Ms();
    s_scanCount = 0;
    if (mode == MODE_VISION && old != MODE_VISION) K230_StartTrack();
    else if (mode != MODE_VISION && old == MODE_VISION) K230_StopTrack();
    printf("Mode -> %s%s\r\n", Menu_GetModeName(), App_ModeImplemented(mode) ? "" : " (no motion impl)");
}

void App_SwitchMode(void)
{
    /* 只循环"真正实现了运动"的模式。PATROL/RADAR/SCAN/APPROACH/ALIGN/WAIT_ARM
     * 目前没有运动实现(采摘链没有入口, 雷达模式用的舵机是空函数), 放进循环
     * 只会让按键切到"车不动"的状态, 看起来像死机。 */
    static const WorkMode_TypeDef modes[] = {
        MODE_IDLE, MODE_MANUAL, MODE_VISION, MODE_AVOID
    };
    uint8_t i;
    if (Estop_IsActive())
    {
        if (Estop_Reset(ESTOP_SRC_SOFT)) g_mode = MODE_IDLE;
        return;
    }
    for (i = 0; i < sizeof(modes) / sizeof(modes[0]); i++)
        if (modes[i] == g_mode) break;
    App_SetMode(modes[(i + 1) % (sizeof(modes) / sizeof(modes[0]))]);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    BSP_Init();
    g_mpuOK = BSP_IsMpuReady();
    Menu_SplashScreen();
    Menu_Refresh();
    printf("\r\nSmart Picking Car - topology + local grid\r\n");
    printf("ESP32: USART2; LD19: USART3; K230: USART1\r\n");
    AppTaskCreate();
    vTaskStartScheduler();
    printf("[RTOS] scheduler start failed\r\n");
    while (1);
}

static BaseType_t App_Create(TaskFunction_t fn, const char *name, uint16_t stack, UBaseType_t priority)
{
    BaseType_t result = xTaskCreate(fn, name, stack, 0, priority, 0);
    if (result != pdPASS) printf("[RTOS] create %s failed\r\n", name);
    return result;
}

static void AppTaskCreate(void)
{
    if (App_Create(SafetyTask, "Safety", 256, 6) != pdPASS) while (1);
    if (App_Create(ControlTask, "Control", 384, 5) != pdPASS) while (1);
    if (App_Create(NavTask, "Nav", 768, 5) != pdPASS) while (1);
    if (App_Create(LidarTask, "Lidar", 512, 4) != pdPASS) while (1);
    if (App_Create(MissionTask, "Mission", 512, 4) != pdPASS) while (1);
    if (App_Create(CommTask, "Comm", 512, 3) != pdPASS) while (1);
    if (App_Create(UiTask, "UI", 768, 2) != pdPASS) while (1);      /* FatFs/长文件名已移出, 但留足余量 */
    if (App_Create(LogTask, "Log", 768, 1) != pdPASS) while (1);
    if (App_Create(LedTask, "LED", 128, 3) != pdPASS) while (1);
    if (App_Create(SensorTask, "Sensor", 384, 3) != pdPASS) while (1);
}

static void SafetyTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        Estop_Task(Get_Ms());
        if (g_distFixed > 0.0f && g_distFixed < 20.0f) Estop_Request(ESTOP_SRC_ULTRA);
        if (Estop_IsActive() && g_mode != MODE_ESTOP) g_mode = MODE_ESTOP;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
}

static void ControlTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        Car_ControlTask();      /* 唯一写电机的地方: 20ms 取一条底盘命令并输出 */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}

static void NavTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        uint32_t now = Get_Ms();
        Location_Update(now);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
    }
}

static void LidarTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        Lidar_Process(Get_Ms());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}

static void MissionTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    K230_Status_TypeDef *vision;
    (void)arg;
    for (;;)
    {
        uint32_t now = Get_Ms();
        vision = K230_GetStatus();
        if (g_mode == MODE_AVOID && !Estop_IsActive())
        {
            uint16_t front = Lidar_GetMinDistance(0, 45);
            if ((front && front < 600) || (g_distFixed > 0.0f && g_distFixed < 25.0f))
            {
                uint16_t left = Lidar_GetMinDistance(90, 45);
                uint16_t right = Lidar_GetMinDistance(270, 45);
                Car_PostTurn(CAR_SRC_AVOID, left >= right ? CAR_DIR_LEFT : CAR_DIR_RIGHT, g_speed);
            }
            else Car_PostDir(CAR_SRC_AVOID, CAR_DIR_FORWARD, g_speed);
        }
        switch (g_mode)
        {
            case MODE_SCAN:
                if (vision->raw.flag && vision->online) App_SetMode(MODE_APPROACH);
                else if ((uint32_t)(now - s_missionMs) >= 3000)
                { K230_StopTrack(); App_SetMode(MODE_PATROL); }
                break;
            case MODE_APPROACH:
                if (vision->raw.y < 50) App_SetMode(MODE_ALIGN);
                break;
            case MODE_ALIGN:
                if (vision->raw.x <= 20 && vision->raw.flag)
                { Car_Stop(); K230_SendCmd(K230_CMD_GRAB_ACK, 0, 0); App_SetMode(MODE_WAIT_ARM); }
                else if ((uint32_t)(now - s_missionMs) >= 15000) App_SetMode(MODE_PATROL);
                break;
            case MODE_WAIT_ARM:
                if (!vision->grabPending) { if (g_basket < 255) g_basket++; K230_GrabDone(); App_SetMode(MODE_PATROL); }
                else if ((uint32_t)(now - s_missionMs) >= 3000)
                { if (++s_scanCount > 1) App_SetMode(MODE_PATROL); else K230_SendCmd(K230_CMD_GRAB_ACK, 0, 0); }
                break;
            default: break;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}

static void CommTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastTelemetry = 0;
    (void)arg;
    for (;;)
    {
        uint32_t now = Get_Ms();
        UARTHost_Process(now);
        K230_Process();
        K230_GrabTask();
        if ((uint32_t)(now - lastTelemetry) >= 100)
        { lastTelemetry = now; UARTHost_SendTelemetry(now); }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

static void UiTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t lastKey = 0, lastLcd = 0;
    (void)arg;
    for (;;)
    {
        uint32_t now = Get_Ms();
        if ((uint32_t)(now - lastKey) >= 50) { lastKey = now; Menu_KeyProcess(); }
        if ((uint32_t)(now - lastLcd) >= 300) { lastLcd = now; Menu_Refresh(); }
        Buzzer_Task();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

/* 传感器任务: 超声波是"阻塞式"测距(无回波要等满30ms), 从 UiTask 挪出来以后
 * 屏幕刷新与按键响应不再被它拖住。优先级 3: 低于 Safety/Control/Nav/Mission/Lidar,
 * 高于 UI/Log, 保证 g_distFixed(20cm 急停判据) 的更新不会被刷屏挤掉。 */
static void SensorTask(void *arg)
{
    uint32_t lastFixed = 0, lastRadar = 0;
    (void)arg;
    for (;;)
    {
        uint32_t now = Get_Ms();
        if ((uint32_t)(now - lastFixed) >= 100)
        { lastFixed = now; g_distFixed = Ultrasonic_GetDistance(ULTRA_FIXED); }
        if ((uint32_t)(now - lastRadar) >= 300)
        { lastRadar = now; g_distRadar = Ultrasonic_GetDistance(ULTRA_RADAR); }
        vTaskDelay(pdMS_TO_TICKS(5));       /* 不用 vTaskDelayUntil: 内部阻塞会让它一路追赶 */
    }
}

static void LogTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        NavLog_Update(Get_Ms());
        NavLog_Service();       /* 执行其它任务挂过来的 SD 作业(存地图/刷盘/查容量) */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
    }
}

static void LedTask(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;)
    {
        LED1_Toggle;
        App_IwdgFeed();     /* 可选看门狗: APP_USE_IWDG=0 时是空操作 */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(500));
    }
}

volatile const char *g_stackOverflowTask;
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    g_stackOverflowTask = name;
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vAssertCalled(const char *file, unsigned long line)
{
    (void)file; (void)line;
    __disable_irq();
    for (;;);
}

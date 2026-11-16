#include "uart_host.h"
#include "bluetooth.h"
#include "lanyacon.h"
#include "menu.h"
#include "car.h"
#include "estop.h"
#include "location.h"
#include "gps.h"
#include "lidar.h"
#include "map_collent.h"
#include "sensor_adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static UARTHost_Status_TypeDef s_status;
static char s_line[UART_HOST_LINE_SIZE];
static uint16_t s_lineLength;
static uint8_t s_discardLine;

static int UARTHost_Value(const char *line, const char *key, char *out, uint16_t size)
{
    const char *p = strstr(line, key);
    uint16_t n = 0;
    if (!p || !size) return 0;
    p += strlen(key);
    while (*p == ' ' || *p == '	') p++;
    if (*p == '"') p++;
    while (*p && *p != ',' && *p != '}' && *p != '"' && n + 1 < size) out[n++] = *p++;
    out[n] = 0;
    return n > 0;
}

static void UARTHost_Manual(const char *command)
{
    if (!strcmp(command, "F") || !strcmp(command, "forward")) Car_Move(CAR_DIR_FORWARD, g_speed);
    else if (!strcmp(command, "B") || !strcmp(command, "back")) Car_Move(CAR_DIR_BACKWARD, g_speed);
    else if (!strcmp(command, "L") || !strcmp(command, "left")) Car_Turn(CAR_DIR_LEFT, g_speed);
    else if (!strcmp(command, "R") || !strcmp(command, "right")) Car_Turn(CAR_DIR_RIGHT, g_speed);
    else Car_Stop();
}

uint8_t Command_Exec(const char *line, const char *source)
{
    char value[40];
    char command[24];
    uint8_t hasCommand;
    uint8_t ok = 1;
    if (!line || !*line) return 0;
    hasCommand = UARTHost_Value(line, "\"cmd\":", command, sizeof(command)) != 0;
    s_status.rxCommands++;
    strncpy(s_status.lastCommand, line, sizeof(s_status.lastCommand) - 1);
    s_status.lastCommand[sizeof(s_status.lastCommand) - 1] = 0;
    if (source)
    {
        if (!strcmp(source, "WIFI")) s_status.wifi = 1;
        else if (!strcmp(source, "BT")) s_status.bluetooth = 1;
        else if (!strcmp(source, "VOICE")) s_status.voice = 1;
    }
    if (strlen(line) == 1 && strchr("FBLSR+-M", line[0]))
    {
        if (line[0] == '+') { if (g_speed <= 55) g_speed += 5; }
        else if (line[0] == '-') { if (g_speed >= 15) g_speed -= 5; }
        else if (line[0] == 'M') App_SwitchMode();
        else if (line[0] == 'S') Car_Stop();
        else if (g_mode == MODE_MANUAL && !Estop_IsActive()) UARTHost_Manual(line);
        return 1;
    }
    if (hasCommand && !strcmp(command, "voice") && UARTHost_Value(line, "\"text\":", value, sizeof(value)))
        return Command_Exec(value, "VOICE");
    if (!strcmp(line, "estop") || !strcmp(line, "stop") || (hasCommand && !strcmp(command, "estop")))
    {
        Estop_Request(ESTOP_SRC_SOFT);
        UARTHost_SendEvent("ESTOP", "ESP");
        return 1;
    }
    if (!strcmp(line, "reset") || (hasCommand && !strcmp(command, "reset")))
    {
        ok = Estop_Reset(ESTOP_SRC_SOFT);
        if (ok) App_SetMode(MODE_IDLE);
        return ok;
    }
    if (hasCommand)
    {
        strcpy(value, command);
        if (!strcmp(value, "goto_node"))
        {
            uint16_t id = 0;
            MapPath_TypeDef path;
            minimap *start = MapCollent_GetByIndex(0);
            if (UARTHost_Value(line, "\"id\":", value, sizeof(value))) id = (uint16_t)strtoul(value, 0, 0);
            ok = start && MapCollent_Plan(start->id, id, &path);
            if (ok) App_SetMode(MODE_PATROL);
        }
        else if (!strcmp(value, "mode") && UARTHost_Value(line, "\"v\":", value, sizeof(value)))
        {
            if (!strcmp(value, "PATROL")) App_SetMode(MODE_PATROL);
            else if (!strcmp(value, "MANUAL")) App_SetMode(MODE_MANUAL);
            else if (!strcmp(value, "IDLE")) App_SetMode(MODE_IDLE);
            else ok = 0;
        }
        else if (!strcmp(value, "speed") && UARTHost_Value(line, "\"v\":", value, sizeof(value)))
        {
            int speed = (int)(atof(value) * 100.0f);
            if (speed < 10) speed = 10;
            if (speed > 60) speed = 60;
            g_speed = (int16_t)speed;
        }
        else if (!strcmp(value, "teach") && UARTHost_Value(line, "\"act\":", value, sizeof(value)))
        {
            if (!strcmp(value, "add"))
            {
                char type[16] = "WAYPOINT";
                int16_t x = 0, y = 0;
                UARTHost_Value(line, "\"type\":", type, sizeof(type));
                if (UARTHost_Value(line, "\"x\":", value, sizeof(value))) x = (int16_t)(atof(value) * 100.0f);
                if (UARTHost_Value(line, "\"y\":", value, sizeof(value))) y = (int16_t)(atof(value) * 100.0f);
                ok = MapCollent_AddNode(x, y, 0,
                    !strcmp(type, "PARK") ? MAP_PARK : !strcmp(type, "UNLOAD") ? MAP_UNLOAD :
                    !strcmp(type, "INSPECT") ? MAP_INSPECT : !strcmp(type, "TARGET") ? MAP_TARGET : MAP_WAYPOINT,
                    type) != 0;
            }
            else if (!strcmp(value, "del") && UARTHost_Value(line, "\"id\":", value, sizeof(value)))
                ok = MapCollent_DeleteNode((uint16_t)strtoul(value, 0, 0));
        }
        else if (!strcmp(value, "ask"))
        {
            UARTHost_SendEvent("MAP_META", 0);
        }
        else ok = 0;
    }
    else if (!strcmp(line, "patrol") || !strcmp(line, "start")) App_SetMode(MODE_PATROL);
    else if (!strcmp(line, "manual")) App_SetMode(MODE_MANUAL);
    else if (g_mode == MODE_MANUAL && !Estop_IsActive()) UARTHost_Manual(line);
    else ok = 0;
    if (!ok) s_status.rxErrors++;
    return ok;
}

static void UARTHost_Write(const char *text)
{
    size_t length = strlen(text);
    if (length >= sizeof(s_status.lastTx)) length = sizeof(s_status.lastTx) - 1;
    memcpy(s_status.lastTx, text, length);
    s_status.lastTx[length] = 0;
    BT_SendString((char *)text);
}

void UARTHost_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_lineLength = 0;
    s_discardLine = 0;
}

void UARTHost_Process(uint32_t now)
{
    uint8_t byte;
    while (BT_ReadByte(&byte))
    {
        if (LanyaCon_Feed(byte)) { s_status.lastRxMs = now; continue; }
        if (byte == '\r' || byte == '\n')
        {
            if (s_lineLength)
            {
                s_line[s_lineLength] = 0;
                s_status.rxLines++;
                s_status.lastRxMs = now;
                strncpy(s_status.lastRx, s_line, sizeof(s_status.lastRx) - 1);
                s_status.lastRx[sizeof(s_status.lastRx) - 1] = 0;
                Command_Exec(s_line, "ESP");
                s_lineLength = 0;
            }
            s_discardLine = 0;
        }
        else if (s_lineLength == 0 && strchr("FBLRSM+-", byte))
        {
            char command[2] = { (char)byte, 0 };
            s_status.lastRx[0] = (char)byte;
            s_status.lastRx[1] = 0;
            Command_Exec(command, "ESP");
        }
        else if (!s_discardLine)
        {
            if ((uint32_t)s_lineLength + 1 < sizeof(s_line)) s_line[s_lineLength++] = (char)byte;
            else { s_discardLine = 1; s_lineLength = 0; s_status.rxErrors++; }
        }
    }
    LanyaCon_Task(now);
}

void UARTHost_SendEvent(const char *event, const char *detail)
{
    char line[160];
    if (detail) snprintf(line, sizeof(line), "{\"ev\":\"%s\",\"detail\":\"%s\"}\r\n", event, detail);
    else
    {
        minimap *node;
        uint16_t i;
        snprintf(line, sizeof(line), "{\"ev\":\"%s\",\"nodes\":%u}\r\n", event, MapCollent_GetCount());
        UARTHost_Write(line);
        for (i = 0; i < MapCollent_GetCount(); i++)
        {
            node = MapCollent_GetByIndex(i);
            snprintf(line, sizeof(line), "{\"node\":%u,\"x\":%d,\"y\":%d,\"type\":\"%s\"}\r\n",
                     node->id, node->x, node->y, MapCollent_GetTypeName(node->type));
            UARTHost_Write(line);
        }
        return;
    }
    UARTHost_Write(line);
}

void UARTHost_SendTelemetry(uint32_t now)
{
    Location_DataTypeDef *loc = Location_GetData();
    GPS_DataTypeDef *gps = GPS_GetData();
    char line[300];
    snprintf(line, sizeof(line),
        "{\"t\":%lu,\"x\":%.6f,\"y\":%.6f,\"h\":%.1f,\"v\":%.2f,\"mode\":\"%s\",\"bat\":%.2f,\"gps\":[%.6f,%.6f,%u],\"obs\":{\"front\":%u,\"left\":%u,\"right\":%u},\"basket\":%u,\"err\":%u}\r\n",
        (unsigned long)now, loc->longitude, loc->latitude, loc->heading, loc->speedKmh / 3.6f,
        Menu_GetModeName(), SensorADC_GetVoltage(ADC_CH_POWER1), gps->latitude, gps->longitude,
        loc->satellites, Lidar_GetMinDistance(0, 20), Lidar_GetMinDistance(90, 20),
        Lidar_GetMinDistance(270, 20), g_basket, Estop_GetSource());
    UARTHost_Write(line);
    s_status.txTelemetry++;
}

UARTHost_Status_TypeDef *UARTHost_GetStatus(void) { return &s_status; }

#ifndef _MAP_COLLENT_H
#define _MAP_COLLENT_H

#include "stm32f4xx.h"

#define MAP_NODE_W          64
#define MAP_NODE_H          64
#define MAP_GRID_CELL_CM    10
#define MAP_MAX_NODES       200
#define MAP_MAX_PATH        64
#define MAP_INVALID_ID      0xFFFF

typedef enum
{
    MAP_WAYPOINT = 0,
    MAP_JUNCTION,
    MAP_PARK,
    MAP_UNLOAD,
    MAP_INSPECT,
    MAP_TARGET
} MapNodeType_TypeDef;

typedef struct minimap
{
    char name[10];
    char sign[2];
    int16_t x;
    int16_t y;
    int16_t heading;
    uint16_t id;
    uint8_t type;
    uint8_t flags;
    struct minimap *S;
    struct minimap *N;
    struct minimap *W;
    struct minimap *E;
} minimap;

typedef struct
{
    uint16_t ids[MAP_MAX_PATH];
    uint16_t count;
    uint32_t lengthCm;
} MapPath_TypeDef;

void MapCollent_Init(void);
void MapCollent_Clear(void);
minimap *MapCollent_AddNode(int16_t x, int16_t y, int16_t heading,
                            uint8_t type, const char *name);
uint8_t MapCollent_DeleteNode(uint16_t id);
minimap *MapCollent_GetNode(uint16_t id);
minimap *MapCollent_GetByIndex(uint16_t index);
uint16_t MapCollent_GetCount(void);
uint8_t MapCollent_Connect(uint16_t from, uint16_t to, uint8_t bidirectional);
uint8_t MapCollent_Plan(uint16_t start, uint16_t goal, MapPath_TypeDef *path);
const char *MapCollent_GetTypeName(uint8_t type);

uint8_t *MapCollent_GetLocalGrid(void);
void MapCollent_ClearLocalGrid(void);
void MapCollent_SetGridCell(uint8_t x, uint8_t y, uint8_t occupancy);
uint8_t MapCollent_GetGridCell(uint8_t x, uint8_t y);

int map_write(minimap *map, char *path);
int map_read(minimap *map, char *path);
int map_write_SD(minimap *map, char *path);

#endif

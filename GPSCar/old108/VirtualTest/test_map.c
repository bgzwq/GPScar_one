#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "map_collent.h"
#include "ff.h"

static uint8_t fileData[32768];
static uint32_t fileSize;
static int failed;

#define CHECK(c) do { if (c) printf("PASS %s\n", #c); else { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failed++; } } while (0)
FRESULT f_open(FIL *f, const char *p, uint8_t mode) { (void)p; f->pos = 0; if ((mode & FA_CREATE_ALWAYS) && !(mode & FA_READ)) fileSize = 0; return FR_OK; }
FRESULT f_read(FIL *f, void *b, UINT n, UINT *r) { if (f->pos + n > fileSize) { *r = 0; return FR_DISK_ERR; } memcpy(b, fileData + f->pos, n); f->pos += n; *r = n; return FR_OK; }
FRESULT f_write(FIL *f, const void *b, UINT n, UINT *w) { if (f->pos + n > sizeof(fileData)) return FR_DISK_ERR; memcpy(fileData + f->pos, b, n); f->pos += n; if (f->pos > fileSize) fileSize = f->pos; *w = n; return FR_OK; }
FRESULT f_lseek(FIL *f, uint32_t p) { f->pos = p; return FR_OK; }
FRESULT f_close(FIL *f) { (void)f; return FR_OK; }
FRESULT f_mkdir(const char *p) { (void)p; return FR_OK; }

#include "../Mid/Src/map_collent.c"

static void makeMap(void)
{
    MapCollent_Clear();
    MapCollent_AddNode(0, 0, 0, MAP_PARK, "P0");
    MapCollent_AddNode(100, 0, 90, MAP_WAYPOINT, "W1");
    MapCollent_AddNode(100, 100, 180, MAP_TARGET, "T2");
    MapCollent_AddNode(0, 100, 0, MAP_UNLOAD, "U3");
    MapCollent_Connect(0, 1, 1);
    MapCollent_Connect(1, 2, 1);
    MapCollent_Connect(2, 3, 1);
}

int main(void)
{
    MapPath_TypeDef path;
    makeMap();
    CHECK(MapCollent_GetCount() == 4);
    CHECK(MapCollent_GetNode(0)->E == MapCollent_GetNode(1));
    CHECK(MapCollent_GetNode(1)->W == MapCollent_GetNode(0));
    CHECK(MapCollent_Plan(0, 2, &path));
    CHECK(path.count == 3 && path.ids[0] == 0 && path.ids[2] == 2);
    CHECK(path.lengthCm == 200);
    CHECK(!MapCollent_Plan(0, 99, &path));
    MapCollent_SetGridCell(63, 63, 200);
    MapCollent_SetGridCell(64, 0, 99);
    CHECK(MapCollent_GetGridCell(63, 63) == 200 && MapCollent_GetGridCell(64, 0) == 0);
    CHECK(map_write_SD(0, "MAP.BIN"));
    MapCollent_Clear();
    CHECK(map_read(0, "MAP.BIN"));
    CHECK(MapCollent_GetCount() == 4 && MapCollent_Plan(0, 2, &path) && path.lengthCm == 200);
    fileData[20] ^= 0x80;
    CHECK(!map_read(0, "MAP.BIN"));
    makeMap();
    CHECK(MapCollent_DeleteNode(1));
    CHECK(MapCollent_GetNode(1) == 0);
    CHECK(MapCollent_GetCount() == 3);
    CHECK(map_write_SD(0, "MAP.BIN"));
    CHECK(map_read(0, "MAP.BIN"));
    CHECK(MapCollent_AddNode(200, 200, 0, MAP_INSPECT, "NEW") != 0);
    CHECK(MapCollent_GetNode(1) == 0);
    printf("MAP RESULT: %s\n", failed ? "FAIL" : "PASS");
    return failed ? 1 : 0;
}

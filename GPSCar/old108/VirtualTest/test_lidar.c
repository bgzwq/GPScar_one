#include <stdio.h>
#include <string.h>
#include "lidar.h"
static int failed;
#define CHECK(c) do { if (c) printf("PASS %s\n", #c); else { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failed++; } } while (0)
#include "../Program/Source/lidar.c"

static uint8_t crc(const uint8_t *p, int n)
{
    uint8_t c = 0;
    int i;
    for (i = 0; i < n; i++) c = s_crcTable[c ^ p[i]];
    return c;
}
static void packet(uint16_t start, uint16_t end, uint16_t distance, int corrupt)
{
    uint8_t p[47] = {0};
    int i;
    p[0] = 0x54; p[1] = 12; p[2] = 0x58; p[3] = 0x02;
    p[4] = start & 255; p[5] = start >> 8;
    for (i = 0; i < 12; i++) { p[6 + i * 3] = distance & 255; p[7 + i * 3] = distance >> 8; p[8 + i * 3] = 80; }
    p[42] = end & 255; p[43] = end >> 8;
    p[46] = crc(p, 46);
    if (corrupt) p[10]++;
    for (i = 0; i < 47; i++) Lidar_Byte(p[i], 1000);
}
int main(void)
{
    packet(35900, 100, 850, 0);
    CHECK(Lidar_GetData()->packets == 1);
    CHECK(Lidar_GetData()->online == 1);
    CHECK(Lidar_GetData()->distanceMm[359] == 850);
    CHECK(Lidar_GetData()->distanceMm[0] == 850);
    CHECK(Lidar_GetMinDistance(0, 2) == 850);
    packet(9000, 10000, 1200, 1);
    CHECK(Lidar_GetData()->packets == 1);
    CHECK(Lidar_GetData()->crcErrors == 1);
    CHECK(Lidar_GetData()->distanceMm[95] == 0);
    printf("LIDAR RESULT: %s\n", failed ? "FAIL" : "PASS");
    return failed ? 1 : 0;
}

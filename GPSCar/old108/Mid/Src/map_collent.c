#include "map_collent.h"
#include "ff.h"
#include <string.h>
#include <math.h>

#define MAP_FILE_MAGIC  0x50414D43UL
#define MAP_FILE_VERSION 1

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t crc;
} MapFileHeader_TypeDef;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t heading;
    uint16_t id;
    uint8_t type;
    uint8_t flags;
    int16_t next[4];
    char name[10];
    char sign[2];
} MapDiskNode_TypeDef;

static minimap s_nodes[MAP_MAX_NODES];
static uint16_t s_count;
static uint16_t s_nextId;
static uint8_t s_localGrid[MAP_NODE_W][MAP_NODE_H];
minimap Map;

static uint32_t MapCollent_Crc32Byte(uint32_t crc, uint8_t value)
{
    uint8_t bit;
    crc ^= value;
    for (bit = 0; bit < 8; bit++)
        crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
    return crc;
}

static int32_t MapCollent_Distance2(const minimap *a, const minimap *b)
{
    int32_t dx = (int32_t)a->x - b->x;
    int32_t dy = (int32_t)a->y - b->y;
    return dx * dx + dy * dy;
}

static minimap **MapCollent_Link(minimap *node, uint8_t direction)
{
    if (direction == 0) return &node->N;
    if (direction == 1) return &node->E;
    if (direction == 2) return &node->S;
    return &node->W;
}

void MapCollent_Clear(void)
{
    memset(s_nodes, 0, sizeof(s_nodes));
    memset(&Map, 0, sizeof(Map));
    s_count = 0;
    s_nextId = 0;
    MapCollent_ClearLocalGrid();
}

void MapCollent_Init(void)
{
    MapCollent_Clear();
}

minimap *MapCollent_AddNode(int16_t x, int16_t y, int16_t heading,
                            uint8_t type, const char *name)
{
    minimap *node;
    if (s_count >= MAP_MAX_NODES) return 0;
    node = &s_nodes[s_count];
    memset(node, 0, sizeof(*node));
    node->x = x;
    node->y = y;
    node->heading = heading;
    node->id = s_nextId++;
    node->type = type;
    if (name)
    {
        strncpy(node->name, name, sizeof(node->name) - 1);
        node->sign[0] = name[0];
    }
    s_count++;
    return node;
}

minimap *MapCollent_GetNode(uint16_t id)
{
    uint16_t i;
    for (i = 0; i < s_count; i++)
        if (s_nodes[i].id == id) return &s_nodes[i];
    return 0;
}

minimap *MapCollent_GetByIndex(uint16_t index)
{
    return index < s_count ? &s_nodes[index] : 0;
}

uint16_t MapCollent_GetCount(void)
{
    return s_count;
}

uint8_t MapCollent_Connect(uint16_t from, uint16_t to, uint8_t bidirectional)
{
    static const int16_t dx[4] = { 0, 1, 0, -1 };
    static const int16_t dy[4] = { 1, 0, -1, 0 };
    minimap *a = MapCollent_GetNode(from);
    minimap *b = MapCollent_GetNode(to);
    uint8_t best = 0;
    uint8_t reverse = 2;
    uint8_t i;
    int32_t bestScore = 0x7FFFFFFF;
    if (!a || !b || a == b) return 0;
    for (i = 0; i < 4; i++)
    {
        int32_t vx = (int32_t)b->x - a->x;
        int32_t vy = (int32_t)b->y - a->y;
        int32_t score = dx[i] * vx + dy[i] * vy;
        if (score > 0 && MapCollent_Distance2(a, b) - score < bestScore)
        {
            bestScore = MapCollent_Distance2(a, b) - score;
            best = i;
            reverse = (uint8_t)((i + 2) & 3);
        }
    }
    *MapCollent_Link(a, best) = b;
    if (bidirectional) *MapCollent_Link(b, reverse) = a;
    return 1;
}

uint8_t MapCollent_DeleteNode(uint16_t id)
{
    static int16_t links[MAP_MAX_NODES][4];
    uint16_t i;
    uint16_t removed = MAP_INVALID_ID;
    uint8_t d;
    for (i = 0; i < s_count; i++) if (s_nodes[i].id == id) removed = i;
    if (removed == MAP_INVALID_ID) return 0;
    for (i = 0; i < s_count; i++)
        for (d = 0; d < 4; d++)
        {
            minimap *next = *MapCollent_Link(&s_nodes[i], d);
            links[i][d] = next && next->id != id ? (int16_t)next->id : -1;
        }
    memmove(&s_nodes[removed], &s_nodes[removed + 1], (s_count - removed - 1) * sizeof(minimap));
    memset(&s_nodes[s_count - 1], 0, sizeof(minimap));
    s_count--;
    for (i = 0; i < s_count; i++)
        for (d = 0; d < 4; d++)
            *MapCollent_Link(&s_nodes[i], d) = links[i][d] >= 0 ?
                MapCollent_GetNode((uint16_t)links[i][d]) : 0;
    return 1;
}

uint8_t MapCollent_Plan(uint16_t start, uint16_t goal, MapPath_TypeDef *path)
{
    uint32_t dist[MAP_MAX_NODES];
    int16_t prev[MAP_MAX_NODES];
    uint8_t used[MAP_MAX_NODES];
    uint16_t reverse[MAP_MAX_PATH];
    uint16_t startIndex = MAP_INVALID_ID, goalIndex = MAP_INVALID_ID;
    uint16_t i, n = 0;
    if (!path) return 0;
    path->count = 0;
    path->lengthCm = 0;
    for (i = 0; i < s_count; i++)
    {
        dist[i] = 0xFFFFFFFFUL;
        prev[i] = -1;
        used[i] = 0;
        if (s_nodes[i].id == start) startIndex = i;
        if (s_nodes[i].id == goal) goalIndex = i;
    }
    if (startIndex == MAP_INVALID_ID || goalIndex == MAP_INVALID_ID) return 0;
    dist[startIndex] = 0;
    for (n = 0; n < s_count; n++)
    {
        uint16_t u = MAP_INVALID_ID;
        uint8_t d;
        for (i = 0; i < s_count; i++)
            if (!used[i] && dist[i] != 0xFFFFFFFFUL &&
                (u == MAP_INVALID_ID || dist[i] < dist[u])) u = i;
        if (u == MAP_INVALID_ID || u == goalIndex) break;
        used[u] = 1;
        for (d = 0; d < 4; d++)
        {
            minimap *v = *MapCollent_Link(&s_nodes[u], d);
            uint16_t vi;
            uint32_t alt;
            if (!v) continue;
            for (vi = 0; vi < s_count; vi++) if (&s_nodes[vi] == v) break;
            if (vi == s_count || used[vi]) continue;
            alt = dist[u] + (uint32_t)(sqrtf((float)MapCollent_Distance2(&s_nodes[u], v)) + 0.5f);
            if (alt < dist[vi]) { dist[vi] = alt; prev[vi] = (int16_t)u; }
        }
    }
    if (dist[goalIndex] == 0xFFFFFFFFUL) return 0;
    n = 0;
    i = goalIndex;
    while (i != startIndex && n < MAP_MAX_PATH)
    {
        reverse[n++] = s_nodes[i].id;
        if (prev[i] < 0) return 0;
        i = (uint16_t)prev[i];
    }
    if (n >= MAP_MAX_PATH) return 0;
    reverse[n++] = start;
    for (i = 0; i < n; i++) path->ids[i] = reverse[n - 1 - i];
    path->count = n;
    for (i = 1; i < n; i++)
        path->lengthCm += (uint32_t)(sqrtf((float)MapCollent_Distance2(MapCollent_GetNode(path->ids[i - 1]),
                                                         MapCollent_GetNode(path->ids[i]))) + 0.5f);
    return 1;
}

const char *MapCollent_GetTypeName(uint8_t type)
{
    static const char *names[] = { "WAY", "JUNC", "PARK", "UNLOAD", "CHECK", "TARGET" };
    return type <= MAP_TARGET ? names[type] : "?";
}

uint8_t *MapCollent_GetLocalGrid(void) { return &s_localGrid[0][0]; }
void MapCollent_ClearLocalGrid(void) { memset(s_localGrid, 0, sizeof(s_localGrid)); }
void MapCollent_SetGridCell(uint8_t x, uint8_t y, uint8_t occupancy)
{
    if (x < MAP_NODE_W && y < MAP_NODE_H) s_localGrid[x][y] = occupancy;
}
uint8_t MapCollent_GetGridCell(uint8_t x, uint8_t y)
{
    return (x < MAP_NODE_W && y < MAP_NODE_H) ? s_localGrid[x][y] : 0;
}

int map_write(minimap *map, char *path)
{
    FIL file;
    UINT written;
    MapFileHeader_TypeDef header;
    uint16_t i;
    uint8_t d;
    uint32_t crc = 0xFFFFFFFFUL;
    (void)map;
    f_mkdir("0:/CARMAP");
    if (f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return 0;
    header.magic = MAP_FILE_MAGIC;
    header.version = MAP_FILE_VERSION;
    header.count = s_count;
    header.crc = 0;
    if (f_write(&file, &header, sizeof(header), &written) != FR_OK || written != sizeof(header))
    { f_close(&file); return 0; }
    for (i = 0; i < s_count; i++)
    {
        MapDiskNode_TypeDef disk;
        minimap *node = &s_nodes[i];
        memset(&disk, 0, sizeof(disk));
        disk.x = node->x; disk.y = node->y; disk.heading = node->heading;
        disk.id = node->id; disk.type = node->type; disk.flags = node->flags;
        memcpy(disk.name, node->name, sizeof(disk.name));
        memcpy(disk.sign, node->sign, sizeof(disk.sign));
        for (d = 0; d < 4; d++)
        {
            minimap *next = *MapCollent_Link(node, d);
            disk.next[d] = next ? (int16_t)next->id : -1;
        }
        for (written = 0; written < sizeof(disk); written++)
            crc = MapCollent_Crc32Byte(crc, ((uint8_t *)&disk)[written]);
        if (f_write(&file, &disk, sizeof(disk), &written) != FR_OK || written != sizeof(disk))
        { f_close(&file); return 0; }
    }
    header.crc = crc ^ 0xFFFFFFFFUL;
    f_lseek(&file, 0);
    if (f_write(&file, &header, sizeof(header), &written) != FR_OK || written != sizeof(header))
    { f_close(&file); return 0; }
    return f_close(&file) == FR_OK;
}

int map_read(minimap *map, char *path)
{
    FIL file;
    UINT read;
    MapFileHeader_TypeDef header;
    MapDiskNode_TypeDef disk;
    uint16_t i;
    uint8_t d;
    uint32_t crc = 0xFFFFFFFFUL;
    (void)map;
    if (f_open(&file, path, FA_READ) != FR_OK) return 0;
    if (f_read(&file, &header, sizeof(header), &read) != FR_OK || read != sizeof(header) ||
        header.magic != MAP_FILE_MAGIC || header.version != MAP_FILE_VERSION ||
        header.count > MAP_MAX_NODES)
    { f_close(&file); return 0; }
    MapCollent_Clear();
    for (i = 0; i < header.count; i++)
    {
        if (f_read(&file, &disk, sizeof(disk), &read) != FR_OK || read != sizeof(disk))
        { f_close(&file); MapCollent_Clear(); return 0; }
        for (read = 0; read < sizeof(disk); read++)
            crc = MapCollent_Crc32Byte(crc, ((uint8_t *)&disk)[read]);
        memcpy(&s_nodes[i].x, &disk.x, 10);
        memcpy(s_nodes[i].name, disk.name, sizeof(disk.name));
        memcpy(s_nodes[i].sign, disk.sign, sizeof(disk.sign));
        if (disk.id >= s_nextId) s_nextId = disk.id + 1;
    }
    s_count = header.count;
    f_close(&file);
    if ((crc ^ 0xFFFFFFFFUL) != header.crc) { MapCollent_Clear(); return 0; }
    if (f_open(&file, path, FA_READ) != FR_OK) { MapCollent_Clear(); return 0; }
    f_lseek(&file, sizeof(header));
    for (i = 0; i < s_count; i++)
    {
        if (f_read(&file, &disk, sizeof(disk), &read) != FR_OK || read != sizeof(disk))
        { f_close(&file); MapCollent_Clear(); return 0; }
        for (d = 0; d < 4; d++)
            if (disk.next[d] >= 0)
                *MapCollent_Link(&s_nodes[i], d) = MapCollent_GetNode((uint16_t)disk.next[d]);
    }
    f_close(&file);
    return 1;
}

int map_write_SD(minimap *map, char *path)
{
    return map_write(map, path);
}

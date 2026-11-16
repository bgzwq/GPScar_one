#ifndef FF_H
#define FF_H
#include <stdint.h>
typedef unsigned int UINT;
typedef struct { uint32_t pos; } FIL;
typedef enum { FR_OK = 0, FR_DISK_ERR, FR_NOT_READY } FRESULT;
#define FA_READ 1
#define FA_WRITE 2
#define FA_CREATE_ALWAYS 8
FRESULT f_open(FIL *, const char *, uint8_t);
FRESULT f_read(FIL *, void *, UINT, UINT *);
FRESULT f_write(FIL *, const void *, UINT, UINT *);
FRESULT f_lseek(FIL *, uint32_t);
FRESULT f_close(FIL *);
FRESULT f_mkdir(const char *);
#endif

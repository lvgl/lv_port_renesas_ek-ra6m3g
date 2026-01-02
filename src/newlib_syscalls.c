#include "bsp_api.h"
#include "hal_data.h"
#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>
#undef errno
extern int errno;

int _write(int file, char *ptr, int len);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _read(int file, char *ptr, int len);
int _lseek(int file, int ptr, int dir);

int _write(int file, char *ptr, int len)
{
    FSP_PARAMETER_NOT_USED(file);
    FSP_PARAMETER_NOT_USED(ptr);
    FSP_PARAMETER_NOT_USED(len);

    return 0;
}

int _close(int file)
{
  FSP_PARAMETER_NOT_USED(file);
  return -1;
}
int _fstat(int file, struct stat *st)
{
    FSP_PARAMETER_NOT_USED(file);
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
    FSP_PARAMETER_NOT_USED(file);
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
    FSP_PARAMETER_NOT_USED(file);
    FSP_PARAMETER_NOT_USED(ptr);
    FSP_PARAMETER_NOT_USED(dir);
  return 0;
}

int _read(int file, char *ptr, int len)
{
    FSP_PARAMETER_NOT_USED(file);
    FSP_PARAMETER_NOT_USED(ptr);
    FSP_PARAMETER_NOT_USED(len);
    return 0;
}

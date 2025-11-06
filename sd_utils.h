#ifndef SD_UTILS_H
#define SD_UTILS_H

#include "ff.h"
#include "pico/stdlib.h"

// Structure to manage the log file
typedef struct
{
    FIL file;
    bool is_open;
} sd_logger_t;

FRESULT sd_mount(void);
FRESULT sd_logger_open(sd_logger_t *logger, const char *filename);
FRESULT sd_logger_write(sd_logger_t *logger, const char *msg);
FRESULT sd_logger_flush(sd_logger_t *logger);
void sd_logger_close(sd_logger_t *logger);
bool sd_read_text(const char *path, char *buf, size_t buflen);

#endif

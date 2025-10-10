#include "sd_utils.h"
#include <stdio.h>
#include <string.h>

static FATFS fs; // File system object

// Provide dummy time for FatFs
DWORD get_fattime(void)
{
    return ((DWORD)(2025 - 1980) << 25) | ((DWORD)10 << 21) // October
           | ((DWORD)8 << 16)                               // Day
           | ((DWORD)15 << 11);                             // Hour
}

// Mount the SD card
FRESULT sd_mount(void)
{
    FRESULT fr;
    printf("[SD] Mounting SD card...\n");
    fr = f_mount(&fs, "", 1);
    if (fr == FR_OK)
        printf("[SD] SD card mounted successfully.\n");
    else
        printf("[SD] Mount failed: %d\n", fr);
    return fr;
}

// Open a file for logging
FRESULT sd_logger_open(sd_logger_t *logger, const char *filename)
{
    FRESULT fr = f_open(&logger->file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK)
    {
        logger->is_open = true;
        printf("[SD] File '%s' opened for writing.\n", filename);
    }
    else
    {
        logger->is_open = false;
        printf("[SD] Failed to open file '%s': %d\n", filename, fr);
    }
    return fr;
}

// Write a string message to file
FRESULT sd_logger_write(sd_logger_t *logger, const char *msg)
{
    if (!logger->is_open)
        return FR_NOT_READY;
    UINT bw;
    FRESULT fr = f_write(&logger->file, msg, strlen(msg), &bw);
    if (fr != FR_OK)
    {
        printf("[SD] Write error: %d\n", fr);
        return fr;
    }
    return FR_OK;
}

// Flush buffer to disk
FRESULT sd_logger_flush(sd_logger_t *logger)
{
    if (!logger->is_open)
        return FR_NOT_READY;
    return f_sync(&logger->file);
}

// Close file
void sd_logger_close(sd_logger_t *logger)
{
    if (logger->is_open)
    {
        f_close(&logger->file);
        logger->is_open = false;
        printf("[SD] File closed.\n");
    }
}

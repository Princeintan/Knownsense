// diskio.c  -- Pico FatFs low-level SPI driver (SD v1/v2 + SDSC/SDHC support)
// Tailored to MISO=16, MOSI=19, SCK=18, CS=17
#include "ff.h"
#include "diskio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>

// SPI pins (as you use)
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

#define SD_BLOCK_SIZE 512

// card addressing divisor (1 for SDHC, 512 for SDSC)
static uint32_t cdv = 1; // default assume block-addressing (SDHC). Will set to 512 for SDSC.

static inline void cs_select(void)
{
    asm volatile("nop; nop; nop;");
    gpio_put(PIN_CS, 0);
    asm volatile("nop; nop; nop;");
}

static inline void cs_deselect(void)
{
    asm volatile("nop; nop; nop;");
    gpio_put(PIN_CS, 1);
    asm volatile("nop; nop; nop;");
}

static uint8_t spi_xfer(uint8_t d)
{
    uint8_t r;
    spi_write_read_blocking(SPI_PORT, &d, &r, 1);
    return r;
}
static void spi_write_bytes(const uint8_t *buf, int len)
{
    spi_write_blocking(SPI_PORT, buf, len);
}
static void spi_read_bytes(uint8_t *buf, int len)
{
    spi_read_blocking(SPI_PORT, 0xFF, buf, len);
}
static void spi_dummy_clocks(int n)
{
    uint8_t ff = 0xFF;
    for (int i = 0; i < n; ++i)
        spi_write_blocking(SPI_PORT, &ff, 1);
}

// sd command send, returns R1 (or 0xFF on timeout)
static int sd_command_raw(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t cmdb[6];
    cmdb[0] = 0x40 | cmd;
    cmdb[1] = (arg >> 24) & 0xFF;
    cmdb[2] = (arg >> 16) & 0xFF;
    cmdb[3] = (arg >> 8) & 0xFF;
    cmdb[4] = arg & 0xFF;
    cmdb[5] = crc;

    cs_select();
    spi_write_bytes(cmdb, 6);

    // read until MSB==0 (response ready) or timeout
    for (int i = 0; i < 100; ++i)
    {
        uint8_t r = spi_xfer(0xFF);
        if (!(r & 0x80))
            return r;
    }
    return 0xFF;
}

static void sd_command_release(void)
{
    spi_xfer(0xFF);
    cs_deselect();
    spi_xfer(0xFF);
}

static int wait_token(uint8_t token, uint32_t timeout_ms)
{
    absolute_time_t dl = make_timeout_time_ms(timeout_ms);
    do
    {
        uint8_t b = spi_xfer(0xFF);
        if (b == token)
            return 1;
    } while (absolute_time_diff_us(get_absolute_time(), dl) > 0);
    return 0;
}

// helper: send command, read optional N bytes (final_count) after R1, then release CS
// if final_count < 0, it will keep CS selected and not read any extra (caller will continue)
static int sd_cmd_with_response(uint8_t cmd, uint32_t arg, uint8_t crc, int final_count, uint8_t *out)
{
    int r = sd_command_raw(cmd, arg, crc);
    if (r == 0xFF)
    {
        sd_command_release();
        return -1;
    }
    if (final_count > 0 && out)
    {
        for (int i = 0; i < final_count; ++i)
            out[i] = spi_xfer(0xFF);
    }
    else if (final_count > 0 && !out)
    {
        for (int i = 0; i < final_count; ++i)
            spi_xfer(0xFF);
    }
    // leave CS selected; caller should call sd_command_release() if desired
    return r;
}

/*------------------------------------------------------------------------*/
/* FatFs disk functions                                                   */
/*------------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv)
{
    printf("disk_initialize...\n");

    // SPI pins + start at 100 kHz (very safe init)
    spi_init(SPI_PORT, 100 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    sleep_ms(10);
    spi_dummy_clocks(16); // >=74 clock cycles (16*8 = 128)

    // Try CMD0 up to 5 times (some cards need a little nudging)
    int r = 0xFF;
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        r = sd_command_raw(0, 0, 0x95);
        sd_command_release();
        printf("CMD0 -> %02X (attempt %d)\n", r, attempt + 1);
        if (r == 0x01)
            break;
        sleep_ms(20);
    }
    if (r != 0x01)
    {
        printf("CMD0 failed\n");
        return STA_NOINIT;
    }

    // CMD8 to check voltage/SDv2 support (send 0x1AA)
    uint8_t resp4[4] = {0};
    r = sd_cmd_with_response(8, 0x1AA, 0x87, 4, resp4);
    sd_command_release();
    printf("CMD8 -> %02X  (resp: %02X %02X %02X %02X)\n", r, resp4[0], resp4[1], resp4[2], resp4[3]);

    if (r == 0x01)
    {
        // v2 card path
        // loop ACMD41 with HCS bit until ready
        for (int i = 0; i < 200; ++i)
        {
            // send APP_CMD (55)
            sd_cmd_with_response(55, 0, 0x65, 0, NULL);
            sd_command_release();
            // send ACMD41 with HCS
            r = sd_cmd_with_response(41, 0x40000000, 0x77, 0, NULL);
            sd_command_release();
            if (r == 0x00)
                break;
            sleep_ms(50);
        }
        if (r != 0x00)
        {
            printf("ACMD41 timeout (v2)\n");
            return STA_NOINIT;
        }
        // CMD58 to read OCR and detect SDHC/SDXC
        uint8_t ocr[4] = {0};
        r = sd_cmd_with_response(58, 0, 0xFD, 4, ocr);
        sd_command_release();
        printf("CMD58 -> %02X OCR: %02X %02X %02X %02X\n", r, ocr[0], ocr[1], ocr[2], ocr[3]);
        // Bit 30 (0x40 in first byte read here) indicates CCS (card capacity status) for SDHC/SDXC
        if (ocr[0] & 0x40)
        {
            cdv = 1; // block addressing (LBA)
        }
        else
        {
            cdv = 512; // byte addressing (SDSC)
        }
    }
    else if (r == (0x01 | 0x04) || r == 0x05 || r == 0x09 || r == 0x0D)
    {
        // Some cards return R1 with ILLEGAL_COMMAND bit set (means SD v1 or MMC)
        // Try v1 init: ACMD41 without HCS
        for (int i = 0; i < 200; ++i)
        {
            sd_cmd_with_response(55, 0, 0x65, 0, NULL);
            sd_command_release();
            r = sd_cmd_with_response(41, 0, 0x77, 0, NULL);
            sd_command_release();
            if (r == 0x00)
                break;
            sleep_ms(50);
        }
        if (r != 0x00)
        {
            printf("ACMD41 timeout (v1)\n");
            return STA_NOINIT;
        }
        // SDSC uses byte addressing
        cdv = 512;
    }
    else
    {
        // Unexpected response to CMD8 — try v1 path as fallback
        printf("CMD8 unexpected resp %02X, trying v1 init\n", r);
        for (int i = 0; i < 200; ++i)
        {
            sd_cmd_with_response(55, 0, 0x65, 0, NULL);
            sd_command_release();
            r = sd_cmd_with_response(41, 0, 0x77, 0, NULL);
            sd_command_release();
            if (r == 0x00)
                break;
            sleep_ms(50);
        }
        if (r != 0x00)
            return STA_NOINIT;
        cdv = 512;
    }

    // CMD16: ensure block length = 512 for SDSC (ignored by SDHC)
    if (cdv == 512)
    {
        r = sd_cmd_with_response(16, 512, 0x15, 0, NULL);
        sd_command_release();
        printf("CMD16 -> %02X\n", r);
        if (r != 0x00)
        {
            printf("CMD16 failed\n");
            return STA_NOINIT;
        }
    }

    // Optionally read CSD to get number of sectors — FatFs doesn't require it here.
    // Speed up SPI to 1 MHz (aligns with your MicroPython test)
    spi_init(SPI_PORT, 1000 * 1000);

    printf("disk_initialize OK (cdv=%lu)\n", (unsigned long)cdv);
    return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
    // Could add checks (CMD13) but keep simple
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (count == 0)
        return RES_PARERR;

    // convert sector -> address
    uint32_t addr = (uint32_t)sector * (uint32_t)cdv;

    if (count == 1)
    {
        int r = sd_cmd_with_response(17, addr, 0xFF, 0, NULL); // CMD17
        if (r != 0x00)
        {
            sd_command_release();
            printf("CMD17 R1=%02X\n", r);
            return RES_ERROR;
        }
        // wait for data token 0xFE
        if (!wait_token(0xFE, 200))
        {
            sd_command_release();
            printf("read token timeout\n");
            return RES_ERROR;
        }
        // read block
        spi_read_bytes(buff, SD_BLOCK_SIZE);
        // discard CRC
        spi_xfer(0xFF);
        spi_xfer(0xFF);
        sd_command_release();
        return RES_OK;
    }
    else
    {
        // multi block read (CMD18) then CMD12 to stop
        int r = sd_cmd_with_response(18, addr, 0xFF, 0, NULL);
        if (r != 0x00)
        {
            sd_command_release();
            printf("CMD18 R1=%02X\n", r);
            return RES_ERROR;
        }
        for (UINT i = 0; i < count; ++i)
        {
            if (!wait_token(0xFE, 200))
            {
                sd_command_release();
                return RES_ERROR;
            }
            spi_read_bytes(buff + i * SD_BLOCK_SIZE, SD_BLOCK_SIZE);
            spi_xfer(0xFF);
            spi_xfer(0xFF); // crc
        }
        // stop transmission
        sd_cmd_with_response(12, 0, 0xFF, 0, NULL); // may require skip1 on some cards; we just call
        sd_command_release();
        return RES_OK;
    }
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (count == 0)
        return RES_PARERR;
    uint32_t addr = (uint32_t)sector * (uint32_t)cdv;

    if (count == 1)
    {
        int r = sd_cmd_with_response(24, addr, 0xFF, 0, NULL); // CMD24
        if (r != 0x00)
        {
            sd_command_release();
            printf("CMD24 R1=%02X\n", r);
            return RES_ERROR;
        }
        // send start token
        spi_xfer(0xFE);
        spi_write_bytes(buff, SD_BLOCK_SIZE);
        // dummy CRC
        spi_xfer(0xFF);
        spi_xfer(0xFF);
        uint8_t resp = spi_xfer(0xFF);
        if ((resp & 0x1F) != 0x05)
        {
            sd_command_release();
            printf("data resp bad %02X\n", resp);
            return RES_ERROR;
        }
        // wait until not busy
        if (!wait_token(0xFF, 500))
        {
            sd_command_release();
            printf("write busy timeout\n");
            return RES_ERROR;
        }
        sd_command_release();
        return RES_OK;
    }
    else
    {
        // multi-block write (CMD25) - implement if needed
        return RES_PARERR;
    }
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    switch (cmd)
    {
    case GET_SECTOR_SIZE:
        *(WORD *)buff = SD_BLOCK_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        return RES_OK;
    case CTRL_SYNC:
        return RES_OK;
    }
    return RES_PARERR;
}

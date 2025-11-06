#include "ff.h"
#include "diskio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>

// SPI pins
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

#define SD_BLOCK_SIZE 512

// card addressing divisor (1 = SDHC/SDXC, 512 = SDSC)
static uint32_t cdv = 1;
static DSTATUS card_status = STA_NOINIT;
static uint32_t card_sectors = 0; // total number of sectors (capacity)

// -------------------- Helpers --------------------
static inline void cs_select(void) { gpio_put(PIN_CS, 0); }
static inline void cs_deselect(void) { gpio_put(PIN_CS, 1); }

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
static int sd_cmd_with_response(uint8_t cmd, uint32_t arg, uint8_t crc, int n, uint8_t *out)
{
	int r = sd_command_raw(cmd, arg, crc);
	if (r == 0xFF)
	{
		sd_command_release();
		return -1;
	}
	if (n > 0 && out)
		for (int i = 0; i < n; i++)
			out[i] = spi_xfer(0xFF);
	return r;
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

// -------------------- Disk Functions --------------------
DSTATUS disk_initialize(BYTE pdrv)
{
	printf("disk_initializing...\n");

	// SPI init
	spi_init(SPI_PORT, 100 * 1000); // safe 100 kHz for startup
	gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
	gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
	gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
	gpio_init(PIN_CS);
	gpio_set_dir(PIN_CS, GPIO_OUT);
	cs_deselect();

	sleep_ms(10);
	spi_dummy_clocks(16);

	// CMD0
	int r = 0xFF;
	for (int attempt = 0; attempt < 10; attempt++)
	{
		r = sd_command_raw(0, 0, 0x95);
		sd_command_release();
		printf("CMD0 -> %02X (attempt %d)\n", r, attempt + 1);
		if (r == 0x01)
			break;
		sleep_ms(20);
	}
	if (r != 0x01)
		return STA_NOINIT;

	// CMD8
	uint8_t resp4[4] = {0};
	r = sd_cmd_with_response(8, 0x1AA, 0x87, 4, resp4);
	sd_command_release();
	printf("CMD8 -> %02X (resp: %02X %02X %02X %02X)\n", r, resp4[0], resp4[1], resp4[2], resp4[3]);

	if (r == 0x01)
	{
		// v2 init (SDHC/SDXC)
		for (int i = 0; i < 200; i++)
		{
			sd_cmd_with_response(55, 0, 0x65, 0, NULL);
			sd_command_release();
			r = sd_cmd_with_response(41, 0x40000000, 0x77, 0, NULL);
			sd_command_release();
			if (r == 0x00)
				break;
			sleep_ms(50);
		}
		if (r != 0x00)
			return STA_NOINIT;
		uint8_t ocr[4];
		r = sd_cmd_with_response(58, 0, 0xFD, 4, ocr);
		sd_command_release();
		cdv = (ocr[0] & 0x40) ? 1 : 512;
	}
	else
	{
		// v1 init (SDSC)
		for (int i = 0; i < 200; i++)
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
		// CMD16: set 512-byte blocks
		r = sd_cmd_with_response(16, 512, 0x15, 0, NULL);
		sd_command_release();
		if (r != 0x00)
			return STA_NOINIT;
	}

	// CMD9: read CSD and calculate capacity
	uint8_t csd[16];
	r = sd_cmd_with_response(9, 0, 0xAF, 0, NULL); // CMD9
	if (r != 0x00)
	{
		sd_command_release();
		return STA_NOINIT;
	}
	if (!wait_token(0xFE, 200))
	{
		sd_command_release();
		return STA_NOINIT;
	}
	spi_read_bytes(csd, 16); // read CSD
	spi_xfer(0xFF);
	spi_xfer(0xFF); // CRC
	sd_command_release();

	if ((csd[0] >> 6) == 1)
	{
		// CSD v2.0
		uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
		card_sectors = (c_size + 1) * 1024; // number of 512-byte sectors
	}
	else
	{
		// CSD v1.0
		uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | ((csd[8] & 0xC0) >> 6);
		uint32_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7);
		uint32_t read_bl_len = csd[5] & 0x0F;
		uint32_t block_len = 1UL << read_bl_len;
		uint32_t mult = 1UL << (c_size_mult + 2);
		uint32_t blocknr = (c_size + 1) * mult;
		card_sectors = (blocknr * block_len) / 512;
	}

	// speed up SPI to 1 MHz
	spi_init(SPI_PORT, 300 * 1000);

	printf("disk_initialize OK, cdv=%u, sectors=%lu, capacity=%.2f MB\n",
		   cdv, card_sectors, (card_sectors / 2048.0));

	card_status = 0;
	return card_status;
}

DSTATUS disk_status(BYTE pdrv)
{
	return card_status;
}

// (disk_read and disk_write remain same as before)

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
	case GET_SECTOR_COUNT:
		*(DWORD *)buff = card_sectors;
		return RES_OK;
	}
	return RES_PARERR;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
	if (card_status & STA_NOINIT)
		return RES_NOTRDY;
	if (count == 0)
		return RES_PARERR;

	for (UINT i = 0; i < count; i++)
	{
		// CMD17 for single block
		int r = sd_command_raw(17, (sector + i) * cdv, 0xE1);
		if (r != 0x00)
		{
			sd_command_release();
			return RES_ERROR;
		}

		// wait for data token
		if (!wait_token(0xFE, 200))
		{
			sd_command_release();
			return RES_ERROR;
		}

		// read 512 bytes
		spi_read_bytes(buff + i * 512, 512);

		// discard CRC
		spi_xfer(0xFF);
		spi_xfer(0xFF);

		sd_command_release();
	}
	return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
	if (card_status & STA_NOINIT)
		return RES_NOTRDY;
	if (count == 0)
		return RES_PARERR;

	if (count == 1)
	{
		// ----- Single block write (CMD24) -----
		int r = sd_command_raw(24, sector * cdv, 0xE1);
		if (r != 0x00)
		{
			sd_command_release();
			return RES_ERROR;
		}

		spi_xfer(0xFE); // data token
		spi_write_bytes(buff, 512);
		spi_xfer(0xFF);
		spi_xfer(0xFF); // dummy CRC

		uint8_t resp = spi_xfer(0xFF);
		if ((resp & 0x1F) != 0x05)
		{
			sd_command_release();
			return RES_ERROR;
		}
		while (spi_xfer(0xFF) == 0x00)
			tight_loop_contents();
		sd_command_release();
	}
	else
	{
		// ----- Multi-block write (CMD25) -----
		int r = sd_command_raw(25, sector * cdv, 0xE1);
		if (r != 0x00)
		{
			sd_command_release();
			return RES_ERROR;
		}

		for (UINT i = 0; i < count; i++)
		{
			spi_xfer(0xFC); // multi-block token
			spi_write_bytes(buff + i * 512, 512);
			spi_xfer(0xFF);
			spi_xfer(0xFF); // dummy CRC

			uint8_t resp = spi_xfer(0xFF);
			if ((resp & 0x1F) != 0x05)
			{
				sd_command_release();
				return RES_ERROR;
			}
			while (spi_xfer(0xFF) == 0x00)
				tight_loop_contents();
		}

		// send stop token
		spi_xfer(0xFD);
		// wait until card finishes
		while (spi_xfer(0xFF) == 0x00)
			tight_loop_contents();
		sd_command_release();
	}

	return RES_OK;
}

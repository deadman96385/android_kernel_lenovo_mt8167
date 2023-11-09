/*
 * rt5680-spi.h  --  ALC5680 ALSA SoC audio codec driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5680_SPI_H__
#define __RT5680_SPI_H__

#define RT5680_SPI_BUF_LEN 240

/* SPI Command */
enum {
	RT5680_SPI_CMD_16_READ = 0,
	RT5680_SPI_CMD_16_WRITE,
	RT5680_SPI_CMD_32_READ,
	RT5680_SPI_CMD_32_WRITE,
	RT5680_SPI_CMD_BURST_READ,
	RT5680_SPI_CMD_BURST_WRITE,
	RT5680_SPI_CMD_CODEC_READ = 8,
	RT5680_SPI_CMD_CODEC_WRITE,
};

int rt5680_spi_read(u32 addr, unsigned int *val, size_t len);
int rt5680_spi_write(u32 addr, unsigned int val, size_t len);
int rt5680_spi_burst_read(u32 addr, u8 *rxbuf, size_t len);
int rt5680_spi_burst_write(u32 addr, const u8 *txbuf, size_t len);
int rt5680_spi_reg_read(unsigned int reg, unsigned int *val);
int rt5680_spi_reg_write(unsigned int reg, unsigned int val);

extern struct spi_device *rt5680_spi;

#endif /* __RT5680_SPI_H__ */

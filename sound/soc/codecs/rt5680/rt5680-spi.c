/*
 * rt5680-spi.c  --  ALC5680 ALSA SoC audio codec driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include "rt5680-spi.h"

struct spi_device *rt5680_spi;

int rt5680_spi_read(u32 addr, unsigned int *val, size_t len)
{
	struct spi_message message;
	struct spi_transfer x[1];
	int status;
	u8 write_buf[13];
	u8 read_buf[13];
	
	memset(write_buf, 0, sizeof(write_buf));
	memset(read_buf, 0, sizeof(read_buf));
	
	write_buf[0] =
		(len == 4) ? RT5680_SPI_CMD_32_READ : RT5680_SPI_CMD_16_READ;
	write_buf[1] = (addr & 0xff000000) >> 24;
	write_buf[2] = (addr & 0x00ff0000) >> 16;
	write_buf[3] = (addr & 0x0000ff00) >> 8;
	write_buf[4] = (addr & 0x000000ff) >> 0;
	
	spi_message_init(&message);
	memset(x, 0, sizeof(x));

	x[0].len = 9 + len;
	x[0].tx_buf = write_buf;
	x[0].rx_buf = read_buf;
	spi_message_add_tail(&x[0], &message);

	status = spi_sync(rt5680_spi, &message);

	if (len == 4)
		*val = read_buf[12] | read_buf[11] << 8 | read_buf[10] << 16 |
			read_buf[9] << 24;
	else
		*val = read_buf[10] | read_buf[9] << 8;

	dev_dbg(&rt5680_spi->dev, "%s:  %08x => %08x\n", __func__, addr, *val);
	return status;
}
EXPORT_SYMBOL_GPL(rt5680_spi_read);

int rt5680_spi_write(u32 addr, unsigned int val, size_t len)
{
	int status;
	u8 write_buf[10]; /* last byte is dummy */
	dev_dbg(&rt5680_spi->dev, "%s: %08x <= %08x\n", __func__, addr, val);

	write_buf[1] = (addr & 0xff000000) >> 24;
	write_buf[2] = (addr & 0x00ff0000) >> 16;
	write_buf[3] = (addr & 0x0000ff00) >> 8;
	write_buf[4] = (addr & 0x000000ff) >> 0;

	if (len == 4) {
		write_buf[0] = RT5680_SPI_CMD_32_WRITE;
		write_buf[5] = (val & 0xff000000) >> 24;
		write_buf[6] = (val & 0x00ff0000) >> 16;
		write_buf[7] = (val & 0x0000ff00) >> 8;
		write_buf[8] = (val & 0x000000ff) >> 0;
	} else {
		write_buf[0] = RT5680_SPI_CMD_16_WRITE;
		write_buf[5] = (val & 0x0000ff00) >> 8;
		write_buf[6] = (val & 0x000000ff) >> 0;
	}
	
	write_buf[9] = 0; /* dummy */

	if(!rt5680_spi)
		printk("RT5680 SPI DEV not intantiate !!\n");

	status = spi_write(rt5680_spi, write_buf,
		(len == 4) ? sizeof(write_buf) : sizeof(write_buf) - 2);
	
	printk("---%s :------> status = %d\n",__func__,status);
	
	if (status)
		dev_err(&rt5680_spi->dev, "%s error %d\n", __FUNCTION__, status);

	return status;
}
EXPORT_SYMBOL_GPL(rt5680_spi_write);

/**
 * rt5680_spi_burst_read - Read data from SPI by rt5680 dsp memory address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5680_spi_burst_read(u32 addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5680_SPI_CMD_BURST_READ;
	int status;
	u8 write_buf[9 + RT5680_SPI_BUF_LEN];
	u8 read_buf[9 + RT5680_SPI_BUF_LEN];
	unsigned int i, end, end2, offset = 0;

	struct spi_message message;
	struct spi_transfer x[1];
	
	memset(write_buf, 0, sizeof(write_buf));

	while (offset < len) {
		memset(read_buf, 0, sizeof(read_buf));

		if (offset + RT5680_SPI_BUF_LEN <= len)
			end = RT5680_SPI_BUF_LEN;
		else
			end = len % RT5680_SPI_BUF_LEN;
		
		end2 = (end % 8 == 0) ? end : (end - (end % 8) + 8);
		
		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 9 + end2;
		x[0].tx_buf = write_buf;
		x[0].rx_buf = read_buf;
		spi_message_add_tail(&x[0], &message);

		status = spi_sync(rt5680_spi, &message);
		if (status)
			return false;
		
		for (i = 0; i < end; i += 8) {
			if (i + 7 < end) {
				rxbuf[offset + i + 0] = read_buf[9 + i + 7];
				rxbuf[offset + i + 1] = read_buf[9 + i + 6];
				rxbuf[offset + i + 2] = read_buf[9 + i + 5];
				rxbuf[offset + i + 3] = read_buf[9 + i + 4];
				rxbuf[offset + i + 4] = read_buf[9 + i + 3];
				rxbuf[offset + i + 5] = read_buf[9 + i + 2];
				rxbuf[offset + i + 6] = read_buf[9 + i + 1];
				rxbuf[offset + i + 7] = read_buf[9 + i + 0];
			} else { //Avoid rxbuf overflow!
				if (i + 0 < end) rxbuf[offset + i + 0] = read_buf[9 + i + 7];
				if (i + 1 < end) rxbuf[offset + i + 1] = read_buf[9 + i + 6];
				if (i + 2 < end) rxbuf[offset + i + 2] = read_buf[9 + i + 5];
				if (i + 3 < end) rxbuf[offset + i + 3] = read_buf[9 + i + 4];
				if (i + 4 < end) rxbuf[offset + i + 4] = read_buf[9 + i + 3];
				if (i + 5 < end) rxbuf[offset + i + 5] = read_buf[9 + i + 2];
				if (i + 6 < end) rxbuf[offset + i + 6] = read_buf[9 + i + 1];
				if (i + 7 < end) rxbuf[offset + i + 7] = read_buf[9 + i + 0];
			}
		}

		offset += RT5680_SPI_BUF_LEN;
	}

	return true;
}
EXPORT_SYMBOL_GPL(rt5680_spi_burst_read);

/**
 * rt5680_spi_burst_write - Write data to SPI by rt5680 dsp memory address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5680_spi_burst_write(u32 addr, const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5680_SPI_CMD_BURST_WRITE;
	u8 *write_buf;
	unsigned int i, end, end2, offset = 0;
	int status;

	write_buf = kmalloc(RT5680_SPI_BUF_LEN + 6, GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	while (offset < len) {
		if (offset + RT5680_SPI_BUF_LEN <= len)
			end = RT5680_SPI_BUF_LEN;
		else
			end = len % RT5680_SPI_BUF_LEN;
		
		end2 = (end % 8 == 0) ? end : (end - (end % 8) + 8);

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			if (i + 7 < end) {
				write_buf[i + 12] = txbuf[offset + i + 0];
				write_buf[i + 11] = txbuf[offset + i + 1];
				write_buf[i + 10] = txbuf[offset + i + 2];
				write_buf[i +  9] = txbuf[offset + i + 3];
				write_buf[i +  8] = txbuf[offset + i + 4];
				write_buf[i +  7] = txbuf[offset + i + 5];
				write_buf[i +  6] = txbuf[offset + i + 6];
				write_buf[i +  5] = txbuf[offset + i + 7];
			} else { //Avoid txbuf overflow!
				write_buf[i + 12] = (i + 0 < end) ? txbuf[offset + i + 0] : 0xFF;
				write_buf[i + 11] = (i + 1 < end) ? txbuf[offset + i + 1] : 0xFF;
				write_buf[i + 10] = (i + 2 < end) ? txbuf[offset + i + 2] : 0xFF;
				write_buf[i +  9] = (i + 3 < end) ? txbuf[offset + i + 3] : 0xFF;
				write_buf[i +  8] = (i + 4 < end) ? txbuf[offset + i + 4] : 0xFF;
				write_buf[i +  7] = (i + 5 < end) ? txbuf[offset + i + 5] : 0xFF;
				write_buf[i +  6] = (i + 6 < end) ? txbuf[offset + i + 6] : 0xFF;
				write_buf[i +  5] = (i + 7 < end) ? txbuf[offset + i + 7] : 0xFF;
			}
		}

		write_buf[end2 + 5] = 0; /* dummy */

		status = spi_write(rt5680_spi, write_buf, end2 + 6);

		if (status) {
			dev_err(&rt5680_spi->dev, "%s error %d\n", __FUNCTION__,
				status);
			kfree(write_buf);
			return status;
		}

		offset += RT5680_SPI_BUF_LEN;
	}

	kfree(write_buf);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5680_spi_burst_write);

int rt5680_spi_reg_read(unsigned int reg, unsigned int *val)
{
	struct spi_message message;
	struct spi_transfer x[1];
	u8 spi_cmd = RT5680_SPI_CMD_CODEC_READ;
	int status;
	u8 write_buf[9];
	u8 read_buf[9];

	memset(write_buf, 0, sizeof(write_buf));
	memset(read_buf, 0, sizeof(read_buf));
	
	write_buf[0] = spi_cmd;
	write_buf[1] = (reg & 0x0000ff00) >> 8;
	write_buf[2] = (reg & 0x000000ff) >> 0;

	spi_message_init(&message);
	memset(x, 0, sizeof(x));

	x[0].len = 9;
	x[0].tx_buf = write_buf;
	x[0].rx_buf = read_buf;
	spi_message_add_tail(&x[0], &message);

	status = spi_sync(rt5680_spi, &message);

	*val = read_buf[8] | read_buf[7] << 8;

	return status;
}
EXPORT_SYMBOL_GPL(rt5680_spi_reg_read);

int rt5680_spi_reg_write(unsigned int reg, unsigned int val)
{
	u8 spi_cmd = RT5680_SPI_CMD_CODEC_WRITE;
	int status;
	u8 write_buf[6];

	write_buf[0] = spi_cmd;
	write_buf[1] = (reg & 0x0000ff00) >> 8;
	write_buf[2] = (reg & 0x000000ff) >> 0;
	write_buf[3] = (val & 0x0000ff00) >> 8;
	write_buf[4] = (val & 0x000000ff) >> 0;
	write_buf[5] = 0; /* dummy */

	status = spi_write(rt5680_spi, write_buf, sizeof(write_buf));

	if (status)
		dev_err(&rt5680_spi->dev, "%s error %d\n", __FUNCTION__, status);

	return status;
}
EXPORT_SYMBOL_GPL(rt5680_spi_reg_write);

static int rt5680_spi_probe(struct spi_device *spi)
{
	struct pinctrl * rt5680_pinctrl = NULL;
	struct pinctrl_state * rt5680_spi_state = NULL;

	printk("start--> %s\n", __func__);

	 rt5680_pinctrl = devm_pinctrl_get(&spi->dev);

	 if (IS_ERR(rt5680_pinctrl)) {
		printk("%s pinctrl_get failed  \n",__func__ );
	 }

	 rt5680_spi_state = pinctrl_lookup_state(rt5680_pinctrl, "default");
	 
	if(IS_ERR(rt5680_spi_state)) {
                printk("%s Can't find pinctrl state %s\n", __func__, "default");
        }

	 pinctrl_select_state(rt5680_pinctrl, rt5680_spi_state);
	
	 rt5680_spi = spi;
	 
	printk("end<-- %s\n", __func__);

	return 0;
}

static const struct of_device_id rt5680_of_match[] = {
	{ .compatible = "realtek,rt5680_spi"},
	{},
};
MODULE_DEVICE_TABLE(of, rt5680_of_match);

static struct spi_driver rt5680_spi_driver = {
	.driver = {
			.name = "rt5680",
			.of_match_table = rt5680_of_match,
	},
	.probe = rt5680_spi_probe,
};

static int __init rt5680_spi_init(void)
{
	printk("%s().\n", __func__);
	return spi_register_driver(&rt5680_spi_driver);
}

static void __exit rt5680_spi_exit(void)
{
	spi_unregister_driver(&rt5680_spi_driver);
}


module_init(rt5680_spi_init);
module_exit(rt5680_spi_exit);
//module_spi_driver(rt5680_spi_driver);

MODULE_DESCRIPTION("ALC5680 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");

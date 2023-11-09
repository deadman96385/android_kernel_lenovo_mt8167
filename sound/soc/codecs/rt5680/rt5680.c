/*
 * rt5680.c  --  ALC5680 ALSA SoC audio codec driver
 *
 * Copyright 2017 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "rt5680.h"
#include "rt5680-spi.h"

#define BYPASS_DSP

int reset_gpio;

#define ALC5680_i2C_MODE  1

struct rt5680_priv *g_rt5680;
struct proc_dir_entry *rt5680_dir = NULL;
unsigned int fw_load_done=0;
uint8_t init_done = 0;

struct rt5680_init_reg {
	u16 reg;
	u16 val;
	u16 delay; /* ms */
};

struct rt5680_init_memory {
	u32 reg;
	u32 val;
};

static struct rt5680_init_reg init_list[] = {
	{0x00FA, 0x0001, 0},
	{0x068A, 0x0250, 0},
	{0x01F0, 0x4000, 0},
	{0x0609, 0x1122, 0},
	{0x060A, 0x3622, 0},
	{0x060B, 0x1022, 0},
	{0x060C, 0x3622, 0},
	{0x0671, 0xC0D0, 0},
	{0x0603, 0x0444, 0},
	{0x068F, 0x0007, 0},
	{0x0690, 0x0007, 0},
	{0x0684, 0x0217, 0},
	{0x0122, 0x0000, 0},
	{0x0121, 0x0000, 0},
	{0x0014, 0x5454, 0},
	{0x0673, 0xAEAA, 0},
	{0x0660, 0x3840, 0},
	{0x0661, 0x3840, 0},
	{0x0665, 0x0101, 0},
	{0x0681, 0x0118, 0},
	{0x0682, 0x0118, 0},
	{0x07F3, 0x0008, 0},
	{0x061D, 0xE4CF, 0},
	{0x0049, 0x0000, 0},
	{0x0070, 0x8000, 0},
	{0x0072, 0x0000, 0},
	{0x0073, 0x8000, 0},
	{0x0170, 0x1100, 0},
	{0x0171, 0x1101, 0},
	{0x002B, 0x0002, 0},
  #if ALC5680_i2C_MODE
  	{0x0031, 0x0000, 0},
  #else
       {0x0031, 0x8000, 0},
  #endif
	{0x0038, 0x8FF0, 0},
	{0x003A, 0x9900, 0},
	{0x061A, 0x0044, 0},
	{0x061C, 0x0044, 0},
	{0x0063, 0xA27E, 0},
	{0x0066, 0xD682, 0},
	{0x0066, 0x201A, 0},
	{0x0063, 0xEF7E, 0},
	{0x0034, 0x0123, 0},
	{0x0035, 0x4567, 0},
	{0x0047, 0x0066, 0},
	{0x0016, 0xAFAF, 0},
	{0x0042, 0xC0AA, 0},
	{0x001C, 0x2F2F, 0},
	{0x0100, 0x1414, 0},
	{0x0101, 0x0000, 0},
	{0x068D, 0x0F07, 0},
	{0x0020, 0xA080, 0},
	{0x001A, 0x2F2F, 0},
	{0x0050, 0xC553, 0},
	{0x00C2, 0x5000, 0},
	{0x02D3, 0x003E, 0},
	{0x02C5, 0x803C, 0},
	{0x0501, 0x6000, 0},
	{0x0062, 0xBF03, 0},
	{0x0067, 0x1F0A, 0},
	{0x007A, 0x0302, 0},
	{0x007B, 0x0800, 0},
	{0x007B, 0x0802, 0},
	{0x007C, 0xCF00, 0},
	{0x007D, 0xF000, 0},
	{0x007D, 0xF002, 0},
	{0x0080, 0x2000, 0},
	{0x00C1, 0x8600, 0},
	{0x00C0, 0xFCD9, 0},
	{0x007E, 0x0111, 0},
	{0x007F, 0x0555, 0},
	{0x0068, 0x7A07, 0},
	{0x0069, 0x0330, 0},
	{0x0500, 0x3470, 0},
	{0x0502, 0x0000, 0},
	{0x0503, 0x2F2F, 0},
	{0x0504, 0x2F2F, 0},
	{0x0510, 0x9808, 0},
	{0x0513, 0x0810, 0},
	{0x0075, 0x0000, 0},
	{0x0046, 0x8080, 0},
	{0x0015, 0xAFAF, 0},
	{0x0014, 0x5757, 0},
#if ALC5680_i2C_MODE
	{0x004C, 0x4242, 0},
	{0x004B, 0x5050, 0},
	{0x0021, 0xA000, 0},
#else
	{0x004C, 0x0202, 0},
	{0x004B, 0x9090, 0},
	{0x0021, 0x0000, 0},
#endif
	{0x001B, 0x2F2F, 0},
	{0x0081, 0x0001, 0},
	{0x0520, 0x0070, 0},
	{0x0076, 0x1488, 0},
	{0x0077, 0xE777, 0},
	{0x0078, 0x0010, 0},
	{0x0032, 0x000C, 0},
	{0x0061, 0xF0C0, 0},
	{0x008A, 0x0200, 0},
	{0x008C, 0x2000, 0},
	{0x0083, 0x0007, 0},
	{0x0084, 0x7EFB, 0},
	{0x0085, 0x1022, 0},
	{0x0086, 0x2200, 0},
	{0x0087, 0x2220, 0},
	{0x0088, 0x1102, 0},
	{0x0089, 0x2200, 0},
	{0x004A, 0x4440, 0},
	{0x004D, 0x9088, 0},
	{0x004E, 0x9084, 0},
	{0x0022, 0x0000, 0},
	{0x001D, 0x2F2F, 0},
	{0x0040, 0xF0AA, 0},
	{0x0001, 0x5050, 0},
	{0x0662, 0x1010, 0},
	{0x0663, 0x1010, 0},
	{0x0610, 0xB090, 0},
	{0x0611, 0xB090, 0},
	{0x0008, 0x9C9C, 0},
	{0x0060, 0x0018, 0},
	{0x0064, 0x30D0, 0},
	{0x019B, 0x0003, 0},
	{0x0003, 0x8080, 0},
	{0x0065, 0x01FD, 0},
};
     #if ALC5680_i2C_MODE
     #else
static struct rt5680_init_memory memory_init_list[] = {
	{0x18008230, 0x0000FFFF},
	{0x18009128, 0x0000000F},
	{0x18000000, 0x60008000},
	{0x18000004, 0x600087FC},
	{0x18000010, 0x00000008},
	{0x18000030, 0x00000100},
	{0x18000100, 0x60009000},
	{0x18000104, 0x600097FC},
	{0x18000110, 0x00000018},
	{0x18000130, 0x00000100},
	{0x18004200, 0x60008000},
	{0x18004204, 0x600087FC},
	{0x18004210, 0x00000008},
	{0x18004230, 0x00000100},
	{0x18004300, 0x60009000},
	{0x18004304, 0x600097FC},
	{0x18004310, 0x00000018},
	{0x18004330, 0x00000100},
	{0x1800422C, 0x00000001},
	{0x1800432C, 0x00000001},
	{0x1800002C, 0x00000001},
	{0x1800012C, 0x00000001},
};
    #endif
/**
 * rt5680_dsp_mode_i2c_write_addr - Write value to address on DSP mode.
 * @rt5680: Private Data.
 * @addr: Address index.
 * @value: Address data.
 *
 *
 * Returns 0 for success or negative error code.
 */
static int rt5680_dsp_mode_i2c_write_addr(struct rt5680_priv *rt5680,
		unsigned int addr, unsigned int value, unsigned int opcode)
{

	int ret;

	mutex_lock(&rt5680->mutex);

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_ADDR_MSB,
		addr >> 16);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set addr msb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_ADDR_LSB,
		addr & 0xffff);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set addr lsb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_DATA_MSB,
		value >> 16);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set data msb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_DATA_LSB,
		value & 0xffff);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set data lsb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_OP_CODE,
		opcode);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set op code value: %d\n", ret);
		goto err;
	}

err:
	mutex_unlock(&rt5680->mutex);

	return ret;
}

/**
 * rt5680_dsp_mode_i2c_read_addr - Read value from address on DSP mode.
 * rt5680: Private Data.
 * @addr: Address index.
 * @value: Address data.
 *
 *
 * Returns 0 for success or negative error code.
 */
static int rt5680_dsp_mode_i2c_read_addr(
	struct rt5680_priv *rt5680, unsigned int addr, unsigned int *value)
{

	int ret;
	unsigned int msb, lsb;

	mutex_lock(&rt5680->mutex);

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_ADDR_MSB,
		addr >> 16);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set addr msb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_ADDR_LSB,
		addr & 0xffff);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set addr lsb value: %d\n", ret);
		goto err;
	}

	ret = regmap_write(rt5680->regmap_physical, RT5680_DSP_I2C_OP_CODE,
		0x0002);
	if (ret < 0) {
		dev_err(&rt5680->i2c->dev, "Failed to set op code value: %d\n", ret);
		goto err;
	}

	regmap_read(rt5680->regmap_physical, RT5680_DSP_I2C_DATA_MSB, &msb);
	regmap_read(rt5680->regmap_physical, RT5680_DSP_I2C_DATA_LSB, &lsb);
	*value = (msb << 16) | lsb;

err:
	mutex_unlock(&rt5680->mutex);

	return ret;
}

/**
 * rt5680_dsp_mode_i2c_write - Write register on DSP mode.
 * rt5680: Private Data.
 * @reg: Register index.
 * @value: Register data.
 *
 *
 * Returns 0 for success or negative error code.
 */
static int rt5680_dsp_mode_i2c_write(struct rt5680_priv *rt5680,
		unsigned int reg, unsigned int value)
{
	return rt5680_dsp_mode_i2c_write_addr(rt5680, 0x1800c000 + reg * 2,
		value << 16 | value, 0x1);
}

/**
 * rt5680_dsp_mode_i2c_read - Read register on DSP mode.
 * @codec: SoC audio codec device.
 * @reg: Register index.
 * @value: Register data.
 *
 *
 * Returns 0 for success or negative error code.
 */
static int rt5680_dsp_mode_i2c_read(
	struct rt5680_priv *rt5680, unsigned int reg, unsigned int *value)
{
	int ret = rt5680_dsp_mode_i2c_read_addr(rt5680, 0x1800c000 + reg * 2,
		value);

	*value &= 0xffff;

	return ret;
}

static int rt5680_read(unsigned int reg, unsigned int *val)
{
	struct rt5680_priv *rt5680 = g_rt5680;

	if (rt5680->is_dsp_mode) {
		if (reg < 0x800	)
			rt5680_dsp_mode_i2c_read(rt5680, reg, val);
		else
			rt5680_dsp_mode_i2c_read_addr(rt5680, reg, val);
	} else
		regmap_read(rt5680->regmap_physical, reg, val);

	return 0;
}

static int rt5680_write(unsigned int reg, unsigned int val)
{
	struct rt5680_priv *rt5680 = g_rt5680;

	if (rt5680->is_dsp_mode)
		rt5680_dsp_mode_i2c_write(rt5680, reg, val);
	else
		regmap_write(rt5680->regmap_physical, reg, val);

	return 0;
}

static void codec_init(void)
{
	int i;
	struct rt5680_priv *rt5680 = g_rt5680;

	for (i = 0; i < ARRAY_SIZE(init_list); i++) {
		rt5680_write(init_list[i].reg, init_list[i].val);
		msleep(init_list[i].delay);
	}
	rt5680->is_dsp_mode = true;
#ifdef BYPASS_DSP
	#if ALC5680_i2C_MODE
	#else
		for (i = 0; i < ARRAY_SIZE(memory_init_list); i++) {
			// rt5680_dsp_mode_i2c_write_addr(rt5680,
			// 	memory_init_list[i].reg, memory_init_list[i].val, 0x1);
		    	rt5680_spi_write(memory_init_list[i].reg, 
				memory_init_list[i].val, 0x4);
		}
      #endif
#endif
}

static bool rt5680_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RT5680_RESET:
	case RT5680_LOUT:
	case RT5680_HP_OUT:
	case RT5680_MONO_OUT:
	case RT5680_BST12_CTRL:
	case RT5680_BST34_CTRL:
	case RT5680_VAD_INBUF_CTRL:
	case RT5680_CAL_ADC_MIXER_CTRL:
	case RT5680_MICBIAS1_CTRL1:
	case RT5680_MICBIAS1_CTRL2:
	case RT5680_DAC1_POST_DIG_VOL:
	case RT5680_DAC1_DIG_VOL:
	case RT5680_DAC2_DIG_VOL:
	case RT5680_DAC3_DIG_VOL:
	case RT5680_STO1_ADC_DIG_VOL:
	case RT5680_MONO_ADC_DIG_VOL:
	case RT5680_STO2_ADC_DIG_VOL:
	case RT5680_STO3_ADC_DIG_VOL:
	case RT5680_ADC_BST_GAIN_CTRL1:
	case RT5680_ADC_BST_GAIN_CTRL2:
	case RT5680_ADC_BST_GAIN_CTRL3:
	case RT5680_SPDIF_IN_CTRL:
	case RT5680_IF3_DATA_CTRL:
	case RT5680_IF4_DATA_CTRL:
	case RT5680_IF5_DATA_CTRL:
	case RT5680_TDM1_CTRL1:
	case RT5680_TDM1_CTRL2:
	case RT5680_TDM1_CTRL3:
	case RT5680_TDM1_CTRL4:
	case RT5680_TDM1_CTRL5:
	case RT5680_TDM1_CTRL6:
	case RT5680_TDM1_CTRL7:
	case RT5680_TDM2_CTRL1:
	case RT5680_TDM2_CTRL2:
	case RT5680_TDM2_CTRL3:
	case RT5680_TDM2_CTRL4:
	case RT5680_TDM2_CTRL5:
	case RT5680_TDM2_CTRL6:
	case RT5680_TDM2_CTRL7:
	case RT5680_STO1_DAC_MIXER_CTRL1:
	case RT5680_STO1_DAC_MIXER_CTRL2:
	case RT5680_MONO_DAC_MIXER_CTRL1:
	case RT5680_MONO_DAC_MIXER_CTRL2:
	case RT5680_DD_MIXER_CTRL1:
	case RT5680_DD_MIXER_CTRL2:
	case RT5680_DAC1_MIXER_CTRL:
	case RT5680_DAC2_MIXER_CTRL:
	case RT5680_DAC3_MIXER_CTRL:
	case RT5680_DAC_SOURCE_CTRL:
	case RT5680_STO1_ADC_MIXER_CTRL:
	case RT5680_MONO_ADC_MIXER_CTRL1:
	case RT5680_MONO_ADC_MIXER_CTRL2:
	case RT5680_STO2_ADC_MIXER_CTRL:
	case RT5680_STO3_ADC_MIXER_CTRL:
	case RT5680_DMIC_CTRL1:
	case RT5680_DMIC_CTRL2:
	case RT5680_HPF_CTRL1:
	case RT5680_SV_ZCD_CTRL1:
	case RT5680_PWR_ADC:
	case RT5680_PWR_DIG1:
	case RT5680_PWR_DIG2:
	case RT5680_PWR_ANA1:
	case RT5680_PWR_ANA2:
	case RT5680_PWR_DSP:
	case RT5680_PWR_LDO1:
	case RT5680_PWR_LDO2:
	case RT5680_PWR_LDO3:
	case RT5680_PWR_LDO4:
	case RT5680_PWR_LDO5:
	case RT5680_I2S1_SDP:
	case RT5680_I2S2_SDP:
	case RT5680_I2S3_SDP:
	case RT5680_I2S4_SDP:
	case RT5680_I2S5_SDP:
	case RT5680_I2S_LRCK_BCLK_SOURCE:
	case RT5680_CLK_TREE_CTRL1:
	case RT5680_CLK_TREE_CTRL2:
	case RT5680_CLK_TREE_CTRL3:
	case RT5680_CLK_TREE_CTRL4:
	case RT5680_PLL1_CTRL1:
	case RT5680_PLL1_CTRL2:
	case RT5680_PLL2_CTRL1:
	case RT5680_PLL2_CTRL2:
	case RT5680_DSP_CLK_SOURCE1:
	case RT5680_DSP_CLK_SOURCE2:
	case RT5680_GLB_CLK1:
	case RT5680_GLB_CLK2:
	case RT5680_ASRC1:
	case RT5680_ASRC2:
	case RT5680_ASRC3:
	case RT5680_ASRC4:
	case RT5680_ASRC5:
	case RT5680_ASRC6:
	case RT5680_ASRC7:
	case RT5680_ASRC8:
	case RT5680_ASRC9:
	case RT5680_ASRC10:
	case RT5680_ASRC11:
	case RT5680_ASRC12:
	case RT5680_ASRC13:
	case RT5680_ASRC14:
	case RT5680_ASRC15:
	case RT5680_ASRC16:
	case RT5680_ASRC17:
	case RT5680_ASRC18:
	case RT5680_ASRC19:
	case RT5680_ASRC20:
	case RT5680_ASRC21:
	case RT5680_ASRC22:
	case RT5680_ASRC23:
	case RT5680_ASRC24:
	case RT5680_ASRC25:
	case RT5680_FRAC_DIV_CTRL1:
	case RT5680_FRAC_DIV_CTRL2:
	case RT5680_JACK_MIC_DET_CTRL1:
	case RT5680_JACK_MIC_DET_CTRL2:
	case RT5680_JACK_MIC_DET_CTRL3:
	case RT5680_JACK_MIC_DET_CTRL4:
	case RT5680_JACK_DET_CTRL1:
	case RT5680_JACK_DET_CTRL2:
	case RT5680_JACK_DET_CTRL3:
	case RT5680_JACK_DET_CTRL4:
	case RT5680_JACK_DET_CTRL5:
	case RT5680_IRQ_ST1:
	case RT5680_IRQ_ST2:
	case RT5680_IRQ_CTRL1:
	case RT5680_IRQ_CTRL2:
	case RT5680_IRQ_CTRL3:
	case RT5680_IRQ_CTRL4:
	case RT5680_IRQ_CTRL5:
	case RT5680_IRQ_CTRL6:
	case RT5680_IRQ_CTRL7:
	case RT5680_IRQ_CTRL8:
	case RT5680_IRQ_CTRL9:
	case RT5680_MF_PIN_CTRL1:
	case RT5680_MF_PIN_CTRL2:
	case RT5680_MF_PIN_CTRL3:
	case RT5680_GPIO_CTRL1:
	case RT5680_GPIO_CTRL2:
	case RT5680_GPIO_CTRL3:
	case RT5680_GPIO_CTRL4:
	case RT5680_GPIO_CTRL5:
	case RT5680_GPIO_CTRL6:
	case RT5680_GPIO_ST1:
	case RT5680_GPIO_ST2:
	case RT5680_LP_DET_CTRL:
	case RT5680_STO1_ADC_HPF_CTRL1:
	case RT5680_STO1_ADC_HPF_CTRL2:
	case RT5680_MONO_ADC_HPF_CTRL1:
	case RT5680_MONO_ADC_HPF_CTRL2:
	case RT5680_STO2_ADC_HPF_CTRL1:
	case RT5680_STO2_ADC_HPF_CTRL2:
	case RT5680_STO3_ADC_HPF_CTRL1:
	case RT5680_STO3_ADC_HPF_CTRL2:
	case RT5680_ZCD_CTRL:
	case RT5680_IL_CMD1:
	case RT5680_IL_CMD2:
	case RT5680_IL_CMD3:
	case RT5680_IL_CMD4:
	case RT5680_4BTN_IL_CMD1:
	case RT5680_4BTN_IL_CMD2:
	case RT5680_4BTN_IL_CMD3:
	case RT5680_PS_IL_CMD1:
	case RT5680_DSP_OUTB_0123_MIXER_CTRL:
	case RT5680_DSP_OUTB_45_MIXER_CTRL:
	case RT5680_DSP_OUTB_67_MIXER_CTRL:
	case RT5680_MCLK_GATING_CTRL:
	case RT5680_VENDOR_ID:
	case RT5680_VENDOR_ID1:
	case RT5680_VENDOR_ID2:
	case RT5680_PDM_OUTPUT_CTRL:
	case RT5680_PDM1_CTRL1:
	case RT5680_PDM1_CTRL2:
	case RT5680_PDM1_CTRL3:
	case RT5680_PDM1_CTRL4:
	case RT5680_PDM1_CTRL5:
	case RT5680_PDM2_CTRL1:
	case RT5680_PDM2_CTRL2:
	case RT5680_PDM2_CTRL3:
	case RT5680_PDM2_CTRL4:
	case RT5680_PDM2_CTRL5:
	case RT5680_STO_DAC_POST_VOL_CTRL:
	case RT5680_ST_CTRL:
	case RT5680_MCLK_DET_PROTECT_CTRL:
	case RT5680_STO_HP_NG2_CTRL1:
	case RT5680_STO_HP_NG2_CTRL2:
	case RT5680_STO_HP_NG2_CTRL3:
	case RT5680_STO_HP_NG2_CTRL4:
	case RT5680_STO_HP_NG2_CTRL5:
	case RT5680_STO_HP_NG2_CTRL6:
	case RT5680_STO_HP_NG2_ST1:
	case RT5680_STO_HP_NG2_ST2:
	case RT5680_STO_HP_NG2_ST3:
	case RT5680_NG2_ENV_DITHER_CTRL:
	case RT5680_MONO_AMP_NG2_CTRL1:
	case RT5680_MONO_AMP_NG2_CTRL2:
	case RT5680_MONO_AMP_NG2_CTRL3:
	case RT5680_MONO_AMP_NG2_CTRL4:
	case RT5680_MONO_AMP_NG2_CTRL5:
	case RT5680_MONO_AMP_NG2_ST1:
	case RT5680_MONO_AMP_NG2_ST2:
	case RT5680_IF_INPUT_DET_ST1:
	case RT5680_IF_INPUT_DET_ST2:
	case RT5680_IF_INPUT_DET_ST3:
	case RT5680_STO_DAC_SIL_DET_CTRL:
	case RT5680_MONO_DACL_SIL_DET_CTRL:
	case RT5680_MONO_DACR_SIL_DET_CTRL:
	case RT5680_DD_MIXERL_SIL_DET_CTRL:
	case RT5680_DD_MIXERR_SIL_DET_CTRL:
	case RT5680_SIL_DET_CTRLOUTPUT1:
	case RT5680_SIL_DET_CTRLOUTPUT2:
	case RT5680_SIL_DET_CTRLOUTPUT3:
	case RT5680_SIL_DET_CTRLOUTPUT4:
	case RT5680_SIL_DET_CTRLOUTPUT5:
	case RT5680_SIL_DET_CTRLOUTPUT6:
	case RT5680_SIL_DET_CTRLOUTPUT7:
	case RT5680_SIL_DET_CTRLOUTPUT8:
	case RT5680_ADC_EQ_CTRL1:
	case RT5680_ADC_EQ_CTRL2:
	case RT5680_DAC_EQ_CTRL1:
	case RT5680_DAC_EQ_CTRL2:
	case RT5680_DAC_EQ_CTRL3:
	case RT5680_DAC_EQ_CTRL4:
	case RT5680_I2S_MASTER_CLK_CTRL1:
	case RT5680_I2S_MASTER_CLK_CTRL2:
	case RT5680_I2S_MASTER_CLK_CTRL3:
	case RT5680_I2S_MASTER_CLK_CTRL4:
	case RT5680_I2S_MASTER_CLK_CTRL5:
	case RT5680_I2S_MASTER_CLK_CTRL6:
	case RT5680_I2S_MASTER_CLK_CTRL7:
	case RT5680_I2S_MASTER_CLK_CTRL8:
	case RT5680_I2S_MASTER_CLK_CTRL9:
	case RT5680_I2S_MASTER_CLK_CTRL10:
	case RT5680_I2S_MASTER_CLK_CTRL11:
	case RT5680_I2S_MASTER_CLK_CTRL12:
	case RT5680_I2S_MASTER_CLK_CTRL13:
	case RT5680_HP_DECR_DECOUP_CTRL1:
	case RT5680_HP_DECR_DECOUP_CTRL2:
	case RT5680_HP_DECR_DECOUP_CTRL3:
	case RT5680_HP_DECR_DECOUP_CTRL4:
	case RT5680_VAD_ADC_FILTER_CTRL1:
	case RT5680_VAD_ADC_FILTER_CTRL2:
	case RT5680_VAD_CLK_SETTING1:
	case RT5680_VAD_CLK_SETTING2:
	case RT5680_VAD_ADC_PLL3_CTRL1:
	case RT5680_VAD_ADC_PLL3_CTRL2:
	case RT5680_HP_BL_CTRL1:
	case RT5680_HP_BL_CTRL2:
	case RT5680_HP_IMP_SENS_CTRL1:
	case RT5680_HP_IMP_SENS_CTRL2:
	case RT5680_HP_IMP_SENS_CTRL3:
	case RT5680_HP_IMP_SENS_CTRL4:
	case RT5680_HP_IMP_SENS_CTRL5:
	case RT5680_HP_IMP_SENS_CTRL6:
	case RT5680_HP_IMP_SENS_CTRL7:
	case RT5680_HP_IMP_SENS_CTRL8:
	case RT5680_HP_IMP_SENS_CTRL9:
	case RT5680_HP_IMP_SENS_CTRL10:
	case RT5680_HP_IMP_SENS_CTRL11:
	case RT5680_HP_IMP_SENS_CTRL12:
	case RT5680_HP_IMP_SENS_CTRL13:
	case RT5680_HP_IMP_SENS_CTRL14:
	case RT5680_HP_IMP_SENS_CTRL15:
	case RT5680_HP_IMP_SENS_CTRL16:
	case RT5680_HP_IMP_SENS_CTRL17:
	case RT5680_HP_IMP_SENS_CTRL18:
	case RT5680_HP_IMP_SENS_DIG_CTRL1:
	case RT5680_HP_IMP_SENS_DIG_CTRL2:
	case RT5680_HP_IMP_SENS_DIG_CTRL3:
	case RT5680_HP_IMP_SENS_DIG_CTRL4:
	case RT5680_HP_IMP_SENS_DIG_CTRL5:
	case RT5680_HP_IMP_SENS_DIG_CTRL6:
	case RT5680_HP_IMP_SENS_DIG_CTRL7:
	case RT5680_HP_IMP_SENS_DIG_CTRL8:
	case RT5680_HP_IMP_SENS_DIG_CTRL9:
	case RT5680_HP_IMP_SENS_DIG_CTRL10:
	case RT5680_HP_IMP_SENS_DIG_CTRL11:
	case RT5680_HP_IMP_SENS_DIG_CTRL12:
	case RT5680_HP_IMP_SENS_DIG_CTRL13:
	case RT5680_HP_IMP_SENS_DIG_CTRL14:
	case RT5680_HP_IMP_SENS_DIG_CTRL15:
	case RT5680_HP_IMP_SENS_DIG_CTRL16:
	case RT5680_HP_IMP_SENS_DIG_CTRL17:
	case RT5680_ALC_PGA_CTRL1:
	case RT5680_ALC_PGA_CTRL2:
	case RT5680_ALC_PGA_CTRL3:
	case RT5680_ALC_PGA_CTRL4:
	case RT5680_ALC_PGA_CTRL5:
	case RT5680_ALC_PGA_CTRL6:
	case RT5680_ALC_PGA_CTRL7:
	case RT5680_ALC_PGA_ST1:
	case RT5680_ALC_PGA_ST2:
	case RT5680_ALC_PGA_ST3:
	case RT5680_ALC_PGA_SOURCE_CTRL1:
	case RT5680_HAPTIC_GEN_CTRL1:
	case RT5680_HAPTIC_GEN_CTRL2:
	case RT5680_HAPTIC_GEN_CTRL3:
	case RT5680_HAPTIC_GEN_CTRL4:
	case RT5680_HAPTIC_GEN_CTRL5:
	case RT5680_HAPTIC_GEN_CTRL6:
	case RT5680_HAPTIC_GEN_CTRL7:
	case RT5680_HAPTIC_GEN_CTRL8:
	case RT5680_HAPTIC_GEN_CTRL9:
	case RT5680_HAPTIC_GEN_CTRL10:
	case RT5680_AUTO_RC_CLK_CTRL1:
	case RT5680_AUTO_RC_CLK_CTRL2:
	case RT5680_AUTO_RC_CLK_CTRL3:
	case RT5680_DAC_L_EQ_LPF1_A1:
	case RT5680_DAC_L_EQ_LPF1_H0:
	case RT5680_DAC_R_EQ_LPF1_A1:
	case RT5680_DAC_R_EQ_LPF1_H0:
	case RT5680_DAC_L_EQ_LPF2_A1:
	case RT5680_DAC_L_EQ_LPF2_H0:
	case RT5680_DAC_R_EQ_LPF2_A1:
	case RT5680_DAC_R_EQ_LPF2_H0:
	case RT5680_DAC_L_EQ_BPF1_A1:
	case RT5680_DAC_L_EQ_BPF1_A2:
	case RT5680_DAC_L_EQ_BPF1_H0:
	case RT5680_DAC_R_EQ_BPF1_A1:
	case RT5680_DAC_R_EQ_BPF1_A2:
	case RT5680_DAC_R_EQ_BPF1_H0:
	case RT5680_DAC_L_EQ_BPF2_A1:
	case RT5680_DAC_L_EQ_BPF2_A2:
	case RT5680_DAC_L_EQ_BPF2_H0:
	case RT5680_DAC_R_EQ_BPF2_A1:
	case RT5680_DAC_R_EQ_BPF2_A2:
	case RT5680_DAC_R_EQ_BPF2_H0:
	case RT5680_DAC_L_EQ_BPF3_A1:
	case RT5680_DAC_L_EQ_BPF3_A2:
	case RT5680_DAC_L_EQ_BPF3_H0:
	case RT5680_DAC_R_EQ_BPF3_A1:
	case RT5680_DAC_R_EQ_BPF3_A2:
	case RT5680_DAC_R_EQ_BPF3_H0:
	case RT5680_DAC_L_EQ_BPF4_A1:
	case RT5680_DAC_L_EQ_BPF4_A2:
	case RT5680_DAC_L_EQ_BPF4_H0:
	case RT5680_DAC_R_EQ_BPF4_A1:
	case RT5680_DAC_R_EQ_BPF4_A2:
	case RT5680_DAC_R_EQ_BPF4_H0:
	case RT5680_DAC_L_EQ_BPF5_A1:
	case RT5680_DAC_L_EQ_BPF5_A2:
	case RT5680_DAC_L_EQ_BPF5_H0:
	case RT5680_DAC_R_EQ_BPF5_A1:
	case RT5680_DAC_R_EQ_BPF5_A2:
	case RT5680_DAC_R_EQ_BPF5_H0:
	case RT5680_DAC_L_EQ_HPF1_A1:
	case RT5680_DAC_L_EQ_HPF1_H0:
	case RT5680_DAC_R_EQ_HPF1_A1:
	case RT5680_DAC_R_EQ_HPF1_H0:
	case RT5680_DAC_L_EQ_HPF2_A1:
	case RT5680_DAC_L_EQ_HPF2_A2:
	case RT5680_DAC_L_EQ_HPF2_H0:
	case RT5680_DAC_R_EQ_HPF2_A1:
	case RT5680_DAC_R_EQ_HPF2_A2:
	case RT5680_DAC_R_EQ_HPF2_H0:
	case RT5680_DAC_L_EQ_HPF3_A1:
	case RT5680_DAC_L_EQ_HPF3_H0:
	case RT5680_DAC_R_EQ_HPF3_A1:
	case RT5680_DAC_R_EQ_HPF3_H0:
	case RT5680_DAC_L_BI_EQ_H0_1:
	case RT5680_DAC_L_BI_EQ_H0_2:
	case RT5680_DAC_L_BI_EQ_B1_1:
	case RT5680_DAC_L_BI_EQ_B1_2:
	case RT5680_DAC_L_BI_EQ_B2_1:
	case RT5680_DAC_L_BI_EQ_B2_2:
	case RT5680_DAC_L_BI_EQ_A1_1:
	case RT5680_DAC_L_BI_EQ_A1_2:
	case RT5680_DAC_L_BI_EQ_A2_1:
	case RT5680_DAC_L_BI_EQ_A2_2:
	case RT5680_DAC_R_BI_EQ_H0_1:
	case RT5680_DAC_R_BI_EQ_H0_2:
	case RT5680_DAC_R_BI_EQ_B1_1:
	case RT5680_DAC_R_BI_EQ_B1_2:
	case RT5680_DAC_R_BI_EQ_B2_1:
	case RT5680_DAC_R_BI_EQ_B2_2:
	case RT5680_DAC_R_BI_EQ_A1_1:
	case RT5680_DAC_R_BI_EQ_A1_2:
	case RT5680_DAC_R_BI_EQ_A2_1:
	case RT5680_DAC_R_BI_EQ_A2_2:
	case RT5680_DAC_L_EQ_PRE_VOL_CTRL:
	case RT5680_DAC_R_EQ_PRE_VOL_CTRL:
	case RT5680_DAC_L_EQ_POST_VOL_CTRL:
	case RT5680_DAC_R_EQ_POST_VOL_CTRL:
	case RT5680_ADC_L_EQ_LPF_A1:
	case RT5680_ADC_L_EQ_LPF_H0:
	case RT5680_ADC_R_EQ_LPF_A1:
	case RT5680_ADC_R_EQ_LPF_H0:
	case RT5680_ADC_L_EQ_BPF1_A1:
	case RT5680_ADC_L_EQ_BPF1_A2:
	case RT5680_ADC_L_EQ_BPF1_H0:
	case RT5680_ADC_R_EQ_BPF1_A1:
	case RT5680_ADC_R_EQ_BPF1_A2:
	case RT5680_ADC_R_EQ_BPF1_H0:
	case RT5680_ADC_L_EQ_BPF2_A1:
	case RT5680_ADC_L_EQ_BPF2_A2:
	case RT5680_ADC_L_EQ_BPF2_H0:
	case RT5680_ADC_R_EQ_BPF2_A1:
	case RT5680_ADC_R_EQ_BPF2_A2:
	case RT5680_ADC_R_EQ_BPF2_H0:
	case RT5680_ADC_L_EQ_BPF3_A1:
	case RT5680_ADC_L_EQ_BPF3_A2:
	case RT5680_ADC_L_EQ_BPF3_H0:
	case RT5680_ADC_R_EQ_BPF3_A1:
	case RT5680_ADC_R_EQ_BPF3_A2:
	case RT5680_ADC_R_EQ_BPF3_H0:
	case RT5680_ADC_L_EQ_BPF4_A1:
	case RT5680_ADC_L_EQ_BPF4_A2:
	case RT5680_ADC_L_EQ_BPF4_H0:
	case RT5680_ADC_R_EQ_BPF4_A1:
	case RT5680_ADC_R_EQ_BPF4_A2:
	case RT5680_ADC_R_EQ_BPF4_H0:
	case RT5680_ADC_L_EQ_HPF1_A1:
	case RT5680_ADC_L_EQ_HPF1_H0:
	case RT5680_ADC_R_EQ_HPF1_A1:
	case RT5680_ADC_R_EQ_HPF1_H0:
	case RT5680_ADC_L_EQ_PRE_VOL_CTRL:
	case RT5680_ADC_R_EQ_PRE_VOL_CTRL:
	case RT5680_ADC_L_EQ_POST_VOL_CTRL:
	case RT5680_ADC_R_EQ_POST_VOL_CTRL:
	case RT5680_PITCH_HELLO_DET_CTRL1:
	case RT5680_PITCH_HELLO_DET_CTRL2:
	case RT5680_PITCH_HELLO_DET_CTRL3:
	case RT5680_PITCH_HELLO_DET_CTRL4:
	case RT5680_PITCH_HELLO_DET_CTRL5:
	case RT5680_PITCH_HELLO_DET_CTRL6:
	case RT5680_PITCH_HELLO_DET_CTRL7:
	case RT5680_PITCH_HELLO_DET_CTRL8:
	case RT5680_PITCH_HELLO_DET_CTRL9:
	case RT5680_PITCH_HELLO_DET_CTRL10:
	case RT5680_PITCH_HELLO_DET_CTRL11:
	case RT5680_PITCH_HELLO_DET_CTRL12:
	case RT5680_PITCH_HELLO_DET_CTRL13:
	case RT5680_PITCH_HELLO_DET_CTRL14:
	case RT5680_PITCH_HELLO_DET_CTRL15:
	case RT5680_PITCH_HELLO_DET_CTRL16:
	case RT5680_PITCH_HELLO_DET_CTRL17:
	case RT5680_PITCH_HELLO_DET_CTRL18:
	case RT5680_PITCH_HELLO_DET_CTRL19:
	case RT5680_PITCH_HELLO_DET_CTRL20:
	case RT5680_PITCH_HELLO_DET_CTRL21:
	case RT5680_PITCH_HELLO_DET_CTRL22:
	case RT5680_PITCH_HELLO_DET_CTRL23:
	case RT5680_OK_DET_CTRL1:
	case RT5680_OK_DET_CTRL2:
	case RT5680_OK_DET_CTRL3:
	case RT5680_OK_DET_CTRL4:
	case RT5680_OK_DET_CTRL5:
	case RT5680_OK_DET_CTRL6:
	case RT5680_OK_DET_CTRL7:
	case RT5680_OK_DET_CTRL8:
	case RT5680_OK_DET_CTRL9:
	case RT5680_OK_DET_CTRL10:
	case RT5680_OK_DET_CTRL11:
	case RT5680_OK_DET_CTRL12:
	case RT5680_OK_DET_CTRL13:
	case RT5680_OK_DET_CTRL14:
	case RT5680_OK_DET_CTRL15:
	case RT5680_DFLL_CAL_CTRL1:
	case RT5680_DFLL_CAL_CTRL2:
	case RT5680_DFLL_CAL_CTRL3:
	case RT5680_DFLL_CAL_CTRL4:
	case RT5680_DFLL_CAL_CTRL5:
	case RT5680_DFLL_CAL_CTRL6:
	case RT5680_DFLL_CAL_CTRL7:
	case RT5680_DFLL_CAL_CTRL8:
	case RT5680_DFLL_CAL_CTRL9:
	case RT5680_DFLL_CAL_CTRL10:
	case RT5680_DFLL_CAL_CTRL11:
	case RT5680_DFLL_CAL_CTRL12:
	case RT5680_DFLL_CAL_CTRL13:
	case RT5680_DFLL_CAL_CTRL14:
	case RT5680_VAD_FUNCTION_CTRL1:
	case RT5680_DELAY_BUFFER_SRAM_CTRL1:
	case RT5680_DELAY_BUFFER_SRAM_CTRL2:
	case RT5680_DELAY_BUFFER_SRAM_CTRL3:
	case RT5680_DELAY_BUFFER_SRAM_CTRL4:
	case RT5680_DELAY_BUFFER_SRAM_CTRL5:
	case RT5680_DELAY_BUFFER_SRAM_CTRL6:
	case RT5680_DELAY_BUFFER_SRAM_CTRL7:
	case RT5680_DMIC_CLK_ON_OFF_CTRL1:
	case RT5680_DMIC_CLK_ON_OFF_CTRL2:
	case RT5680_DMIC_CLK_ON_OFF_CTRL3:
	case RT5680_DMIC_CLK_ON_OFF_CTRL4:
	case RT5680_DMIC_CLK_ON_OFF_CTRL5:
	case RT5680_DMIC_CLK_ON_OFF_CTRL6:
	case RT5680_DMIC_CLK_ON_OFF_CTRL7:
	case RT5680_DMIC_CLK_ON_OFF_CTRL8:
	case RT5680_DMIC_CLK_ON_OFF_CTRL9:
	case RT5680_DMIC_CLK_ON_OFF_CTRL10:
	case RT5680_DMIC_CLK_ON_OFF_CTRL11:
	case RT5680_DMIC_CLK_ON_OFF_CTRL12:
	case RT5680_DAC_MULTI_DRC_MISC_CTRL:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL1:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL2:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL3:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL4:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL5:
	case RT5680_DAC_MULTI_DRC_COEF_FB1_CTRL6:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL7:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL8:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL9:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL10:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL11:
	case RT5680_DAC_MULTI_DRC_COEF_FB2_CTRL12:
	case RT5680_DAC_MULTI_DRC_HB_CTRL1:
	case RT5680_DAC_MULTI_DRC_HB_CTRL2:
	case RT5680_DAC_MULTI_DRC_HB_CTRL3:
	case RT5680_DAC_MULTI_DRC_HB_CTRL4:
	case RT5680_DAC_MULTI_DRC_HB_CTRL5:
	case RT5680_DAC_MULTI_DRC_HB_CTRL6:
	case RT5680_DAC_MULTI_DRC_HB_CTRL7:
	case RT5680_DAC_MULTI_DRC_HB_CTRL8:
	case RT5680_DAC_MULTI_DRC_HB_CTRL9:
	case RT5680_DAC_MULTI_DRC_HB_CTRL10:
	case RT5680_DAC_MULTI_DRC_HB_CTRL11:
	case RT5680_DAC_MULTI_DRC_HB_CTRL12:
	case RT5680_DAC_MULTI_DRC_MB_CTRL1:
	case RT5680_DAC_MULTI_DRC_MB_CTRL2:
	case RT5680_DAC_MULTI_DRC_MB_CTRL3:
	case RT5680_DAC_MULTI_DRC_MB_CTRL4:
	case RT5680_DAC_MULTI_DRC_MB_CTRL5:
	case RT5680_DAC_MULTI_DRC_MB_CTRL6:
	case RT5680_DAC_MULTI_DRC_MB_CTRL7:
	case RT5680_DAC_MULTI_DRC_MB_CTRL8:
	case RT5680_DAC_MULTI_DRC_MB_CTRL9:
	case RT5680_DAC_MULTI_DRC_MB_CTRL10:
	case RT5680_DAC_MULTI_DRC_MB_CTRL11:
	case RT5680_DAC_MULTI_DRC_MB_CTRL12:
	case RT5680_DAC_MULTI_DRC_BB_CTRL1:
	case RT5680_DAC_MULTI_DRC_BB_CTRL2:
	case RT5680_DAC_MULTI_DRC_BB_CTRL3:
	case RT5680_DAC_MULTI_DRC_BB_CTRL4:
	case RT5680_DAC_MULTI_DRC_BB_CTRL5:
	case RT5680_DAC_MULTI_DRC_BB_CTRL6:
	case RT5680_DAC_MULTI_DRC_BB_CTRL7:
	case RT5680_DAC_MULTI_DRC_BB_CTRL8:
	case RT5680_DAC_MULTI_DRC_BB_CTRL9:
	case RT5680_DAC_MULTI_DRC_BB_CTRL10:
	case RT5680_DAC_MULTI_DRC_BB_CTRL11:
	case RT5680_DAC_MULTI_DRC_BB_CTRL12:
	case RT5680_DAC_MULTI_DRC_POS_CTRL1:
	case RT5680_DAC_MULTI_DRC_POS_CTRL2:
	case RT5680_DAC_MULTI_DRC_POS_CTRL3:
	case RT5680_DAC_MULTI_DRC_POS_CTRL4:
	case RT5680_DAC_MULTI_DRC_POS_CTRL5:
	case RT5680_DAC_MULTI_DRC_POS_CTRL6:
	case RT5680_DAC_MULTI_DRC_POS_CTRL7:
	case RT5680_DAC_MULTI_DRC_POS_CTRL8:
	case RT5680_DAC_MULTI_DRC_POS_CTRL9:
	case RT5680_DAC_MULTI_DRC_POS_CTRL10:
	case RT5680_DAC_MULTI_DRC_POS_CTRL11:
	case RT5680_DAC_MULTI_DRC_POS_CTRL12:
	case RT5680_DAC_MULTI_DRC_POS_CTRL13:
	case RT5680_DAC_MULTI_DRC_POS_ST1:
	case RT5680_DAC_MULTI_DRC_POS_ST2:
	case RT5680_ADC_ALC_CTRL1:
	case RT5680_ADC_ALC_CTRL2:
	case RT5680_ADC_ALC_CTRL3:
	case RT5680_ADC_ALC_CTRL4:
	case RT5680_ADC_ALC_CTRL5:
	case RT5680_ADC_ALC_CTRL6:
	case RT5680_ADC_ALC_CTRL7:
	case RT5680_ADC_ALC_CTRL8:
	case RT5680_ADC_ALC_CTRL9:
	case RT5680_ADC_ALC_CTRL10:
	case RT5680_ADC_ALC_CTRL11:
	case RT5680_ADC_ALC_CTRL12:
	case RT5680_ADC_ALC_CTRL13:
	case RT5680_ADC_ALC_CTRL14:
	case RT5680_ADC_ALC_ST1:
	case RT5680_ADC_ALC_ST2:
	case RT5680_HP_DC_CAL_CTRL1:
	case RT5680_HP_DC_CAL_CTRL2:
	case RT5680_HP_DC_CAL_CTRL3:
	case RT5680_HP_DC_CAL_CTRL4:
	case RT5680_HP_DC_CAL_CTRL5:
	case RT5680_HP_DC_CAL_CTRL6:
	case RT5680_HP_DC_CAL_CTRL7:
	case RT5680_HP_DC_CAL_CTRL8:
	case RT5680_HP_DC_CAL_CTRL9:
	case RT5680_HP_DC_CAL_CTRL10:
	case RT5680_HP_DC_CAL_CTRL11:
	case RT5680_HP_DC_CAL_CTRL12:
	case RT5680_HP_DC_CAL_ST1:
	case RT5680_HP_DC_CAL_ST2:
	case RT5680_HP_DC_CAL_ST3:
	case RT5680_HP_DC_CAL_ST4:
	case RT5680_HP_DC_CAL_ST5:
	case RT5680_HP_DC_CAL_ST6:
	case RT5680_HP_DC_CAL_ST7:
	case RT5680_HP_DC_CAL_ST8:
	case RT5680_HP_DC_CAL_ST9:
	case RT5680_HP_DC_CAL_ST10:
	case RT5680_HP_DC_CAL_ST11:
	case RT5680_HP_DC_CAL_ST12:
	case RT5680_HP_DC_CAL_ST13:
	case RT5680_MONO_AMP_DC_CAL_CTRL1:
	case RT5680_MONO_AMP_DC_CAL_CTRL2:
	case RT5680_MONO_AMP_DC_CAL_CTRL3:
	case RT5680_MONO_AMP_DC_CAL_CTRL4:
	case RT5680_MONO_AMP_DC_CAL_CTRL5:
	case RT5680_MONO_AMP_DC_CAL_CTRL6:
	case RT5680_MONO_AMP_DC_CAL_CTRL7:
	case RT5680_MONO_AMP_DC_CAL_ST1:
	case RT5680_MONO_AMP_DC_CAL_ST2:
	case RT5680_MONO_AMP_DC_CAL_ST3:
	case RT5680_DSP_IB_CTRL1:
	case RT5680_DSP_IB_CTRL2:
	case RT5680_DSP_IN_OB_CTRL:
	case RT5680_DSP_OB01_DIG_VOL:
	case RT5680_DSP_OB23_DIG_VOL:
	case RT5680_DSP_OB45_DIG_VOL:
	case RT5680_DSP_OB67_DIG_VOL:
	case RT5680_MINI_DSP_OB01_DIG_VOL:
	case RT5680_DSP_IB1_SRC_CTRL1:
	case RT5680_DSP_IB1_SRC_CTRL2:
	case RT5680_DSP_IB1_SRC_CTRL3:
	case RT5680_DSP_IB1_SRC_CTRL4:
	case RT5680_DSP_IB2_SRC_CTRL1:
	case RT5680_DSP_IB2_SRC_CTRL2:
	case RT5680_DSP_IB2_SRC_CTRL3:
	case RT5680_DSP_IB2_SRC_CTRL4:
	case RT5680_DSP_IB3_SRC_CTRL1:
	case RT5680_DSP_IB3_SRC_CTRL2:
	case RT5680_DSP_IB3_SRC_CTRL3:
	case RT5680_DSP_IB3_SRC_CTRL4:
	case RT5680_DSP_OB1_SRC_CTRL1:
	case RT5680_DSP_OB1_SRC_CTRL2:
	case RT5680_DSP_OB1_SRC_CTRL3:
	case RT5680_DSP_OB1_SRC_CTRL4:
	case RT5680_DSP_OB2_SRC_CTRL1:
	case RT5680_DSP_OB2_SRC_CTRL2:
	case RT5680_DSP_OB2_SRC_CTRL3:
	case RT5680_DSP_OB2_SRC_CTRL4:
	case RT5680_HIFI_MINI_DSP_CTRL_ST:
	case RT5680_SPI_SLAVE_CRC_CHECK_CTRL:
	case RT5680_EFUSE_CTRL1:
	case RT5680_EFUSE_CTRL2:
	case RT5680_EFUSE_CTRL3:
	case RT5680_EFUSE_CTRL4:
	case RT5680_EFUSE_CTRL5:
	case RT5680_EFUSE_CTRL6:
	case RT5680_EFUSE_CTRL7:
	case RT5680_EFUSE_CTRL8:
	case RT5680_EFUSE_CTRL9:
	case RT5680_EFUSE_CTRL10:
	case RT5680_EFUSE_CTRL11:
	case RT5680_I2C_AND_SPI_SCRAM_CTRL:
	case RT5680_I2C_SCRAM_WRITE_KEY1_MSB:
	case RT5680_I2C_SCRAM_WRITE_KEY1_LSB:
	case RT5680_I2C_SCRAM_WRITE_KEY2_MSB:
	case RT5680_I2C_SCRAM_WRITE_KEY2_LSB:
	case RT5680_I2C_SCRAM_READ_KEY1_MSB:
	case RT5680_I2C_SCRAM_READ_KEY1_LSB:
	case RT5680_I2C_SCRAM_READ_KEY2_MSB:
	case RT5680_I2C_SCRAM_READ_KEY2_LSB:
	case RT5680_SPI_SCRAM_WRITE_KEY1_1:
	case RT5680_SPI_SCRAM_WRITE_KEY1_2:
	case RT5680_SPI_SCRAM_WRITE_KEY1_3:
	case RT5680_SPI_SCRAM_WRITE_KEY1_4:
	case RT5680_SPI_SCRAM_WRITE_KEY2_1:
	case RT5680_SPI_SCRAM_WRITE_KEY2_2:
	case RT5680_SPI_SCRAM_WRITE_KEY2_3:
	case RT5680_SPI_SCRAM_WRITE_KEY2_4:
	case RT5680_SPI_SCRAM_READ_KEY1_1:
	case RT5680_SPI_SCRAM_READ_KEY1_2:
	case RT5680_SPI_SCRAM_READ_KEY1_3:
	case RT5680_SPI_SCRAM_READ_KEY1_4:
	case RT5680_SPI_SCRAM_READ_KEY2_1:
	case RT5680_SPI_SCRAM_READ_KEY2_2:
	case RT5680_SPI_SCRAM_READ_KEY2_3:
	case RT5680_SPI_SCRAM_READ_KEY2_4:
	case RT5680_GPIO1_TEST_OUTPUT_SEL1:
	case RT5680_GPIO1_TEST_OUTPUT_SEL2:
	case RT5680_GPIO1_TEST_OUTPUT_SEL3:
	case RT5680_GPIO1_TEST_OUTPUT_SEL4:
	case RT5680_PR_REG_MONO_AMP_BIAS_CTRL:
	case RT5680_PR_REG_BIAS_CTRL1:
	case RT5680_PR_REG_BIAS_CTRL2:
	case RT5680_PR_REG_BIAS_CTRL3:
	case RT5680_PR_REG_BIAS_CTRL4:
	case RT5680_PR_REG_BIAS_CTRL5:
	case RT5680_PR_REG_BIAS_CTRL6:
	case RT5680_PR_REG_BIAS_CTRL7:
	case RT5680_PR_REG_BIAS_CTRL8:
	case RT5680_PR_REG_BIAS_CTRL9:
	case RT5680_PR_REG_BIAS_CTRL10:
	case RT5680_PR_REG_BIAS_CTRL11:
	case RT5680_PR_REG_BIAS_CTRL12:
	case RT5680_PR_REG_BIAS_CTRL13:
	case RT5680_PR_REG_ADC12_CLK_CTRL:
	case RT5680_PR_REG_ADC34_CLK_CTRL:
	case RT5680_PR_REG_ADC5_CLK_CTRL1:
	case RT5680_PR_REG_ADC5_CLK_CTRL2:
	case RT5680_PR_REG_ADC67_CLK_CTRL:
	case RT5680_PR_REG_PLL1_CTRL1:
	case RT5680_PR_REG_PLL1_CTRL2:
	case RT5680_PR_REG_PLL2_CTRL1:
	case RT5680_PR_REG_PLL2_CTRL2:
	case RT5680_PR_REG_VREF_CTRL1:
	case RT5680_PR_REG_VREF_CTRL2:
	case RT5680_PR_REG_BST1_CTRL:
	case RT5680_PR_REG_BST2_CTRL:
	case RT5680_PR_REG_BST3_CTRL:
	case RT5680_PR_REG_BST4_CTRL:
	case RT5680_DAC_ADC_DIG_VOL1:
	case RT5680_DAC_ADC_DIG_VOL2:
	case RT5680_VAD_SRAM_TEST:
	case RT5680_PAD_DRIVING_CTRL1:
	case RT5680_PAD_DRIVING_CTRL2:
	case RT5680_PAD_DRIVING_CTRL3:
	case RT5680_DIG_INPUT_PIN_ST_CTRL1:
	case RT5680_DIG_INPUT_PIN_ST_CTRL2:
	case RT5680_DIG_INPUT_PIN_ST_CTRL3:
	case RT5680_DIG_INPUT_PIN_ST_CTRL4:
	case RT5680_DIG_INPUT_PIN_ST_CTRL5:
	case RT5680_TEST_MODE_CTRL1:
	case RT5680_TEST_MODE_CTRL2:
	case RT5680_GPIO1_GPIO3_TEST_MODE_CTRL:
	case RT5680_GPIO5_GPIO6_TEST_MODE_CTRL:
	case RT5680_GPIO6_GPIO7_TEST_MODE_CTRL:
	case RT5680_CODEC_DOMAIN_REG_RW_CTRL:
	case RT5680_DAC1_CLK_AND_CHOPPER_CTRL:
	case RT5680_DAC2_CLK_AND_CHOPPER_CTRL:
	case RT5680_DAC3_CLK_AND_CHOPPER_CTRL:
	case RT5680_DAC4_CLK_AND_CHOPPER_CTRL:
	case RT5680_DAC5_CLK_AND_CHOPPER_CTRL:
	case RT5680_DAC1_DAC2_DUMMY_REG:
	case RT5680_HP_CTRL1:
	case RT5680_HP_CTRL2:
	case RT5680_HP_CTRL3:
	case RT5680_HP_CTRL4:
	case RT5680_HP_CTRL5:
	case RT5680_HP_CTRL6:
	case RT5680_LDO6_PR_CTRL1:
	case RT5680_LDO6_PR_CTRL2:
	case RT5680_LDO6_PR_CTRL3:
	case RT5680_LDO6_PR_CTRL4:
	case RT5680_LDO_AVDD1_PR_CTRL:
	case RT5680_LDO_HV2_PR_CTRL:
	case RT5680_LDO_HV3_PR_CTRL:
	case RT5680_LDO1_LDO3_LDO4_PR_CTRL:
	case RT5680_LDO8_LDO9_PR_CTRL:
	case RT5680_VREF5_L_PR_CTRL:
	case RT5680_VREF5_R_PR_CTRL:
	case RT5680_SLIMBUS_PARAMETER:
	case RT5680_SLIMBUS_RX:
	case RT5680_SLIMBUS_CTRL:
	case RT5680_LOUT_CTRL:
	case RT5680_DUMMY_REG_1:
	case RT5680_DUMMY_REG_2:
	case RT5680_DUMMY_REG_3:
	case RT5680_DUMMY_REG_4:
	case RT5680_DSP_I2C_DATA_MSB:
		return true;

	default:
		return false;
	}
}

static ssize_t rt5680_codec_reset_init_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0] == 'r')
	{
		if(!init_done)
		{
			printk("%s,init_done=%d,reset and init rt5680\n", __func__, init_done);
			init_done = 1;
			
			gpio_direction_output(reset_gpio, 0);
			msleep(20);
			gpio_direction_output(reset_gpio, 1);

			msleep(10);
		   
			codec_init();
		}
	}
	
	return count;
}

static ssize_t rt5680_codec_reset_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;
}
static DEVICE_ATTR(codec_reset_init, 0664, rt5680_codec_reset_init_show, rt5680_codec_reset_init_store);


static ssize_t rt5680_codec_show_range(struct rt5680_priv *rt5680,
	char *buf, int start, int end)
{
	unsigned int val;
	int cnt = 0, i;

	for (i = start; i <= end; i++) {
		if (cnt + RT5680_REG_DISP_LEN >= PAGE_SIZE)
			break;

		if (rt5680_readable_register(NULL, i)) {
			rt5680_read(i, &val);

			cnt += snprintf(buf + cnt, RT5680_REG_DISP_LEN,
					"%04x: %04x\n", i, val);
		}
	}

	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;

	return cnt;
}

static ssize_t rt5680_codec_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct rt5680_priv *rt5680 = g_rt5680;
	ssize_t cnt;

	//mutex_lock(&rt5680->mutex);
	cnt = rt5680_codec_show_range(rt5680, buf, rt5680->reg_page << 8,
		(rt5680->reg_page << 8) | 0xff);
	//mutex_unlock(&rt5680->mutex);
	return cnt;
}

static ssize_t rt5680_codec_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct rt5680_priv *rt5680 = g_rt5680;
	unsigned int val = 0, addr = 0;
	int i;

	if (buf[0] == 'P' || buf[0] == 'p') {
		rt5680->reg_page = buf[1] - '0';
		return count;
	}

	pr_info("register \"%s\" count = %zu\n", buf, count);
	for (i = 0; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			addr = (addr << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			addr = (addr << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}

	for (i = i + 1; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			val = (val << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			val = (val << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			val = (val << 4) | ((*(buf + i) - 'A') + 0xa);
		else
			break;
	}

	pr_info("addr = 0x%04x val = 0x%04x\n", addr, val);
	if (addr > RT5680_DUMMY_REG_4 || val > 0xffff || val < 0)
		return count;

	//mutex_lock(&rt5680->mutex);
	if (i == count) {
		rt5680_read( addr, &val);
		pr_info("0x%04x = 0x%04x\n", addr, val);
	} else
		rt5680_write(addr, val);
	//mutex_unlock(&rt5680->mutex);

	return count;
}
static DEVICE_ATTR(codec_reg, 0644, rt5680_codec_show, rt5680_codec_store);

static ssize_t rt5680_is_dsp_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct rt5680_priv *rt5680 = g_rt5680;

	return snprintf(buf, 3, "%c\n", rt5680->is_dsp_mode ? 'Y' : 'N');
}
static DEVICE_ATTR(is_dsp_mode, 0444, rt5680_is_dsp_mode_show, NULL);

static ssize_t rt5680_codec_adb_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct rt5680_priv *rt5680 = g_rt5680;
	unsigned int val;
	int cnt = 0, i;

	//mutex_lock(&rt5680->mutex);
	for (i = 0; i < rt5680->adb_reg_num; i++) {
		if (cnt + RT5680_REG_DISP_LEN >= PAGE_SIZE)
			break;
		rt5680_read( rt5680->adb_reg_addr[i], &val);
		cnt += snprintf(buf + cnt, 23, "%08x: %08x\n\n",
			rt5680->adb_reg_addr[i], val);
	}
	//mutex_unlock(&rt5680->mutex);

	return cnt;
}

static ssize_t rt5680_codec_adb_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct rt5680_priv *rt5680 = g_rt5680;
	unsigned int value = 0;
	int i = 2, j = 0;

	if (buf[0] == 'R' || buf[0] == 'r') {
		while (j <= 0x100 && i < count) {
			rt5680->adb_reg_addr[j] = 0;
			value = 0;
			for ( ; i < count; i++) {
				if (*(buf + i) <= '9' && *(buf + i) >= '0')
					value = (value << 4) | (*(buf + i) - '0');
				else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
					value = (value << 4) | ((*(buf + i) - 'a')+0xa);
				else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
					value = (value << 4) | ((*(buf + i) - 'A')+0xa);
				else
					break;
			}
			i++;

			rt5680->adb_reg_addr[j] = value;
			j++;
		}
		rt5680->adb_reg_num = j;
	} else if (buf[0] == 'W' || buf[0] == 'w') {
		while (j <= 0x100 && i < count) {
			/* Get address */
			rt5680->adb_reg_addr[j] = 0;
			value = 0;
			for ( ; i < count; i++) {
				if (*(buf + i) <= '9' && *(buf + i) >= '0')
					value = (value << 4) | (*(buf + i) - '0');
				else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
					value = (value << 4) | ((*(buf + i) - 'a')+0xa);
				else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
					value = (value << 4) | ((*(buf + i) - 'A')+0xa);
				else
					break;
			}
			i++;
			rt5680->adb_reg_addr[j] = value;

			/* Get value */
			rt5680->adb_reg_value[j] = 0;
			value = 0;
			for ( ; i < count; i++) {
				if (*(buf + i) <= '9' && *(buf + i) >= '0')
					value = (value << 4) | (*(buf + i) - '0');
				else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
					value = (value << 4) | ((*(buf + i) - 'a')+0xa);
				else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
					value = (value << 4) | ((*(buf + i) - 'A')+0xa);
				else
					break;
			}
			i++;
			rt5680->adb_reg_value[j] = value;

			j++;
		}

		rt5680->adb_reg_num = j;

		//mutex_lock(&rt5680->mutex);
		for (i = 0; i < rt5680->adb_reg_num; i++) {
			rt5680_write(rt5680->adb_reg_addr[i] & 0xffff,
				rt5680->adb_reg_value[i]);
		}
		//mutex_unlock(&rt5680->mutex);
	}

	return count;
}
static DEVICE_ATTR(codec_reg_adb, 0664, rt5680_codec_adb_show,
			rt5680_codec_adb_store);

static int rt5680_dsp_load_fw(void)
{
	const struct firmware *fw;
	struct rt5680_priv *rt5680 = g_rt5680;
	int ret = 0;
	unsigned int val;

#ifdef BYPASS_DSP
	return 0;
#endif
	if (fw_load_done !=0) {
		printk("rt5680 fw had been load!\n");
		return 0;
	}
	mutex_lock(&rt5680->mutex);
	ret = request_firmware(&fw, "0x5ffc0000.dat", &rt5680->i2c->dev);
	if (ret == 0) {
		rt5680_spi_burst_write(0x5ffc0000, fw->data, fw->size);
		printk("%s write successed\n", "0x5ffc0000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n", "0x5ffc0000.dat", ret);
		fw_load_done = 1;
		goto finish;
	}
	ret = request_firmware(&fw, "0x5ffe0000.dat", &rt5680->i2c->dev);
	if (ret == 0) {
		rt5680_spi_burst_write(0x5ffe0000, fw->data, fw->size);
		printk("%s write successed\n", "0x5ffe0000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n", "0x5ffe0000.dat", ret);
		fw_load_done = 2;
		goto finish;
	}
	ret = request_firmware(&fw, "0x50000000.dat", &rt5680->i2c->dev);
	if (ret == 0) {

		rt5680_spi_burst_write(0x50000000, fw->data, fw->size);

		printk("%s write successed\n","0x50000000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n", "0x50000000.dat", ret);
		fw_load_done = 3;
		goto finish;
	}
	ret = request_firmware(&fw, "0x60000000.dat", &rt5680->i2c->dev);
	if (ret == 0) {

		rt5680_spi_burst_write(0x60000000, fw->data, fw->size);

		printk("%s write successed\n", "0x60000000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n","0x60000000.dat", ret);
		fw_load_done = 4;
		goto finish;
	}
	ret = request_firmware(&fw, "0xffc0000.dat", &rt5680->i2c->dev);
	if (ret == 0) {

		rt5680_spi_burst_write(0xffc0000, fw->data, fw->size);

		printk("%s write successed\n", "0xffc0000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n", "0xffc0000.dat", ret);
		fw_load_done = 5;
		goto finish;
	}
	ret = request_firmware(&fw, "0xffe0000.dat", &rt5680->i2c->dev);
	if (ret == 0) {

		rt5680_spi_burst_write(0xffe0000, fw->data, fw->size);

		printk("%s write successed\n", "0xffe0000.dat");
		release_firmware(fw);
	}else{
		printk("%s request_firmware error %d\n", "0xffe0000.dat", ret);
		fw_load_done = 6;
		goto finish;
	}

finish:
	mutex_unlock(&rt5680->mutex);
	if (ret==0) {
		fw_load_done = 10;

		rt5680_dsp_mode_i2c_read(rt5680, RT5680_PWR_DSP, &val);
		rt5680_dsp_mode_i2c_write(rt5680, RT5680_PWR_DSP, (val&~1));
	}

	return ret;
}

static void rt5680_reset_codec(void)
{
	//TODO: do GPIO High -> Low -> High to reset codec

	fw_load_done = 0;
	g_rt5680->is_dsp_mode = false;

	/* reload codec setting */
	codec_init();

	/* reload firmware */
	rt5680_dsp_load_fw();
}

static const struct regmap_config rt5680_regmap_physical = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = RT5680_DUMMY_REG_4,
	.readable_reg = rt5680_readable_register,
	.cache_type = REGCACHE_NONE,
};

static int rt5680_load_fw_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%u\n", fw_load_done);

	return 0;
}

static int rt5680_load_fw_open(struct inode *inode, struct file *file)
{
	return single_open(file, rt5680_load_fw_show, NULL);
}

static ssize_t rt5680_load_fw_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos)
{
	char *tmp = kzalloc((count+1), GFP_KERNEL);

	if (!tmp)
		return -ENOMEM;

	if (copy_from_user(tmp, buf, count)) {
		kfree(tmp);
		return -EFAULT;
	}

	if (strncmp(tmp, "1", 1) == 0) {
		rt5680_dsp_load_fw();
	} else if (strncmp(tmp, "2", 1) == 0) {
		rt5680_reset_codec();
	}

	kfree(tmp);
	return count;
}

static const struct file_operations rt5680_load_fw_fops = {
	.owner		= THIS_MODULE,
	.open		= rt5680_load_fw_open,
	.read		= seq_read,
	.write		= rt5680_load_fw_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct i2c_device_id rt5680_i2c_id[] = {
	{ "rt5680", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5680_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id rt5680_of_match[] = {
	{ .compatible = "realtek,rt5680", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5680_of_match);
#endif


static int rt5680_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct rt5680_priv *rt5680;
	int ret;

	printk("===mlk===start %s\n",__func__);

	rt5680 = devm_kzalloc(&i2c->dev, sizeof(struct rt5680_priv),
				GFP_KERNEL);
	if (rt5680 == NULL)
		return -ENOMEM;

	rt5680->regmap_physical = devm_regmap_init_i2c(i2c, &rt5680_regmap_physical);
	if (IS_ERR(rt5680->regmap_physical)) {
		ret = PTR_ERR(rt5680->regmap_physical);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	g_rt5680 = rt5680;

	fw_load_done = 0;
	rt5680->is_dsp_mode = 0;
	rt5680->i2c = i2c;

	reset_gpio = of_get_named_gpio(i2c->dev.of_node, "rst-gpio", 0);

	ret = gpio_request(reset_gpio, "rt5680_reset_gpio");

	 gpio_direction_output(reset_gpio, 1);

	 msleep(10);

	rt5680_read( RT5680_VENDOR_ID2, &ret);
	if (ret != RT5680_DEVICE_ID) {
		dev_err(&i2c->dev,
			"RT5680 is NOT at i2c mode\n");
	}

	if (!rt5680_spi) {
		printk("rt5680_spi %p\n", rt5680_spi);
		goto _err_defer;
	}

	pr_debug("Ready to rt5680 init\n");
	
	//codec_init();

	ret = device_create_file(&i2c->dev, &dev_attr_codec_reg);
	if (ret != 0) {
		dev_err(&i2c->dev,
			"Failed to create codec_reg sysfs files: %d\n", ret);
		return ret;
	}

	ret = device_create_file(&i2c->dev, &dev_attr_codec_reg_adb);
	if (ret != 0) {
		dev_err(&i2c->dev,
			"Failed to create codec_reg_adb sysfs files: %d\n", ret);
		return ret;
	}

	ret = device_create_file(&i2c->dev, &dev_attr_is_dsp_mode);
	if (ret != 0) {
		dev_err(&i2c->dev,
			"Failed to create is_dsp_mode sysfs files: %d\n", ret);
		return ret;
	}

	ret = device_create_file(&i2c->dev, &dev_attr_codec_reset_init);
	if (ret != 0) {
		dev_err(&i2c->dev,
			"Failed to create is_dsp_mode sysfs files: %d\n", ret);
		return ret;
	}

	rt5680_dir = proc_mkdir("rt5680", NULL);
	proc_create("load_fw", 0666, rt5680_dir, &rt5680_load_fw_fops);

	mutex_init(&rt5680->mutex);

	printk("===mlk===end %s\n",__func__);

	return 0;

_err_defer:
	//return -EPROBE_DEFER;
	return -1;
}

static int rt5680_i2c_remove(struct i2c_client *client)
{
	struct rt5680_priv *rt5680 = i2c_get_clientdata(client);

	printk("%s... \n", __func__);

	regmap_exit(rt5680->regmap);

	return 0;
}

void rt5680_i2c_shutdown(struct i2c_client *client)
{
	printk("%s... \n", __func__);

	if (rt5680_dir != NULL) {
		remove_proc_entry("load_fw", rt5680_dir);
		remove_proc_entry("rt5680", NULL);
		rt5680_dir = NULL;
	}
}

static struct i2c_driver rt5680_i2c_driver = {
	.driver = {
		.name = "rt5680",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt5680_of_match),
	},
	.probe = rt5680_i2c_probe,
	.remove   = rt5680_i2c_remove,
	.shutdown = rt5680_i2c_shutdown,
	.id_table = rt5680_i2c_id,
};

static int __init rt5680_modinit(void)
{

	return i2c_add_driver(&rt5680_i2c_driver);
}

static void __exit rt5680_modexit(void)
{
	i2c_del_driver(&rt5680_i2c_driver);
}

module_init(rt5680_modinit);
module_exit(rt5680_modexit);

MODULE_DESCRIPTION("ASoC ALC5680 driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");

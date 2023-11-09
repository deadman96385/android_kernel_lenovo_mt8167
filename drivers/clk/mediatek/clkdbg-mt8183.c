/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Weiyi Lu <weiyi.lu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>

#include "clkdbg.h"

#define DUMP_INIT_STATE		0

/*
 * clkdbg dump_regs
 */

enum {
	topckgen,
	infracfg,
	scpsys,
	apmixedsys,
	audiosys,
	mfgsys,
	mmsys,
	imgsys,
	camsys,
	vencsys,
};

#define REGBASE_V(_phys, _id_name) { .phys = _phys, .name = #_id_name }

/*
 * checkpatch.pl ERROR:COMPLEX_MACRO
 *
 * #define REGBASE(_phys, _id_name) [_id_name] = REGBASE_V(_phys, _id_name)
 */

static struct regbase rb[] = {
	[topckgen] = REGBASE_V(0x10000000, topckgen),
	[infracfg] = REGBASE_V(0x10001000, infracfg),
	[scpsys]   = REGBASE_V(0x10006000, scpsys),
	[apmixedsys]  = REGBASE_V(0x1000c000, apmixedsys),
	[audiosys]    = REGBASE_V(0x11220000, audiosys),
	[mfgsys]   = REGBASE_V(0x13000000, mfgsys),
	[mmsys]    = REGBASE_V(0x14000000, mmsys),
	[imgsys]   = REGBASE_V(0x15020000, imgsys),
	[camsys]   = REGBASE_V(0x1a000000, camsys),
	[vencsys]  = REGBASE_V(0x17000000, vencsys),
};

#define REGNAME(_base, _ofs, _name)	\
	{ .base = &rb[_base], .ofs = _ofs, .name = #_name }

static struct regname rn[] = {
	REGNAME(topckgen,  0x040, CLK_CFG_0),
	REGNAME(topckgen,  0x050, CLK_CFG_1),
	REGNAME(topckgen,  0x060, CLK_CFG_2),
	REGNAME(topckgen,  0x070, CLK_CFG_3),
	REGNAME(topckgen,  0x080, CLK_CFG_4),
	REGNAME(topckgen,  0x090, CLK_CFG_5),
	REGNAME(topckgen,  0x0a0, CLK_CFG_6),
	REGNAME(topckgen,  0x0b0, CLK_CFG_7),
	REGNAME(topckgen,  0x0c0, CLK_CFG_8),
	REGNAME(topckgen,  0x0d0, CLK_CFG_9),
	REGNAME(topckgen,  0x0e0, CLK_CFG_10),
	REGNAME(audiosys,  0x000, AUDIO_TOP_CON0),
	REGNAME(audiosys,  0x004, AUDIO_TOP_CON1),
	REGNAME(camsys,  0x000, CAMSYS_CG),
	REGNAME(imgsys,  0x000, IMG_CG),
	REGNAME(infracfg,  0x090, MODULE_SW_CG_0),
	REGNAME(infracfg,  0x094, MODULE_SW_CG_1),
	REGNAME(infracfg,  0x0ac, MODULE_SW_CG_2),
	REGNAME(infracfg,  0x0c8, MODULE_SW_CG_3),
	REGNAME(mfgsys,  0x000, MFG_CG),
	REGNAME(mmsys,	0x100, MMSYS_CG_CON0),
	REGNAME(mmsys,	0x110, MMSYS_CG_CON1),
	REGNAME(vencsys,  0x000, VENCSYS_CG),
	REGNAME(apmixedsys,  0x200, ARMPLL_LL_CON0),
	REGNAME(apmixedsys,  0x204, ARMPLL_LL_CON1),
	REGNAME(apmixedsys,  0x20C, ARMPLL_LL_PWR_CON0),
	REGNAME(apmixedsys,  0x210, ARMPLL_L_CON0),
	REGNAME(apmixedsys,  0x214, ARMPLL_L_CON1),
	REGNAME(apmixedsys,  0x21C, ARMPLL_L_PWR_CON0),
	REGNAME(apmixedsys,  0x220, MAINPLL_CON0),
	REGNAME(apmixedsys,  0x224, MAINPLL_CON1),
	REGNAME(apmixedsys,  0x22C, MAINPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x230, UNIVPLL_CON0),
	REGNAME(apmixedsys,  0x234, UNIVPLL_CON1),
	REGNAME(apmixedsys,  0x23C, UNIVPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x240, MFGPLL_CON0),
	REGNAME(apmixedsys,  0x244, MFGPLL_CON1),
	REGNAME(apmixedsys,  0x24C, MFGPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x250, MSDCPLL_CON0),
	REGNAME(apmixedsys,  0x254, MSDCPLL_CON1),
	REGNAME(apmixedsys,  0x25C, MSDCPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x260, TVDPLL_CON0),
	REGNAME(apmixedsys,  0x264, TVDPLL_CON1),
	REGNAME(apmixedsys,  0x26C, TVDPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x270, MMPLL_CON0),
	REGNAME(apmixedsys,  0x274, MMPLL_CON1),
	REGNAME(apmixedsys,  0x27C, MMPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x280, MPLL_CON0),
	REGNAME(apmixedsys,  0x284, MPLL_CON1),
	REGNAME(apmixedsys,  0x28C, MPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x290, CCIPLL_CON0),
	REGNAME(apmixedsys,  0x294, CCIPLL_CON1),
	REGNAME(apmixedsys,  0x29C, CCIPLL_PWR_CON0),
	REGNAME(apmixedsys,  0x2A0, APLL1_CON0),
	REGNAME(apmixedsys,  0x2A4, APLL1_CON1),
	REGNAME(apmixedsys,  0x2B0, APLL1_PWR_CON0),
	REGNAME(apmixedsys,  0x2B4, APLL2_CON0),
	REGNAME(apmixedsys,  0x2B8, APLL2_CON1),
	REGNAME(apmixedsys,  0x2C4, APLL2_PWR_CON0),
	REGNAME(scpsys,  0x0180, PWR_STATUS),
	REGNAME(scpsys,  0x0184, PWR_STATUS_2ND),
	REGNAME(scpsys,  0x0334, MFG_ASYNC_PWR_CON),
	REGNAME(scpsys,  0x0338, MFG_PWR_CON),
	REGNAME(scpsys,  0x033C, MFG_CORE0_PWR_CON),
	REGNAME(scpsys,  0x0340, MFG_CORE1_PWR_CON),
	REGNAME(scpsys,  0x0320, MD1_PWR_CON),
	REGNAME(scpsys,  0x032C, CONN_PWR_CON),
	REGNAME(scpsys,  0x0314, AUD_PWR_CON),
	REGNAME(scpsys,  0x030C, DIS_PWR_CON),
	REGNAME(scpsys,  0x0344, CAM_PWR_CON),
	REGNAME(scpsys,  0x0308, ISP_PWR_CON),
	REGNAME(scpsys,  0x0304, VEN_PWR_CON),
	{}
};

static const struct regname *get_all_regnames(void)
{
	return rn;
}

static void __init init_regbase(void)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(rb); i++)
		rb[i].virt = ioremap(rb[i].phys, PAGE_SIZE);
}

/*
 * clkdbg fmeter
 */

#include <linux/delay.h>

#define clk_readl(addr)		readl(addr)
#define clk_writel(addr, val)	\
	do { writel(val, addr); wmb(); } while (0) /* sync write */

#define FMCLK(_t, _i, _n) { .type = _t, .id = _i, .name = _n }

static const struct fmeter_clk fclks[] = {
	FMCLK(CKGEN,  1, "hd_faxi_ck"),
	FMCLK(CKGEN,  2, "hf_fmm_ck"),
	FMCLK(CKGEN,  3, "hf_fimg_ck"),
	FMCLK(CKGEN,  4, "hf_fcam_ck"),
	FMCLK(CKGEN,  5, "hf_fdsp_ck"),
	FMCLK(CKGEN,  6, "hf_fdsp1_ck"),
	FMCLK(CKGEN,  7, "hf_fdsp2_ck"),
	FMCLK(CKGEN,  8, "hf_fipu_if_ck"),
	FMCLK(CKGEN,  9, "hf_fmfg_ck"),
	FMCLK(CKGEN,  10, "f52m_mfg_ck"),
	FMCLK(CKGEN,  11, "f_fcamtg_ck"),
	FMCLK(CKGEN,  12, "f_fcamtg2_ck"),
	FMCLK(CKGEN,  13, "f_fcamtg3_ck"),
	FMCLK(CKGEN,  14, "f_fcamtg4_ck"),
	FMCLK(CKGEN,  15, "f_fuart_ck"),
	FMCLK(CKGEN,  16, "hf_fspi_ck"),
	FMCLK(CKGEN,  17, "hf_fmsdc50_0_hclk_ck"),
	FMCLK(CKGEN,  18, "hf_fmsdc50_0_ck"),
	FMCLK(CKGEN,  19, "hf_fmsdc30_1_ck"),
	FMCLK(CKGEN,  20, "hf_fmsdc30_2_ck"),
	FMCLK(CKGEN,  21, "hf_faudio_ck"),
	FMCLK(CKGEN,  22, "hf_faud_intbus_ck"),
	FMCLK(CKGEN,  23, "hf_fpmicspi_ck"),
	FMCLK(CKGEN,  24, "f_fpwrap_ulposc_ck"),
	FMCLK(CKGEN,  25, "hf_fatb_ck"),
	FMCLK(CKGEN,  26, "hf_fsspm_ck"),
	FMCLK(CKGEN,  27, "hf_fdpi0_ck"),
	FMCLK(CKGEN,  28, "hf_fscam_ck"),
	FMCLK(CKGEN,  29, "f_fdisp_pwm_ck"),
	FMCLK(CKGEN,  30, "f_fusb_top_ck"),
	FMCLK(CKGEN,  31, "f_fssusb_xhci_ck"),
	FMCLK(CKGEN,  32, "hg_fspm_ck"),
	FMCLK(CKGEN,  33, "f_fi2c_ck"),
	FMCLK(CKGEN,  34, "hf_fscp_ck"),
	FMCLK(CKGEN,  35, "f_fseninf_ck"),
	FMCLK(CKGEN,  36, "f_fdxcc_ck"),
	FMCLK(CKGEN,  37, "hf_faud_engin1_ck"),
	FMCLK(CKGEN,  38, "hf_faud_engin2_ck"),
	FMCLK(CKGEN,  39, "hf_faes_ufsfde_ck"),
	FMCLK(CKGEN,  40, "hf_fufs_ck"),
	FMCLK(CKGEN,  41, "hf_faud_1_ck"),
	FMCLK(CKGEN,  42, "hf_faud_2_ck"),
	FMCLK(CKGEN,  49, "hf_fref_mm_ck"),
	FMCLK(CKGEN,  50, "hf_fref_cam_ck"),
	FMCLK(CKGEN,  51, "hf_hddrphycfg_ck"),
	FMCLK(CKGEN,  52, "f_ufs_mp_sap_cfg_ck"),
	FMCLK(CKGEN,  53, "f_ufs_tick1us_ck"),
	FMCLK(CKGEN,  54, "hd_faxi_east_ck"),
	FMCLK(CKGEN,  55, "hd_faxi_west_ck"),
	FMCLK(CKGEN,  56, "hd_faxi_north_ck"),
	FMCLK(CKGEN,  57, "hd_faxi_south_ck"),
	FMCLK(CKGEN,  58, "hg_fmipicfg_tx_ck"),
	FMCLK(CKGEN,  59, "fmem_ck_bfe_dcm_ch0"),
	FMCLK(CKGEN,  60, "fmem_ck_aft_dcm_ch0"),
	FMCLK(CKGEN,  61, "fmem_ck_bfe_dcm_ch1"),
	FMCLK(CKGEN,  62, "fmem_ck_aft_dcm_ch1"),
	FMCLK(CKGEN,  63, "dramc_pll104m_ck"),
	FMCLK(ABIST,  1, "AD_WBG_DIG_CK_832M"),
	FMCLK(ABIST,  2, "AD_WBG_DIG_CK_960M"),
	FMCLK(ABIST,  3, "UFS_MP_CLK2FREQ"),
	FMCLK(ABIST,  4, "AD_CSI0A_CDPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  5, "AD_CSI0B_CDPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  6, "AD_CSI1A_DPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  7, "AD_CSI1B_DPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  8, "AD_CSI2A_DPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  9, "AD_CSI2B_DPHY_DELAYCAL_CK"),
	FMCLK(ABIST,  10, "AD_MDBPIPLL_CK"),
	FMCLK(ABIST,  11, "AD_MDBRPPLL_CK"),
	FMCLK(ABIST,  12, "AD_MDMCUPLL_CK"),
	FMCLK(ABIST,  13, "AD_MDTXPLL_CK"),
	FMCLK(ABIST,  14, "AD_MDVDSPPLL_CK"),
	FMCLK(ABIST,  16, "AD_MDPLL_FS26M_CK"),
	FMCLK(ABIST,  20, "AD_ARMPLL_L_CK"),
	FMCLK(ABIST,  22, "AD_ARMPLL_LL_CK"),
	FMCLK(ABIST,  23, "AD_MAINPLL_1092M_CK"),
	FMCLK(ABIST,  24, "AD_UNIVPLL_1248M_CK"),
	FMCLK(ABIST,  25, "AD_MFGPLL_CK"),
	FMCLK(ABIST,  26, "AD_MSDCPLL_CK"),
	FMCLK(ABIST,  27, "AD_MMPLL_CK"),
	FMCLK(ABIST,  28, "AD_APLL1_CK"),
	FMCLK(ABIST,  29, "AD_APLL2_CK"),
	FMCLK(ABIST,  30, "AD_APPLLGP_TST_CK"),
	FMCLK(ABIST,  32, "AD_UNIV_192M_CK"),
	FMCLK(ABIST,  34, "AD_TVDPLL_CK"),
	FMCLK(ABIST,  35, "AD_DSI0_MPPLL_TST_CK"),
	FMCLK(ABIST,  36, "AD_DSI0_LNTC_DSICLK"),
	FMCLK(ABIST,  37, "AD_OSC_CK_2"),
	FMCLK(ABIST,  38, "AD_OSC_CK"),
	FMCLK(ABIST,  39, "rtc32k_ck_i"),
	FMCLK(ABIST,  40, "mcusys_arm_clk_out_all"),
	FMCLK(ABIST,  41, "AD_OSC_SYNC_CK"),
	FMCLK(ABIST,  42, "AD_OSC_SYNC_CK_2"),
	FMCLK(ABIST,  43, "msdc01_in_ck"),
	FMCLK(ABIST,  44, "msdc02_in_ck"),
	FMCLK(ABIST,  45, "msdc11_in_ck"),
	FMCLK(ABIST,  46, "msdc12_in_ck"),
	FMCLK(ABIST,  49, "AD_CCIPLL_CK"),
	FMCLK(ABIST,  50, "AD_MPLL_208M_CK"),
	FMCLK(ABIST,  51, "AD_WBG_DIG_CK_CK_416M"),
	FMCLK(ABIST,  52, "AD_WBG_B_DIG_CK_64M"),
	FMCLK(ABIST,  53, "AD_WBG_W_DIG_CK_160M"),
	FMCLK(ABIST,  55, "DA_UNIV_48M_DIV_CK"),
	FMCLK(ABIST,  57, "DA_MPLL_52M_DIV_CK"),
	FMCLK(ABIST,  60, "ckmon1_ck"),
	FMCLK(ABIST,  61, "ckmon2_ck"),
	FMCLK(ABIST,  62, "ckmon3_ck"),
	FMCLK(ABIST,  63, "ckmon4_ck"),
	{}
};

#define CLK_MISC_CFG_0	(rb[topckgen].virt + 0x104)
#define CLK_DBG_CFG		(rb[topckgen].virt + 0x10C)
#define CLK26CALI_0		(rb[topckgen].virt + 0x220)
#define CLK26CALI_1		(rb[topckgen].virt + 0x224)

static unsigned int mt_get_ckgen_freq(unsigned int ID)
{
	int output = 0, i = 0;
	unsigned int temp, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1 = 0;

	clk_dbg_cfg = clk_readl(CLK_DBG_CFG);
	clk_writel(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFC0FC)|(ID << 8)|(0x1));

	clk_misc_cfg_0 = clk_readl(CLK_MISC_CFG_0);
	clk_writel(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));

	clk26cali_1 = clk_readl(CLK26CALI_1);
	clk_writel(CLK26CALI_0, 0x1000);
	clk_writel(CLK26CALI_0, 0x1010);

	while (clk_readl(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = clk_readl(CLK26CALI_1) & 0xFFFF;

	output = (temp * 26000) / 1024;

	clk_writel(CLK_DBG_CFG, clk_dbg_cfg);
	clk_writel(CLK_MISC_CFG_0, clk_misc_cfg_0);

	if (i > 10)
		return 0;
	else
		return output;

}

static unsigned int mt_get_abist_freq(unsigned int ID)
{
	int output = 0, i = 0;
	unsigned int temp, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1 = 0;

	clk_dbg_cfg = clk_readl(CLK_DBG_CFG);
	clk_writel(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFC0FFFC)|(ID << 16));

	clk_misc_cfg_0 = clk_readl(CLK_MISC_CFG_0);
	clk_writel(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (1 << 24));

	clk26cali_1 = clk_readl(CLK26CALI_1);

	clk_writel(CLK26CALI_0, 0x1000);
	clk_writel(CLK26CALI_0, 0x1010);

	while (clk_readl(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = clk_readl(CLK26CALI_1) & 0xFFFF;

	output = (temp * 26000) / 1024;

	clk_writel(CLK_DBG_CFG, clk_dbg_cfg);
	clk_writel(CLK_MISC_CFG_0, clk_misc_cfg_0);

	if (i > 10)
		return 0;
	else
		return (output * 2);
}

static u32 fmeter_freq_op(const struct fmeter_clk *fclk)
{
	if (fclk->type == ABIST)
		return mt_get_abist_freq(fclk->id);
	else if (fclk->type == CKGEN)
		return mt_get_ckgen_freq(fclk->id);
	return 0;
}

static const struct fmeter_clk *get_all_fmeter_clks(void)
{
	return fclks;
}

/*
 * clkdbg dump_state
 */

static const char * const *get_all_clk_names(void)
{
	static const char * const clks[] = {
		"mainpll",
		"univpll",
		"msdcpll",
		"mmpll",
		"mfgpll",
		"tvdpll",
		"apll1",
		"apll2",
		"apmixed_ssusb26m",
		"apmixed_appll26m",
		"apmixed_mipic026m",
		"apmixed_mdpll26m",
		"apmixed_mmsys26m",
		"apmixed_ufs26m",
		"apmixed_mipic126m",
		"apmixed_mempll26m",
		"apmixed_lvpll26m",
		"apmixed_mipid026m",
		"apmixed_mipid126m",
		"syspll_ck",
		"syspll_d2",
		"syspll_d3",
		"syspll_d5",
		"syspll_d7",
		"syspll_d2_d2",
		"syspll_d2_d4",
		"syspll_d2_d8",
		"syspll_d2_d16",
		"syspll_d3_d2",
		"syspll_d3_d4",
		"syspll_d3_d8",
		"syspll_d5_d2",
		"syspll_d5_d4",
		"syspll_d7_d2",
		"syspll_d7_d4",
		"univpll_ck",
		"univpll_d2",
		"univpll_d3",
		"univpll_d5",
		"univpll_d7",
		"univpll_d2_d2",
		"univpll_d2_d4",
		"univpll_d2_d8",
		"univpll_d3_d2",
		"univpll_d3_d4",
		"univpll_d3_d8",
		"univpll_d5_d2",
		"univpll_d5_d4",
		"univpll_d5_d8",
		"apll1_ck",
		"apll1_d2",
		"apll1_d4",
		"apll1_d8",
		"apll2_ck",
		"apll2_d2",
		"apll2_d4",
		"apll2_d8",
		"tvdpll_ck",
		"tvdpll_d2",
		"tvdpll_d4",
		"tvdpll_d8",
		"tvdpll_d16",
		"msdcpll_ck",
		"msdcpll_d2",
		"msdcpll_d4",
		"msdcpll_d8",
		"msdcpll_d16",
		"ad_osc_ck",
		"osc_d2",
		"osc_d4",
		"osc_d8",
		"osc_d16",
		"csw_f26m_ck_d2",
		"mfgpll_ck",
		"univ_192m_ck",
		"univ_192m_d2",
		"univ_192m_d4",
		"univ_192m_d8",
		"univ_192m_d16",
		"univ_192m_d32",
		"mmpll_ck",
		"mmpll_d4",
		"mmpll_d4_d2",
		"mmpll_d4_d4",
		"mmpll_d5",
		"mmpll_d5_d2",
		"mmpll_d5_d4",
		"mmpll_d6",
		"mmpll_d7",
		"f_f26m_ck",
		"clk13m",
		"osc",
		"univpll_192m",
		"axi_sel",
		"mm_sel",
		"cam_sel",
		"mfg_sel",
		"camtg_sel",
		"uart_sel",
		"spi_sel",
		"msdc50_hclk_sel",
		"msdc50_0_sel",
		"msdc30_1_sel",
		"msdc30_2_sel",
		"audio_sel",
		"aud_intbus_sel",
		"fpwrap_ulposc_sel",
		"scp_sel",
		"atb_sel",
		"sspm_sel",
		"dpi0_sel",
		"scam_sel",
		"aud_1_sel",
		"aud_2_sel",
		"disppwm_sel",
		"ssusb_top_xhci_sel",
		"usb_top_sel",
		"spm_sel",
		"i2c_sel",
		"f52m_mfg_sel",
		"seninf_sel",
		"dxcc_sel",
		"camtg2_sel",
		"aud_eng1_sel",
		"aud_eng2_sel",
		"faes_ufsfde_sel",
		"fufs_sel",
		"img_sel",
		"dsp_sel",
		"dsp1_sel",
		"dsp2_sel",
		"ipu_if_sel",
		"camtg3_sel",
		"camtg4_sel",
		"pmicspi_sel",
		"infra_pmic_tmr",
		"infra_pmic_ap",
		"infra_pmic_md",
		"infra_pmic_conn",
		"infra_scp",
		"infra_sej",
		"infra_apxgpt",
		"infra_icusb",
		"infra_gce",
		"infra_therm",
		"infra_i2c0",
		"infra_i2c1",
		"infra_i2c2",
		"infra_i2c3",
		"infra_pwm_hclk",
		"infra_pwm1",
		"infra_pwm2",
		"infra_pwm3",
		"infra_pwm4",
		"infra_pwm",
		"infra_uart0",
		"infra_uart1",
		"infra_uart2",
		"infra_uart3",
		"infra_gce_26m",
		"infra_cqdma_fpc",
		"infra_btif",
		"infra_spi0",
		"infra_msdc0",
		"infra_msdc1",
		"infra_msdc2",
		"infra_msdc0_sck",
		"infra_dvfsrc",
		"infra_gcpu",
		"infra_trng",
		"infra_auxadc",
		"infra_cpum",
		"infra_ccif1_ap",
		"infra_ccif1_md",
		"infra_auxadc_md",
		"infra_msdc1_sck",
		"infra_msdc2_sck",
		"infra_apdma",
		"infra_xiu",
		"infra_device_apc",
		"infra_ccif_ap",
		"infra_debugsys",
		"infra_audio",
		"infra_ccif_md",
		"infra_dxcc_sec_core",
		"infra_dxcc_ao",
		"infra_dramc_f26m",
		"infra_irtx",
		"infra_disppwm",
		"infra_cldma_bclk",
		"infra_audio_26m_bclk",
		"infra_spi1",
		"infra_i2c4",
		"infra_md_tmp_share",
		"infra_spi2",
		"infra_spi3",
		"infra_unipro_sck",
		"infra_unipro_tick",
		"infra_ufs_mp_sap_bck",
		"infra_md32_bclk",
		"infra_sspm",
		"infra_unipro_mbist",
		"infra_sspm_bus_hclk",
		"infra_i2c5",
		"infra_i2c5_arbiter",
		"infra_i2c5_imm",
		"infra_i2c1_arbiter",
		"infra_i2c1_imm",
		"infra_i2c2_arbiter",
		"infra_i2c2_imm",
		"infra_spi4",
		"infra_spi5",
		"infra_cqdma",
		"infra_ufs",
		"infra_aes_ufsfde",
		"infra_ufs_tick",
		"infra_msdc0_self",
		"infra_msdc1_self",
		"infra_msdc2_self",
		"infra_sspm_26m_self",
		"infra_sspm_32k_self",
		"infra_ufs_axi",
		"infra_i2c6",
		"infra_ap_msdc0",
		"infra_md_msdc0",
		"infra_usb",
		"infra_devmpu_bclk",
		"infra_ccif2_ap",
		"infra_ccif2_md",
		"infra_ccif3_ap",
		"infra_ccif3_md",
		"infra_sej_f13m",
		"infra_aes_bclk",
		"infra_i2c7",
		"infra_i2c8",
		"infra_fbist2fpc",
		"aud_tml",
		"aud_dac_predis",
		"aud_dac",
		"aud_adc",
		"aud_apll_tuner",
		"aud_apll2_tuner",
		"aud_24m",
		"aud_22m",
		"aud_afe",
		"aud_i2s4",
		"aud_i2s3",
		"aud_i2s2",
		"aud_i2s1",
		"aud_pdn_adda6_adc",
		"mfg_bg3d",
		"mm_smi_common",
		"mm_smi_larb0",
		"mm_smi_larb1",
		"mm_gals_comm0",
		"mm_gals_comm1",
		"mm_gals_ccu2mm",
		"mm_gals_ipu12mm",
		"mm_gals_img2mm",
		"mm_gals_cam2mm",
		"mm_gals_ipu2mm",
		"mm_mdp_dl_txck",
		"mm_ipu_dl_txck",
		"mm_mdp_rdma0",
		"mm_mdp_rdma1",
		"mm_mdp_rsz0",
		"mm_mdp_rsz1",
		"mm_mdp_tdshp",
		"mm_mdp_wrot0",
		"mm_fake_eng",
		"mm_disp_ovl0",
		"mm_disp_ovl0_2l",
		"mm_disp_ovl1_2l",
		"mm_disp_rdma0",
		"mm_disp_rdma1",
		"mm_disp_wdma0",
		"mm_disp_color0",
		"mm_disp_ccorr0",
		"mm_disp_aal0",
		"mm_disp_gamma0",
		"mm_disp_dither0",
		"mm_disp_split",
		"mm_dsi0_mm",
		"mm_dsi0_if",
		"mm_dpi_mm",
		"mm_dpi_if",
		"mm_fake_eng2",
		"mm_mdp_dl_rx",
		"mm_ipu_dl_rx",
		"mm_26m",
		"mm_mmsys_r2y",
		"mm_disp_rsz",
		"mm_mdp_wdma0",
		"mm_mdp_aal",
		"mm_mdp_ccorr",
		"mm_dbi_mm",
		"mm_dbi_if",
		"vdec_vdec",
		"vdec_larb1",
		"venc_larb",
		"venc_venc",
		"venc_jpgenc",
		"img_owe",
		"img_wpe_b",
		"img_wpe_a",
		"img_mfb",
		"img_rsc",
		"img_dpe",
		"img_fdvt",
		"img_dip",
		"img_larb2",
		"img_larb5",
		"cam_larb6",
		"cam_dfp_vad",
		"cam_cam",
		"cam_camtg",
		"cam_seninf",
		"cam_camsv0",
		"cam_camsv1",
		"cam_camsv2",
		"cam_ccu",
		"cam_larb3",
		"ipu_conn_ipu",
		"ipu_conn_ahb",
		"ipu_conn_axi",
		"ipu_conn_isp",
		"ipu_conn_cam_adl",
		"ipu_conn_img_adl",
		"ipu_conn_dap_rx",
		"ipu_conn_apb2axi",
		"ipu_conn_apb2ahb",
		"ipu_conn_ipu_cab1to2",
		"ipu_conn_ipu1_cab1to2",
		"ipu_conn_ipu2_cab1to2",
		"ipu_conn_cab3to3",
		"ipu_conn_cab2to1",
		"ipu_conn_cab3to1_slice",
		"ipu_adl_cabgen",
		"ipu_core0_jtag",
		"ipu_core0_axi",
		"ipu_core0_ipu",
		"ipu_core1_jtag",
		"ipu_core1_axi",
		"ipu_core1_ipu",
		/* end */
		NULL
	};

	return clks;
}

/*
 * clkdbg pwr_status
 */

static const char * const *get_pwr_names(void)
{
	static const char * const pwr_names[] = {
		[0]  = "MD1",
		[1]  = "CONN",
		[2]  = "DDRPHY",
		[3]  = "DISP",
		[4]  = "MFG",
		[5]  = "ISP",
		[6]  = "INFRA",
		[7]  = "MFG_CORE0",
		[8]  = "MP0_CPUTOP",
		[9]  = "MP0_CPU0",
		[10] = "MP0_CPU1",
		[11] = "MP0_CPU2",
		[12] = "MP0_CPU3",
		[13] = "",
		[14] = "",
		[15] = "",
		[16] = "",
		[17] = "",
		[18] = "",
		[19] = "",
		[20] = "MFG_CORE1",
		[21] = "VENC",
		[22] = "MFG_2D",
		[23] = "MFG_ASYNC",
		[24] = "AUDIO",
		[25] = "CAM",
		[26] = "VPU_TOP",
		[27] = "VPU_CORE0",
		[28] = "VPU_CORE1",
		[29] = "VPU_CORE2",
		[30] = "",
		[31] = "VDEC",
	};

	return pwr_names;
}

u32 get_spm_pwr_status(void)
{
	static void __iomem *scpsys_base, *pwr_sta, *pwr_sta_2nd;

	if (scpsys_base == NULL || pwr_sta == NULL || pwr_sta_2nd == NULL) {
		scpsys_base = ioremap(0x10006000, PAGE_SIZE);
		pwr_sta = scpsys_base + 0x180;
		pwr_sta_2nd = scpsys_base + 0x184;
	}

	return clk_readl(pwr_sta) & clk_readl(pwr_sta_2nd);
}

/*
 * clkdbg dump_clks
 */

static void setup_provider_clk(struct provider_clk *pvdck)
{
	static const struct {
		const char *pvdname;
		u32 pwr_mask;
	} pvd_pwr_mask[] = {
	};

	size_t i;
	const char *pvdname = pvdck->provider_name;

	if (pvdname == NULL)
		return;

	for (i = 0; i < ARRAY_SIZE(pvd_pwr_mask); i++) {
		if (strcmp(pvdname, pvd_pwr_mask[i].pvdname) == 0) {
			pvdck->pwr_mask = pvd_pwr_mask[i].pwr_mask;
			return;
		}
	}
}

/*
 * init functions
 */

static struct clkdbg_ops clkdbg_mt8183_ops = {
	.get_all_fmeter_clks = get_all_fmeter_clks,
	.fmeter_freq = fmeter_freq_op,
	.get_all_regnames = get_all_regnames,
	.get_all_clk_names = get_all_clk_names,
	.get_pwr_names = get_pwr_names,
	.setup_provider_clk = setup_provider_clk,
	.get_spm_pwr_status = get_spm_pwr_status,
};

static void __init init_custom_cmds(void)
{
	static const struct cmd_fn cmds[] = {
		{}
	};

	set_custom_cmds(cmds);
}

static int __init clkdbg_mt8183_init(void)
{
	if (of_machine_is_compatible("mediatek,mt8183") == 0)
		return -ENODEV;

	init_regbase();

	init_custom_cmds();
	set_clkdbg_ops(&clkdbg_mt8183_ops);

#if DUMP_INIT_STATE
	print_regs();
	print_fmeter_all();
#endif /* DUMP_INIT_STATE */

	return 0;
}
device_initcall(clkdbg_mt8183_init);

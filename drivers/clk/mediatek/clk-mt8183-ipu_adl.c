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
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt8183-clk.h>

static const struct mtk_gate_regs ipu_adl_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

#define GATE_IPU_ADL_I(_id, _name, _parent, _shift) {	\
		.id = _id,			\
		.name = _name,			\
		.parent_name = _parent,		\
		.regs = &ipu_adl_cg_regs,		\
		.shift = _shift,		\
		.ops = &mtk_clk_gate_ops_no_setclr_inv,	\
	}

static const struct mtk_gate ipu_adl_clks[] = {
	GATE_IPU_ADL_I(CLK_IPU_ADL_CABGEN, "ipu_adl_cabgen", "dsp_sel", 24),
};

static int clk_mt8183_ipu_adl_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	int r;
	struct device_node *node = pdev->dev.of_node;

	clk_data = mtk_alloc_clk_data(CLK_IPU_ADL_NR_CLK);

	mtk_clk_register_gates(node, ipu_adl_clks, ARRAY_SIZE(ipu_adl_clks),
			clk_data);

	r = of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);

	if (r)
		pr_err("%s(): could not register clock provider: %d\n",
			__func__, r);

	return r;
}

static const struct of_device_id of_match_clk_mt8183_ipu_adl[] = {
	{ .compatible = "mediatek,mt8183-ipu_adl", },
	{}
};

static struct platform_driver clk_mt8183_ipu_adl_drv = {
	.probe = clk_mt8183_ipu_adl_probe,
	.driver = {
		.name = "clk-mt8183-ipu_adl",
		.of_match_table = of_match_clk_mt8183_ipu_adl,
	},
};

builtin_platform_driver(clk_mt8183_ipu_adl_drv);

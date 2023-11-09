/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <mt_typec.h>


struct mt6392_typecreg_set {
	unsigned int offset;
	const char *name;
	unsigned int width;
	const char *comment;
};


/*
 * Module name: MT6392 Register Type-C address: (+0800h)
 * Address	Name	Width		Register Function
 */
static struct mt6392_typecreg_set mt6392_typec_registers[] = {
	{ 0x00, "TYPE_C_PHY_RG_0",				 16, NULL },			 
	{ 0x02, "TYPE_C_PHY_RG_CC_RESERVE_CSR",	 16, NULL },	
	{ 0x04, "TYPE_C_VCMP_CTRL",				 16, NULL }, 
	{ 0x06, "TYPE_C_CTRL",					 16, NULL },
	{ 0x0a, "TYPE_C_CC_SW_CTRL",				 16, NULL },
	{ 0x0c, "TYPE_C_CC_VOL_PERIODIC_MEAS_VAL", 16, NULL },	
	{ 0x0e, "TYPE_C_CC_VOL_DEBOUCE_CNT_VAL",	 16, NULL },
	{ 0x10, "TYPE_C_DRP_SRC_CNT_VAL_0",		 16, NULL },	
	{ 0x14, "TYPE_C_DRP_SNK_CNT_VAL_0",		 16, NULL },	
	{ 0x18, "TYPE_C_DRP_TRY_CNT_VAL_0",		 16, NULL },	
	{ 0x20, "TYPE_C_CC_SRC_DEFAULT_DAC_VAL",	 16, NULL },
	{ 0x22, "TYPE_C_CC_SRC_15_DAC_VAL",		 16, NULL },
	{ 0x24, "TYPE_C_CC_SRC_30_DAC_VAL",		 16, NULL },
	{ 0x28, "TYPE_C_CC_SNK_DAC_VAL_0",		 16, NULL },
	{ 0x2a, "TYPE_C_CC_SNK_DAC_VAL_1",		 16, NULL },	
	{ 0x30, "TYPE_C_INTR_EN_0",				 16, NULL }, 
	{ 0x34, "TYPE_C_INTR_EN_2",				 16, NULL },
	{ 0x38, "TYPE_C_INTR_0",					 16, NULL }, 
	{ 0x3C, "TYPE_C_INTR_2",					 16, NULL }, 
	{ 0x40, "TYPE_C_CC_STATUS",				 16, NULL }, 
	{ 0x42, "TYPE_C_PWR_STATUS",				 16, NULL },	
	{ 0x44, "TYPE_C_PHY_RG_CC1_RESISTENCE_0",  16, NULL }, 
	{ 0x46, "TYPE_C_PHY_RG_CC1_RESISTENCE_1",  16, NULL },
	{ 0x48, "TYPE_C_PHY_RG_CC2_RESISTENCE_0",  16, NULL },
	{ 0x4a, "TYPE_C_PHY_RG_CC2_RESISTENCE_1",  16, NULL }, 
	{ 0x60, "TYPE_C_CC_SW_FORCE_MODE_ENABLE_0",16, NULL }, 
	{ 0x64, "TYPE_C_CC_SW_FORCE_MODE_VAL_0",	 16, NULL }, 
	{ 0x66, "TYPE_C_CC_SW_FORCE_MODE_VAL_1",	 16, NULL },	
	{ 0x68, "TYPE_C_CC_SW_FORCE_MODE_ENABLE_1",16, NULL }, 
	{ 0x6c, "TYPE_C_CC_SW_FORCE_MODE_VAL_2",	 16, NULL },
	{ 0x70, "TYPE_C_CC_DAC_CALI_CTRL",		 16, NULL },
	{ 0x72, "TYPE_C_CC_DAC_CALI_RESULT",		 16, NULL },
	{ 0x80, "TYPE_C_DEBUG_PORT_SELECT_0", 	 16, NULL },	
	{ 0x82, "TYPE_C_DEBUG_PORT_SELECT_1", 	 16, NULL },	
	{ 0x84, "TYPE_C_DEBUG_MODE_SELECT",		 16, NULL },	
	{ 0x88, "TYPE_C_DEBUG_OUT_READ_0",		 16, NULL },	
	{ 0x8a, "TYPE_C_DEBUG_OUT_READ_1",		 16, NULL },	
	{ 0x8c, "TYPE_C_SW_DEBUG_PORT_0", 		 16, NULL },	
	{ 0x8e, "TYPE_C_SW_DEBUG_PORT_1", 		 16, NULL },
	{ 0, NULL, 0, NULL }
};

static inline unsigned int uffs(unsigned int x)
{
	unsigned int r = 1;

	if (!x)
		return 0;
	if (!(x & 0xffff)) {
		x >>= 16;
		r += 16;
	}
	if (!(x & 0xff)) {
		x >>= 8;
		r += 8;
	}
	if (!(x & 0xf)) {
		x >>= 4;
		r += 4;
	}
	if (!(x & 3)) {
		x >>= 2;
		 r += 2;
	}
	if (!(x & 1)) {
		x >>= 1;
		r += 1;
	}

	return r;
}

#define IO_SET_FIELD(hba, reg, field, val) \
		do { \
			unsigned int value; \
			regmap_read(hba->regmap, reg, &value); \
			value &= ~(field); \
			value |= ((val) << (uffs((unsigned int)field) - 1)); \
			regmap_write(hba->regmap, reg, value); \
		} while (0)

#define IO_GET_FIELD(hba, reg, field, val) \
		do { \
			unsigned int value; \
			regmap_read(hba->regmap, reg, &value); \
			val = ((value & (field)) >> (uffs((unsigned int)field) - 1)); \
		} while (0)

static void register_set_field(struct typec_hba *hba, unsigned int address, unsigned int start_bit,
						unsigned int len, unsigned int value)
{
	unsigned long field;
	unsigned int read;

	if (   (0 <= start_bit && start_bit < 16)
		&& ( 0 < len && len <= 16) 
		&& ((start_bit + len) < 16)) {
		
		field = ((1 << len) - 1) << start_bit;
		value &= (1 << len) - 1;

		regmap_read(hba->regmap, address, &read);
		pr_err("[typec debug][Register RMW]Original:0x%08X (0x%x)\n", address, read);
		
		IO_SET_FIELD(hba, address, field, value);
		
		regmap_read(hba->regmap, address, &read);
		pr_err("[typec debug][Register RMW]Modified:0x%08X (0x%x)\n", address, read);
	} else 
		pr_err("[typec debug][Register RMW] Invalid Register field range or length\n");
}

static void register_get_field(struct typec_hba *hba, unsigned int address, unsigned int start_bit,
						unsigned int len, unsigned int value)
{
	unsigned long field;

	if (   (0 <= start_bit && start_bit < 16)
		&& ( 0 < len && len <= 16) 
		&& ((start_bit + len) < 16)) {
		
		field = ((1 << len) - 1) << start_bit;
		IO_GET_FIELD(hba, address, field, value);
		pr_err("[typec debug][Register RMW]Reg:0x%08X start_bit(%d)len(%d)(0x%x)\n",
								address, start_bit, len, value);
	} else
		pr_err("[typec debug][Register RMW]Invalid reg field range or length\n");
}

/* Add verbose debugging later, just print everything for now */
static int mt6392_typec_regdump_show(struct seq_file *s, void *unused)
{
	int i;
	struct mt6392_typecreg_set *typec_reg;
	struct typec_hba *hba = s->private;
	unsigned int typec_base = hba->addr_base;
	unsigned int read;

	seq_printf(s, "MT6392(Type-C_base: 0x%08X)\n", typec_base);
	if (typec_base) {
		seq_puts(s, "\nmt6392_typec Register Dump range [0x0800 ~ 0x0900)\n");
		for (i = 0; i < ARRAY_SIZE(mt6392_typec_registers); i++) {
			typec_reg = &mt6392_typec_registers[i];
			switch (typec_reg->width) {
			case 8:
				regmap_read(hba->regmap, typec_base + typec_reg->offset, &read);
				seq_printf(s, "%-32s(0x%08X]: offset: 0x%02X: 0x%02X\n", typec_reg->name, typec_base + typec_reg->offset,
									typec_reg->offset, (unsigned char)read);
				break;
			case 16:
				regmap_read(hba->regmap, typec_base + typec_reg->offset, &read);
				seq_printf(s, "%-32s(0x%08X): offset: 0x%02X: 0x%04X\n", typec_reg->name, typec_base + typec_reg->offset,
									typec_reg->offset, (unsigned short)read);
				break;
			}
		}
	}

	return 0;
}

static int mt6392_typec_regdump(struct inode *inode, struct file *file)
{
	return single_open(file, mt6392_typec_regdump_show, inode->i_private);
}

static const struct file_operations mt6392_typec_regdump_fops = {
	.open	= mt6392_typec_regdump,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static int mt6392_typec_debug_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "\n===mt6392_typec_debug help===\n");
	seq_puts(m, "\n   LOG control:        echo 0 [debug_zone] > debug\n");

	seq_puts(m, "\n   REGISTER control usage:\n");
	seq_puts(m, "       write register:   echo 1 0 [io_addr] [value] > debug\n");
	seq_puts(m, "       read  register:   echo 1 1 [io_addr] > debug\n");
	seq_puts(m, "       write mask:       echo 1 2 [io_addr] [start_bit] [len] [value] > debug\n");
	seq_puts(m, "       read  mask:       echo 1 3 [io_addr] [start_bit] [len] > debug\n");
	seq_puts(m, "       dump all regiters echo 1 4 > debug\n");
	seq_puts(m, "=========================================\n\n");

	return 0;
}

static ssize_t mt6392_typec_debug_proc_write(struct file *file, const char *buf, size_t count, loff_t *data)
{
	int ret;
	int cmd, p1, p3, p4, p5;
	unsigned int reg_value;
	int sscanf_num;
	struct seq_file	*s = file->private_data;
	struct typec_hba *hba = s->private;
	unsigned int iomem = 0;
	unsigned int value;
	unsigned int long long p2;

	p1 = p2 = p3 = p4 = p5 = -1;

	if (count == 0)
		return -1;

	if (count > 255)
		count = 255;

	ret = copy_from_user(hba->cmd_buf, buf, count);
	if (ret < 0)
		return -1;

	hba->cmd_buf[count] = '\0';
	pr_err("[typec debug]debug received:%s\n", hba->cmd_buf);

	sscanf_num = sscanf(hba->cmd_buf, "%x %x %llx %x %x %x", &cmd, &p1, &p2, &p3, &p4, &p5);
	if (sscanf_num < 1)
		return count;

	if (cmd == 0)
		pr_err("[typec debug] zone <0x%.8x> is not exist yet\n", p1);
	else if (cmd == 1) {
		iomem += p2;
		if (p1 == 0) {
			reg_value = p3;
			
			regmap_read(hba->regmap, iomem, &value);
			pr_err("[typec debug][Register Write]Original:0x%08X (0x%08X)\n",
								iomem, value);

			regmap_write(hba->regmap, iomem, reg_value);

			regmap_read(hba->regmap, iomem, &value);
			pr_err("[typec debug][Register Write]Writed:0x%08X (0x%08X)\n",
								iomem, value);
		} else if (p1 == 1) {
			regmap_read(hba->regmap, iomem, &value);
			pr_err("[typec debug][Register Read]Register:0x%08X (0x%08X)\n",
								iomem, value);
		}
		else if (p1 == 2)
			register_set_field(hba, iomem, p3, p4, p5);
		else if (p1 == 3)
			register_get_field(hba, iomem, p3, p4, p5);
		else
			pr_err("[typec debug] todo\n");
	}

	return count;
}

static int mt6392_typec_debug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt6392_typec_debug_proc_show, inode->i_private);
}

static const struct file_operations mt6392_typec_debug_proc_fops = {
	.open   = mt6392_typec_debug_proc_open,
	.write  = mt6392_typec_debug_proc_write,
	.read   = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt6392_typec_proc_show(struct seq_file *s, void *v)
{
	struct typec_hba *hba  = s->private;
	unsigned short cc_status;
	unsigned short cc_ctrl;
	unsigned int   route=0;
	unsigned int   state;
	unsigned int   vbus;
	unsigned int   vbus_value;
	
	static char *szmt6392_typec_state[]= {
		"Disable",
		"Unattached Source",
		"Attach Wait Source",
		"Attach Source",
		"Unatched Sink",
		"Attach Wait Sink",
		"Attach Sink",
		"Try Source",
		"Try Wait Sink",
		"Unattached Accessory",
		"Attach Wait Accessory",
		"Audio Accessory",
		"Debug Accessory",
		"Unused"
	};
	static char *szmt6392_typec_cc[]= {
		"unknown",
		"  CC1  ",
		"  CC2  ",
	};

	seq_puts(s, "\n===mt6392 Type-C debug information===\n");
	seq_puts(s, "===Table 4-27 Voltage on Sink CC pins===\n");
	seq_puts(s, "  Detection  |Min Volt(V)|Max Volt(V)|Threshold(V)\n");
	seq_puts(s, "-------------+-----------+------------------------\n");
	seq_puts(s, " vRa         |  -0.25    |    0.15   | 0.2 \n");
	seq_puts(s, " vRd-Connect |   0.25    |    2.04   |     \n");
	seq_puts(s, " vRd-USB     |   0.25    |    0.61   | 0.66\n");
	seq_puts(s, " vRd-1.5     |   0.70    |    1.16   | 1.23\n");
	seq_puts(s, " vRd-3.0     |   1.31    |    2.04   |     \n");
	seq_puts(s, "-------------+-----------+-----------+------------\n");

	seq_puts(s, "\n===Table 4-7 Source Perspective===\n");
	seq_puts(s, "  CC1 |  CC2  |    State     | Position\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Open | Open  | Nothing      | N/A\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Rd   | Open  | Sink         | cc1\n");
	seq_puts(s, " Open | Rd    | Sink         | cc2\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Open | Ra    | Powered With | cc1\n");
	seq_puts(s, " Ra   | Open  | -out Sink    | cc2\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Rd   | Ra    | Powered with | cc1\n");
	seq_puts(s, " Ra   | Rd    | Sink/(ACC))  | cc2\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Rd   | Rd    | Debug(ACC)   | N/A\n");
	seq_puts(s, "------+-------+--------------+---------\n");
	seq_puts(s, " Ra   | Ra    | Audio(ACC)   | N/A\n");
	seq_puts(s, "------+-------+--------------+----------\n");	


	cc_status = typec_readw(hba, TYPE_C_CC_STATUS);
	state     = (cc_status & RO_TYPE_C_CC_ST);
	state     = (state > 12) ? 13: state;
    if (state == 3 || state == 6)
		route     = (cc_status & RO_TYPE_C_ROUTED_CC) ? 1 : 2;
	
	cc_ctrl   = typec_readw(hba, TYPE_C_CC_SW_CTRL);
	vbus      = (cc_ctrl & TYPE_C_SW_VBUS_PRESENT) ? 1 : 0;

	seq_puts(s, "\n===MT6392 Type-C status===\n");
	seq_puts(s, "-------+-----------+---------+------------\n");	
	seq_puts(s, " Route |  H/W vBus |   VBUS  | State (CC)\n");
	seq_puts(s, "-------+-----------+---------+------------\n");
#if IS_ENABLED(CONFIG_TCPC_MT6392_VBUS_GPIO)
	vbus_value = !gpio_get_value(hba->typec_vbus_monitor_pin); //modify by kxz
	seq_printf(s, "%s|  %3u     | %3u     | %s\n", szmt6392_typec_cc[route], vbus, vbus_value, szmt6392_typec_state[state]);
#else
	vbus_value 	  = PMIC_IMM_GetOneChannelValue(2, 2, 1) * 10;
	seq_printf(s, "%s|   %3u     | %3u mV | %s\n", szmt6392_typec_cc[route], vbus, vbus_value, szmt6392_typec_state[state]);
#endif

	return 0;
}

static int mt6392_typec_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt6392_typec_proc_show, inode->i_private);
}

static const struct file_operations mt6392_typec_proc_fops = {
	.open   = mt6392_typec_proc_open,
	.read   = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int mt6392_typec_init_debugfs(struct typec_hba *hba)
{
	int	ret;
	struct dentry *file;
	struct dentry *root;

	root = debugfs_create_dir(dev_driver_string(hba->dev), NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}

	file = debugfs_create_file("regdump", S_IRUGO, root, hba, &mt6392_typec_regdump_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	file = debugfs_create_file("debug", S_IRUGO | S_IWUSR, root, hba, &mt6392_typec_debug_proc_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	file = debugfs_create_file("typec", S_IRUGO | S_IWUSR, root, hba, &mt6392_typec_proc_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	hba->debugfs_root = root;

	return 0;

err1:
	debugfs_remove_recursive(root);

err0:
	return ret;
}

void mt6392_typec_exit_debugfs(struct typec_hba *hba)
{
	debugfs_remove_recursive(hba->debugfs_root);
}

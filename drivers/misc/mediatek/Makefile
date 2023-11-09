#
# Copyright (C) 2018 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

#$(call all-subdir-src-or-makefile)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
subdir-ccflags-y += -I$(srctree)/drivers/mmc/host/mediatek/$(MTK_PLATFORM)

obj-$(CONFIG_MMPROFILE) += mmp/
obj-$(CONFIG_MTK_BASE_POWER) += base/
obj-$(CONFIG_MTK_PMIC_NEW_ARCH)	+= pmic/
obj-$(CONFIG_MTK_PSEUDO_M4U)   += pseudo_m4u/
obj-$(CONFIG_MTK_ICE_DEBUG) += ice_debug/
obj-y += mmdvfs/

obj-$(CONFIG_MTK_CCCI_DEVICES)	+=  ccci_util/
obj-$(CONFIG_MTK_NET_CCMNI) +=  ccmni/

obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_CONN_MD) += conn_md/

obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) +=masp/

obj-y += atf/
obj-y += devinfo/
obj-y += sspm/
obj-$(CONFIG_MTK_TINYSYS_MCUPM_SUPPORT) += mcupm/
obj-y += timer/
obj-$(CONFIG_MTK_GIC_EXT) += ext_gic/
obj-$(CONFIG_MTK_SYS_CIRQ) += cirq/
obj-$(CONFIG_MTK_SECURITY_SW_SUPPORT) +=masp/
obj-$(CONFIG_MTPROF)	+= mtprof/
obj-$(CONFIG_MEDIATEK_SOLUTION) += sched/
obj-$(CONFIG_MTK_FREQ_HOPPING) += freqhopping/
obj-$(CONFIG_MTK_GPU_SUPPORT) += gpu/
obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += mtee/
obj-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += gcpu/
obj-$(CONFIG_MTK_MEMORY_LOWPOWER) += memory-lowpower/
obj-$(CONFIG_MTK_MLOG) += mlog/
obj-$(CONFIG_MTK_GPT_SCHEME_SUPPORT)	+= partition/
obj-$(CONFIG_MTK_SENSORS_1_0) += sensors-1.0/
obj-$(CONFIG_MTK_SSMR) += memory-ssmr/
obj-$(CONFIG_MTK_AMMS) += memory-amms/
obj-$(CONFIG_ZONE_MOVABLE_CMA) += zone_movable_cma/
obj-$(CONFIG_MTK_BLOCK_TAG) += blocktag/
obj-$(CONFIG_MTK_LEDS)	+= leds/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_MTK_USBFSH) += usb11/
obj-$(CONFIG_MTK_BOOT) += boot/
obj-$(CONFIG_MTK_BOOT_REASON) += boot_reason/
obj-$(CONFIG_MTK_CHIP) += chip/
obj-$(CONFIG_MTK_SYNC) += sync/
obj-y += pidmap/
obj-$(CONFIG_MTK_RAM_CONSOLE) += ram_console/
obj-$(CONFIG_MEDIATEK_SOLUTION)	+= aee/
obj-$(CONFIG_MEDIATEK_SOLUTION) += sched/
obj-$(CONFIG_USB_MTK_HDRC) += usb20/
obj-$(CONFIG_USB_MU3D_DRV)  += mu3d/
obj-$(CONFIG_MTK_RTC)	+= rtc/
obj-$(CONFIG_MTK_XO)	+= xo/
obj-$(CONFIG_MTK_CMDQ) += cmdq/
obj-$(CONFIG_MTK_LCM) += lcm/
obj-$(CONFIG_MTK_LCM) += ext_disp/
obj-$(CONFIG_MTK_HDMI_SUPPORT) += hdmi/
obj-$(CONFIG_MTK_FB) += video/
obj-$(CONFIG_MTK_PMIC_WRAP_HAL) += pmic_wrap/
obj-$(CONFIG_RT_REGMAP) += rt-regmap/
obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += videocodec/

obj-$(CONFIG_MTK_CQDMA) += cqdma/

ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
obj-y += power/
else
obj-$(CONFIG_MACH_MT8167)+= power/
endif
obj-y += log_store/
obj-$(CONFIG_MTK_ECCCI_C2K) += c2k_usb/
obj-$(CONFIG_USB) += usb_boost/
obj-$(CONFIG_MTK_USB_TYPEC) += typec/
obj-$(CONFIG_THERMAL)   += thermal/
obj-$(CONFIG_MTK_MTD_NAND) += nand/
obj-y += performance/
obj-$(CONFIG_MEDIATEK_SOLUTION) += cmo/
obj-y += io_boost/
obj-y += mtprintk/
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/met_drv/),)
obj-y += met_drv/
endif
obj-y += perf/
obj-$(CONFIG_MTK_JPEG) += jpeg/

obj-$(CONFIG_TOUCHKEY_SUPPORT)  += touchkey/

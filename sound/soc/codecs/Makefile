# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include



snd-soc-mt6392-codec-objs := mt6392-codec.o
snd-soc-mt8167-codec-objs := mt8167-codec.o mt8167-codec-utils.o

obj-$(CONFIG_SND_SOC_RT5680)	+= rt5680/


ifeq ($(CONFIG_SND_SOC_MT6358),y)
  MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
  subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
  subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/mt6358
endif

ifeq ($(CONFIG_SND_SOC_MT6359),y)
  MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
  subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
  subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/accdet/mt6359
endif

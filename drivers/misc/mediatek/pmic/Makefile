subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include


obj-$(CONFIG_MTK_EXTBUCK) += extbuck/
obj-$(CONFIG_MTK_PMIC_COMMON) += common/


obj-$(CONFIG_MFD_RT5081_PMU)	+= rt5081/
obj-$(CONFIG_MFD_MT6370_PMU)    += mt6370/
obj-$(CONFIG_MFD_MT6360_PMU)    += mt6360/

ifneq (,$(findstring evb,$(CONFIG_ARCH_MTK_PROJECT)))
	subdir-ccflags-y += -DMTK_EVB_PLATFORM
endif

ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery
ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc
obj-y += mtk_battery_adc_intf.o
endif

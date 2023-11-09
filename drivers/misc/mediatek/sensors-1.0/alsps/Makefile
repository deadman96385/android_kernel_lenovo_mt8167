# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
obj-y   +=  alsps.o
obj-y   +=  alsps_factory.o
obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_ALSPSHUB)      +=  alspshub/
obj-$(CONFIG_GESTURE_APDS9500)	+=  apds9500/
obj-$(CONFIG_GESTURE_APDS9500_NEW)	+=  apds9500-new/
obj-$(CONFIG_MTK_ROHM_BH1749)	+=  bh1749/
obj-$(CONFIG_MTK_VCNL4200)		+=  vcnl4200/
obj-$(CONFIG_MTK_VCNL4200_NEW)	+=  vcnl4200-new/

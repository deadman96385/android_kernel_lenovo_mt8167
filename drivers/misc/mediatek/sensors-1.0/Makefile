ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-$(CONFIG_MTK_HWMON)	    += hwmon/
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
ifeq ($(CONFIG_CUSTOM_KERNEL_ACCELGYRO),y)
obj-y += accelgyro/
else
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
endif
#obj-$(CONFIG_CUSTOM_KERNEL_HUMIDITY) += humidity/
#obj-$(CONFIG_CUSTOM_KERNEL_PEDOMETER)	+= pedometer/
#obj-$(CONFIG_MTK_KERNEL_GEOFENCE) += geofence/
else
obj-y += dummy.o
endif

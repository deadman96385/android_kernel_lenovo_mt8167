# for TYPEC connection management
ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/inc

obj-$(CONFIG_TCPC_MT6392)		+= mt6392_typec.o

mt6392_typec-y					:= typec.o typec-pltfrm.o
mt6392_typec-$(CONFIG_DEBUG_FS) += typec-debug.o

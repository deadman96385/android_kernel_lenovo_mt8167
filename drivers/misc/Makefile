# SPDX-License-Identifier: GPL-2.0
#
# Makefile for misc devices that really don't fit anywhere else.
#


obj-y				+= mic/

obj-$(CONFIG_UID_SYS_STATS)	+= uid_sys_stats.o
obj-$(CONFIG_MEMORY_STATE_TIME)	+= memory_state_time.o
obj-$(CONFIG_MEDIATEK_SOLUTION) += mediatek/


KCOV_INSTRUMENT_lkdtm_rodata.o	:= n
CFLAGS_lkdtm_rodata.o		+= $(DISABLE_LTO)

OBJCOPYFLAGS :=
OBJCOPYFLAGS_lkdtm_rodata_objcopy.o := \
	--rename-section .text=.rodata,alloc,readonly,load
targets += lkdtm_rodata.o lkdtm_rodata_objcopy.o
$(obj)/lkdtm_rodata_objcopy.o: $(obj)/lkdtm_rodata.o FORCE
	$(call if_changed,objcopy)

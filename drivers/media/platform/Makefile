# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the video capture/playback device drivers.
#

obj-$(CONFIG_VIDEO_M32R_AR_M64278) += arv.o

obj-$(CONFIG_VIDEO_VIA_CAMERA) += via-camera.o

obj-$(CONFIG_VIDEO_PXA27x)	+= pxa_camera.o

obj-$(CONFIG_VIDEO_VIU) += fsl-viu.o

obj-$(CONFIG_VIDEO_VIMC)		+= vimc/
obj-$(CONFIG_VIDEO_VIVID)		+= vivid/
obj-$(CONFIG_VIDEO_VIM2M)		+= vim2m.o

obj-$(CONFIG_VIDEO_MX2_EMMAPRP)		+= mx2_emmaprp.o

obj-$(CONFIG_VIDEO_SH_VEU)		+= sh_veu.o

obj-$(CONFIG_VIDEO_MEM2MEM_DEINTERLACE)	+= m2m-deinterlace.o

obj-$(CONFIG_VIDEO_MUX)			+= video-mux.o

obj-$(CONFIG_VIDEO_SH_VOU)		+= sh_vou.o

obj-$(CONFIG_VIDEO_RCAR_DRIF)		+= rcar_drif.o
obj-$(CONFIG_VIDEO_RENESAS_FCP) 	+= rcar-fcp.o
obj-$(CONFIG_VIDEO_RENESAS_FDP1)	+= rcar_fdp1.o
obj-$(CONFIG_VIDEO_RENESAS_JPU) 	+= rcar_jpu.o
obj-$(CONFIG_VIDEO_RENESAS_VSP1)	+= vsp1/


obj-$(CONFIG_VIDEO_STM32_DCMI)		+= stm32/

ccflags-y += -I$(srctree)/drivers/media/i2c

obj-$(CONFIG_VIDEO_MEDIATEK_VPU)	+= mtk-vpu/

obj-$(CONFIG_VIDEO_MEDIATEK_VCU)	+= mtk-vcu/

obj-$(CONFIG_VIDEO_MEDIATEK_VCODEC)	+= mtk-vcodec/
obj-$(CONFIG_VB2_MEDIATEK_DMA_CONTIG)	+= mtk-videobuf/
obj-$(CONFIG_VIDEO_MEDIATEK_MDP)	+= mtk-mdp/

obj-$(CONFIG_VIDEO_MEDIATEK_JPEG)	+= mtk-jpeg/
obj-$(CONFIG_VIDEO_MEDIATEK_WARP)       += mtk-warp/coda

obj-$(CONFIG_VIDEO_NXP_OUT) += nxp-out.o nxp-mlc.o
obj-$(CONFIG_NXP_OUT_RESOLUTION_CONVERTER) += nxp-resc.o sync-preset.o
obj-$(CONFIG_NXP_OUT_HDMI) += hdmi/
obj-$(CONFIG_NXP_HDMI_CEC) += hdmi/cec/nxp-hdmi-cec.o
obj-$(CONFIG_NXP_OUT_TVOUT) += nxp-tvout.o

EXTRA_CFLAGS += -Iinclude
EXTRA_CFLAGS += -Idrivers/media/platform/nexell
EXTRA_CFLAGS += -Idrivers/media/platform/nexell/out/hdmi
EXTRA_CFLAGS += -I$(srctree)/drivers/platform/nexell/s5p6818/prototype/base -I$(srctree)/drivers/platform/nexell/s5p6818/prototype/module

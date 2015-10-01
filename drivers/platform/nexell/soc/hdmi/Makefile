obj-y = hdmi-priv.o hdmi-preset.o nxp-edid.o nxp-hdcp.o nxp-hdmiphy.o nxp-hdmi-context.o

ifeq ($(CONFIG_S5P6818_PROTO_RELEASE),y)
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_RELEASE
else
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_DEBUG
endif

EXTRA_CFLAGS += -Iinclude
EXTRA_CFLAGS += -Idrivers/media/platform/nexell
EXTRA_CFLAGS += -I$(srctree)/drivers/platform/nexell/s5p6818/prototype/base -I$(srctree)/drivers/platform/nexell/s5p6818/prototype/module

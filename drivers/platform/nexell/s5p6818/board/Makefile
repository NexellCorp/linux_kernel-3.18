#
# Copyright 2009 Nexell Co.
#	http://www.nexell.co.kr
#

PROTO_MODULE := drivers/platform/nexell/s5p6818/cpu/prototype/module
PROTO_BASE   := drivers/platform/nexell/s5p6818/cpu/prototype/base
KBUILD_CFLAGS += -I$(PROTO_MODULE) -I$(PROTO_BASE)

ifeq ($(CONFIG_S5P6818_PROTO_RELEASE),y)
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_RELEASE
else
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_DEBUG
endif

#===============================================================================
# Board Type
#===============================================================================
obj-$(CONFIG_ARCH_S5P6818_SVT) += svt/


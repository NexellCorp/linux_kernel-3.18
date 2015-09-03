#
# Copyright 2009 Nexell Co.
#	http://www.nexell.co.kr
#

obj-y += cpu_init_early.o cpu-sys.o
obj-y += board/

ifeq ($(CONFIG_ARM64),y)
obj-y += setup_64.o
obj-y += platsmp_64.o
obj-$(CONFIG_SMP) += headsmp_64.o
obj-$(CONFIG_SUSPEND) += pm_64.o
endif

PROTO_MODULE := drivers/platform/nexell/s5p6818/prototype/module
PROTO_BASE   := drivers/platform/nexell/s5p6818/prototype/base
KBUILD_CFLAGS += -I$(PROTO_MODULE) -I$(PROTO_BASE)

ifeq ($(CONFIG_S5P6818_PROTO_RELEASE),y)
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_RELEASE
else
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_DEBUG
endif

include  $(srctree)/drivers/platform/nexell/s5p6818/prototype.mk



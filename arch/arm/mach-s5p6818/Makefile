#
# Copyright 2009 Nexell Co.
#	http://www.nexell.co.kr
#
# Makefile for the S5P6818

obj-y := cpu.o s5p6818.o lldebug.o

obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o

#===============================================================================
#	Build options for nexell soc
#===============================================================================
KBUILD_CFLAGS += -mfpu=vfp -mfloat-abi=softfp

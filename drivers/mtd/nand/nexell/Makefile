
obj-$(CONFIG_MTD_NAND_NXP)	+= nand.o

obj-$(CONFIG_MTD_NAND_ECC_HW)	+= nand_ecc.o

ifdef CONFIG_ARM64
obj-y += mtd.lib64.o
else
obj-y += mtd.lib.o
endif
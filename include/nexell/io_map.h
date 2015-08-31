#ifndef __IO_MAP_H__
#define __IO_MAP_H__

extern void __iomem *io_map_base;
#define	__PB_IO_MAP_REGS_PHYS	(0xC0000000)
#define	__PB_IO_MAP_REGS_SIZE	(SZ_1M * 3)

static inline void __iomem * __io_address(long addr)
{
	if (!io_map_base ||
		__PB_IO_MAP_REGS_PHYS > addr ||
		addr > __PB_IO_MAP_REGS_PHYS + __PB_IO_MAP_REGS_SIZE) {
		pr_err("%s invalild addr 0x%lx (0x%x ~ 0x%x)\n",
			__func__, addr, __PB_IO_MAP_REGS_PHYS,
			(__PB_IO_MAP_REGS_PHYS + __PB_IO_MAP_REGS_SIZE));
		return NULL;
	}
	return (void __iomem *)(addr - __PB_IO_MAP_REGS_PHYS + io_map_base);
}

#define IO_ADDRESS(n)	((void __iomem * )__io_address(n))

#endif

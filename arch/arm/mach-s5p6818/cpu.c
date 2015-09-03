/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/version.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/of_platform.h>
#include <linux/clk-provider.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/pgtable.h>
#include <asm/system_misc.h>

/* nexell soc headers */
#include <nexell/platform.h>
#include <mach/iomap.h>

#ifdef	PB_IO_MAP
#undef	PB_IO_MAP
#endif

#define PB_IO_MAP(_n_, _v_, _p_, _s_, _t_) { \
	.virtual	= _v_,					\
	.pfn 		= __phys_to_pfn(_p_),	\
	.length 	= _s_, 					\
	.type 		= _t_ 					\
	},

static struct map_desc cpu_iomap_desc[] = {
	#include <mach/s5p6818_iomap.h>
};

extern void nxp_cpu_init(void);
extern void nxp_cpu_init_time(void);
extern void nxp_cpu_init_irq(void);
extern struct smp_operations nxp_smp_ops;

/*
 * 	cpu initialize and io/memory map.
 * 	procedure: fixup -> map_io -> init_irq -> timer init -> init_machine
 */
static void __init cpu_map_io(void)
{
	int i;

	for (i = 0; i<ARRAY_SIZE(cpu_iomap_desc); i++) {
		printk(KERN_INFO "CPU : iomap[%2d]: p 0x%08x -> v 0x%08x len=0x%x\n", i,
			(u_int)(cpu_iomap_desc[i].pfn<<12),	(u_int)(cpu_iomap_desc[i].virtual),
			(u_int)(cpu_iomap_desc[i].length));
	}

	/* make iotable */
	iotable_init(cpu_iomap_desc, ARRAY_SIZE(cpu_iomap_desc));
}

static void __init cpu_init_early(void)
{
	nxp_cpu_init();
}

static void __init cpu_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

/*
 * Maintainer: Nexell Co., Ltd.
 */
static const char * const cpu_dt_compat[] = {
    "Nexell,s5p6818",
    NULL
};

DT_MACHINE_START(S5P6818, "s5p6818")
	.atag_offset	= 0x00000100,
	.nr				= 6818,
	.smp			= smp_ops(nxp_smp_ops),
	.map_io			= cpu_map_io,
	.init_early		= cpu_init_early,
	.init_machine	= cpu_init_machine,
	.dt_compat  	= cpu_dt_compat,
MACHINE_END

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

#if defined (CONFIG_ARCH_S5P6818)
#include <nexell/s5p6818/s5p6818-head.h>
#include <nexell/s5p6818/s5p6818-base.h>
#include <nexell/s5p6818/s5p6818-irq.h>
#include <nexell/s5p6818/s5p6818-iodesc.h>
#include <nexell/s5p6818/s5p6818-pm.h>
#endif

#if defined (CONFIG_ARM)
#include <mach/iomap.h>
#include <mach/io.h>
#endif
#if defined (CONFIG_ARM64)
#include <nexell/io_map.h>
#endif
#include <nexell/arch_type.h>

#ifndef __ASSEMBLY__

#endif	/* __ASSEMBLY__ */

/*
 * low level debug message
 */
void  lldebugout(const char *fmt, ...);

#if defined(CONFIG_PM_DBGOUT)
extern bool console_suspend_enabled;
#define	pm_dbgout(msg...)	do { \
			if (!console_suspend_enabled) printk(msg); else lldebugout(msg);	\
		} while(0)
#else
#define	pm_dbgout(msg...)	do {} while (0)
#endif

#ifdef CONFIG_EARLY_PRINTK
extern asmlinkage void early_printk(const char *fmt, ...);
#endif

/* system stop */
#ifndef halt
#define	halt()	{ 											\
		printk(KERN_ERR "%s(%d) : %s system halt ...\n", 	\
		__FILE__, __LINE__, __FUNCTION__); 					\
		do {} while(1);										\
	}
#endif
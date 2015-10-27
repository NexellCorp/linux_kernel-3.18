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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

/*
#define	pr_debug	printk
*/

#ifdef CONFIG_ARM64
#undef IO_ADDRESS
static void __iomem* io_reset_addr = NULL;
static inline void __iomem* IO_ADDRESS(unsigned long base)
{
	if (NULL == io_reset_addr)
		io_reset_addr = ioremap(base, PAGE_SIZE);
	return io_reset_addr;
}
#endif

static DEFINE_SPINLOCK(lock);
#if 1
#define	DEALY(s)	udelay(s)
#else
#define	DEALY(s)	{ volatile int loop = 10000 * 10;	while (loop-- > 0) { }	}
#endif

void nxp_soc_peri_reset_enter(int id)
{
	unsigned long base = NX_RSTCON_GetPhysicalAddress();
	void *addr = (void*)IO_ADDRESS(base);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%d [0x%p:0x%lx]\n", __func__, id, addr, base);
	RET_ASSERT(RESET_ID_VIP0 >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return;
	}

	spin_lock_irqsave(&lock, flags);

	NX_RSTCON_SetBaseAddress(addr);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_ASSERT);

	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL_GPL(nxp_soc_peri_reset_enter);

void nxp_soc_peri_reset_exit(int id)
{
	unsigned long base = NX_RSTCON_GetPhysicalAddress();
	void *addr = (void*)IO_ADDRESS(base);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%d [0x%p:0x%lx]\n", __func__, id, addr, base);
	RET_ASSERT(RESET_ID_VIP0 >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return;
	}

	spin_lock_irqsave(&lock, flags);

	NX_RSTCON_SetBaseAddress(addr);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_NEGATE);

	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL_GPL(nxp_soc_peri_reset_exit);

void nxp_soc_peri_reset_set(int id)
{
	unsigned long base = NX_RSTCON_GetPhysicalAddress();
	void *addr = (void*)IO_ADDRESS(base);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%d [0x%p:0x%lx]\n", __func__, id, addr, base);
	RET_ASSERT(RESET_ID_VIP0 >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return;
	}

	spin_lock_irqsave(&lock, flags);

	NX_RSTCON_SetBaseAddress(addr);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_ASSERT);
	DEALY(10);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_NEGATE);

	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL_GPL(nxp_soc_peri_reset_set);

int nxp_soc_peri_reset_status(int id)
{
	unsigned long base = NX_RSTCON_GetPhysicalAddress();
	void *addr = (void*)IO_ADDRESS(base);
	unsigned long flags;
	U32 RSTIndex = id;
	int power = 0;

	pr_debug("%s: id=%d [0x%p:0x%lx]\n", __func__, id, addr, base);
	RET_ASSERT_VAL(RESET_ID_VIP0 >= RSTIndex, -EINVAL);

	spin_lock_irqsave(&lock, flags);

	NX_RSTCON_SetBaseAddress(addr);
	power = NX_RSTCON_GetRST(RSTIndex) ? 1 : 0;

	spin_unlock_irqrestore(&lock, flags);

	return power;
}
EXPORT_SYMBOL_GPL(nxp_soc_peri_reset_status);


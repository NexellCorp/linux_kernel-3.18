/*
 * Spin Table SMP initialisation
 *
 * Copyright (C) 2013 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/smp_plat.h>

#include <nexell/platform.h>

extern void secondary_start_head(void);

static inline int secondary_boot_signature(int cpu)
{
	__raw_writel(virt_to_phys(secondary_start_head), __io_address(SCR_ARM_SECOND_BOOT));
	__raw_writel(cpu, __io_address(SCR_SMP_WAKE_CPU_ID));
	return 0;
}

/*
 * Write pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_SPINLOCK(boot_lock);

void  __init smp_ops_init_cpus(void)
{
	unsigned int i, ncores = NR_CPUS;

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

void  __init smp_ops_prepare_cpus(unsigned int max_cpus)
{
	__raw_writel(virt_to_phys(secondary_start_head), __io_address(SCR_ARM_SECOND_BOOT));
	__raw_writel((-1UL), __io_address(SCR_SIGNAGURE_RESET));
}

void __cpuinit smp_ops_secondary_init(unsigned int cpu)
{
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit smp_ops_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	secondary_boot_signature(cpu);

	/*
	 * This is really belt and braces; we hold unintended secondary
	 * CPUs in the holding pen until we're ready for them.  However,
	 * since we haven't sent them a soft interrupt, they shouldn't
	 * be there.
	 */
	write_pen_release(cpu_logical_map(cpu));

	/*
	 * Send the secondary CPU a soft interrupt, thereby causing
	 * the boot monitor to read the system wide flags register,
	 * and branch to the address found there.
	 */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static inline void cpu_enter_lowpower(void)
{
	unsigned int lv, mv;
	flush_cache_all();

	asm volatile(
	"	mcr	p15, 0, %1, c7, c5, 0\n"
	"	mcr	p15, 0, %1, c7, c10, 4\n"
	/*
	 * Turn off coherency(dcache off)
	 */
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	bic	%0, %0, %2\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	  : "=&r" (lv)
	  : "r" (0), "Ir" (CR_C)
	  : "cc");

	asm volatile(
	/*
	 * Turn off smp
	 */
	"	mrrc	p15, 0, %0, %1, c15\n"
	"	bic	%0, %0, #0x40\n"
	"	mcrr	p15, 0, %0, %1, c15\n"
	  : "=&r" (lv), "=&r" (mv)
	  : "r" (0)
	  : "cc");
}

static inline void cpu_leave_lowpower(void)
{
	unsigned int v;

	asm volatile(
	"	mrc	p15, 0, %0, c1, c0, 0\n"
	"	orr	%0, %0, %1\n"
	"	mcr	p15, 0, %0, c1, c0, 0\n"
	"	mrc	p15, 0, %0, c1, c0, 1\n"
	"	orr	%0, %0, #0x20\n"
	"	mcr	p15, 0, %0, c1, c0, 1\n"
	  : "=&r" (v)
	  : "Ir" (CR_C)
	  : "cc");
}

static inline void platform_do_lowpower(unsigned int cpu, int *spurious)
{
	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	for (;;) {
		/*
		 * here's the WFI
		 */
		__asm__ __volatile(
		"    wfi\n"
		  :
		  :
		  : "memory", "cc");

		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}

		/*
		 * Getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * Just note it happening - when we're woken, we can report
		 * its occurrence.
		 */
		(*spurious)++;
	}
}

int platform_cpu_kill(unsigned int cpu)
{
	return 1;
}

#if 0
extern bool pm_suspend_enter;
extern void (*core_do_suspend)(ulong, ulong);
#endif

static inline void platform_cpu_lowpower(int cpu)
{
#if 0
	void (*power_down)(ulong, ulong) = (void*)(core_do_suspend + 0x220);
	int spurious = 0;

	/*
	 * enter WFI when poweroff or restart
	 */

	if (false == pm_suspend_enter) {
		cpu_enter_lowpower();
		platform_do_lowpower(cpu, &spurious);
		halt();
	}

	/*
	 * enter SRAM text when suspend
	 */
	if (NULL == core_do_suspend)
		lldebugout("SMP: Fail, cpu.%d ioremap for suspend callee\n", cpu);

	dmb();
	power_down(__io_address(PHYS_BASE_ALIVE), __io_address(PHYS_BASE_CLK_DREX));
	nop(); nop(); nop();
	dmb();
#endif
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
static void smp_ops_cpu_die(unsigned int cpu)
{
	int spurious = 0;

#if !defined (CONFIG_S5P6818_PM_IDLE)
	platform_cpu_lowpower(cpu);
#endif

	/*
	 * we're ready for shutdown now, so do it
	 */
	cpu_enter_lowpower();
	platform_do_lowpower(cpu, &spurious);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
	cpu_leave_lowpower();

	/* wakeup form idle */
	write_pen_release(-1);
}

static int smp_ops_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
#endif

struct smp_operations nxp_smp_ops __initdata = {
	.smp_init_cpus	= smp_ops_init_cpus,
	.smp_prepare_cpus	= smp_ops_prepare_cpus,
	.smp_secondary_init	= smp_ops_secondary_init,
	.smp_boot_secondary	= smp_ops_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable	= smp_ops_cpu_disable,
	.cpu_die	= smp_ops_cpu_die,
#endif
};

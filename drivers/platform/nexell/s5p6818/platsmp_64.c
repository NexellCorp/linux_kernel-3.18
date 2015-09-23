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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/io.h>
#include <asm/suspend.h>

#include <nexell/platform.h>

/*
#define	pr_debug	printk
*/

#ifdef CONFIG_HOTPLUG_CPU
extern void secondary_start_head(void);
volatile unsigned long secondary_pen_release = INVALID_HWID;

/*
 * Write secondary_pen_release in a way that is guaranteed to be
 * visible to all observers, irrespective of whether they're taking part
 * in coherency or not.  This is necessary for the hotplug code to work
 * reliably.
 */
static void write_pen_release(u64 val)
{
	void *start = (void *)&secondary_pen_release;
	unsigned long size = sizeof(secondary_pen_release);

	secondary_pen_release = val;
	__flush_dcache_area(start, size);
}

static void inline smp_soc_cpu_signature(unsigned int cpu)
{
	__raw_writel(__pa(secondary_start_head), __io_address(SCR_ARM_SECOND_BOOT));
	__raw_writel(cpu, __io_address(SCR_SMP_WAKE_CPU_ID));
}

static int smp_soc_cpu_init(struct device_node *dn, unsigned int cpu)
{
	return 0;
}

static int smp_soc_cpu_prepare(unsigned int cpu)
{
	return 0;
}

static int smp_soc_cpu_boot(unsigned int cpu)
{
    unsigned long timeout;

	pr_debug("[%s cpu.%d]\n", __func__, cpu);
	smp_soc_cpu_signature(cpu);

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
    arch_send_call_function_ipi_mask(cpumask_of(cpu));

    timeout = jiffies + (1 * HZ);
    while (time_before(jiffies, timeout)) {
        smp_rmb();
        if (secondary_pen_release == INVALID_HWID)
            break;

        udelay(10);
    }

    /*
     * now the secondary core is starting up let it run its
     * calibrations, then wait for it to finish
     */

    return secondary_pen_release != INVALID_HWID ? -ENOSYS : 0;
}

static void smp_soc_cpu_postboot(void)
{
	pr_debug("[%s cpu.%d]\n", __func__, raw_smp_processor_id());
	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(INVALID_HWID);
}

static int smp_soc_cpu_disable(unsigned int cpu)
{
	pr_debug("[%s cpu.%d]\n", __func__, cpu);
	return 0;
}

static void smp_soc_cpu_die(unsigned int cpu)
{
	pr_debug("[%s cpu.%d]\n", __func__, cpu);

	/*
	 * request suspend to 2ndbootloader in EL3
	 * do core cache flush before enter suspend
	 */
	flush_cache_all();
	__asm__("smc 12345");
}
#endif

#ifdef CONFIG_ARM64_CPU_SUSPEND
extern int cpu_suspend_machine(unsigned long arg);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
static int cpu_suspend_finisher(unsigned long index)
{
	cpu_suspend_machine(index);

	/*
	 * request suspend to 2ndbootloader in EL3
	 * do core cache flush before enter suspend
	 */
	flush_cache_all();
	__asm__("smc 12345");

	return 0;
}
#endif
static int smp_soc_cpu_suspend(unsigned long index)
{
	pr_debug("[%s index 0x%lx ..]\n", __func__, index);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
	__cpu_suspend(index, cpu_suspend_finisher);
#else
	cpu_suspend_machine(index);

	/*
	 * request suspend to 2ndbootloader in EL3
	 * do core cache flush before enter suspend
	 */
	flush_cache_all();
	__asm__("smc 12345");
#endif
	return 0;
}
#endif

const struct cpu_operations cpu_nxp_ops = {
	.name		= "nxp-smp",
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_init	= smp_soc_cpu_init,
	.cpu_prepare	= smp_soc_cpu_prepare,
	.cpu_boot	= smp_soc_cpu_boot,
	.cpu_postboot	= smp_soc_cpu_postboot,
	.cpu_disable	= smp_soc_cpu_disable,
	.cpu_die	= smp_soc_cpu_die,
#endif
#ifdef CONFIG_ARM64_CPU_SUSPEND
	.cpu_suspend	= smp_soc_cpu_suspend,
#endif
};

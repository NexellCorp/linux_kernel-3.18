#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <asm/io.h>

#include <nexell/platform.h>

/*
#define	pr_debug	printk
*/
void __iomem *io_map_base = NULL;
extern void (*arm_pm_restart)(char str, const char *cmd);

static void s5p6818_cpu_poweroff(void)
{
	int n, cpu = raw_smp_processor_id();

	for_each_present_cpu(n) {
		if (n == cpu)
			continue;
		NX_CLKPWR_SetCPUPowerOff(n);
	}
	printk("cpu.%d shutdown ...\n", cpu);

	NX_ALIVE_SetWriteEnable(CTRUE);			/* close alive gate */
	NX_ALIVE_SetVDDPWRON(CFALSE, CFALSE);	/* Core power down */
	NX_ALIVE_SetWriteEnable(CFALSE);		/* close alive gate */
	NX_CLKPWR_SetCPUPowerOff(cpu);
	halt();
}

static void s5p6818_cpu_reset(char str, const char *cmd)
{
	printk("System reset: %s ...\n", cmd);
	mdelay(10);

	__raw_writel(0xFFFFFFFF, __io_address(SCR_RESET_SIG_RESET));

	if (cmd && !strcmp(cmd, "recovery"))
		__raw_writel(RECOVERY_SIGNATURE, __io_address(SCR_RESET_SIG_SET));

	if (cmd && !strcmp(cmd, "usbboot"))
		__raw_writel(USBBOOT_SIGNATURE, __io_address(SCR_RESET_SIG_SET));

	__raw_readl (__io_address(SCR_RESET_SIG_READ));

	printk("recovery signature [0x%x:0x%x] \n",
		SCR_RESET_SIG_READ, __raw_readl(__io_address(SCR_RESET_SIG_READ)));

	NX_ALIVE_SetWriteEnable(CFALSE);	/* close alive gate */
	NX_CLKPWR_SetSoftwareResetEnable(CTRUE);
	NX_CLKPWR_DoSoftwareReset();
}

static void s5p6818_cpu_iomap(void)
{
	int i = 0;

	NX_ALIVE_SetBaseAddress((void*)__io_address(NX_ALIVE_GetPhysicalAddress()));
	NX_TIEOFF_SetBaseAddress((void*)__io_address(NX_TIEOFF_GetPhysicalAddress()));
	NX_CLKPWR_SetBaseAddress((void*)__io_address(NX_CLKPWR_GetPhysicalAddress()));
	NX_ECID_SetBaseAddress((void*)__io_address(NX_ECID_GetPhysicalAddress()));
	NX_MCUS_SetBaseAddress((void*)__io_address(NX_MCUS_GetPhysicalAddress()));
	NX_RTC_SetBaseAddress((void*)__io_address(NX_RTC_GetPhysicalAddress()));

	for (i = 0; NX_GPIO_GetNumberOfModule() > i; i++) {
		NX_GPIO_SetBaseAddress(i, (void*)IO_ADDRESS(NX_GPIO_GetPhysicalAddress(i)));
		NX_GPIO_OpenModule(i);
	}

	/*
	 * NOTE> ALIVE Power Gate must enable for RTC register access.
	 * 		 must be clear wfi jump address
	 */
	NX_ALIVE_SetWriteEnable(CTRUE);

	/* clear cpu id register for second cores */
	__raw_writel(0xFFFFFFFF, __io_address(SCR_ARM_SECOND_BOOT));
	__raw_writel(0xFFFFFFFF, __io_address(SCR_SMP_WAKE_CPU_ID));
}

static int __init cpu_early_initcall_setup_64(void)
{
	int ret = 0;

	io_map_base = ioremap(__PB_IO_MAP_REGS_PHYS, __PB_IO_MAP_REGS_SIZE);
	if (NULL == io_map_base) {
		pr_err("fail io map ...\n");
		ret = -ENOMEM;
	}

	s5p6818_cpu_iomap();

	printk("IO MAP: p 0x%x : v 0x%p (0x%x)\n",
		__PB_IO_MAP_REGS_PHYS, io_map_base, __PB_IO_MAP_REGS_SIZE);

	pm_power_off = s5p6818_cpu_poweroff;
	arm_pm_restart = s5p6818_cpu_reset;

	return ret;
}
early_initcall(cpu_early_initcall_setup_64);

/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/setup.h>
#include <asm/io.h>

#include <nexell/platform.h>

static void cpu_setup_base(void)
{
	int i = 0;

	NX_RSTCON_Initialize();
	NX_RSTCON_SetBaseAddress((void*)IO_ADDRESS(NX_RSTCON_GetPhysicalAddress()));

	NX_TIEOFF_Initialize();
	NX_TIEOFF_SetBaseAddress((void*)IO_ADDRESS(NX_TIEOFF_GetPhysicalAddress()));

	NX_GPIO_Initialize();
	for (i = 0; NX_GPIO_GetNumberOfModule() > i; i++) {
		NX_GPIO_SetBaseAddress(i, (void*)IO_ADDRESS(NX_GPIO_GetPhysicalAddress(i)));
		NX_GPIO_OpenModule(i);
	}

	NX_ALIVE_Initialize();
	NX_ALIVE_SetBaseAddress((void*)IO_ADDRESS(NX_ALIVE_GetPhysicalAddress()));
	NX_ALIVE_OpenModule();

	NX_CLKPWR_Initialize();
	NX_CLKPWR_SetBaseAddress((void*)IO_ADDRESS(NX_CLKPWR_GetPhysicalAddress()));
	NX_CLKPWR_OpenModule();

	NX_ECID_Initialize();
	NX_ECID_SetBaseAddress((void*)IO_ADDRESS(NX_ECID_GetPhysicalAddress()));

	/* MCUS for Static Memory. */
	NX_MCUS_Initialize();
	NX_MCUS_SetBaseAddress((void*)IO_ADDRESS(NX_MCUS_GetPhysicalAddress()));
	NX_MCUS_OpenModule();

	/*
	 * NOTE> ALIVE Power Gate must enable for RTC register access.
	 * 		 must be clear wfi jump address
	 */
	NX_ALIVE_SetWriteEnable(CTRUE);
	__raw_writel(0xFFFFFFFF, (void*)__io_address(SCR_ARM_SECOND_BOOT));

	/* clear cpu id register for second cores */
	__raw_writel((-1UL), (void*)__io_address(SCR_SMP_WAKE_CPU_ID));
}

static void cpu_setup_bus(void)
{
}

void (*nxp_pm_poweroff)(void) = NULL;
void (*nxp_pm_reset)(char str, const char *cmd) = NULL;

static void cpu_shutdown(void)
{
	int n, cpu = raw_smp_processor_id();

	if (nxp_pm_poweroff)
		nxp_pm_poweroff();

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

static void cpu_reset(char str, const char *cmd)
{
	lldebugout("System reset: %s ...\n", cmd);
	mdelay(10);

	if (nxp_pm_reset)
		nxp_pm_reset(str, cmd);

	__raw_writel((-1UL), (void*)__io_address(SCR_RESET_SIG_RESET));

	if (cmd && !strcmp(cmd, "recovery")) {
		__raw_writel(RECOVERY_SIGNATURE, __io_address(SCR_RESET_SIG_SET));
		__raw_readl (__io_address(SCR_RESET_SIG_READ));	/* verify */
		printk("recovery sign [0x%x:0x%x] \n",
			SCR_RESET_SIG_READ, readl(__io_address(SCR_RESET_SIG_READ)));
	}

	NX_ALIVE_SetWriteEnable(CFALSE);	/* close alive gate */
	NX_CLKPWR_SetSoftwareResetEnable(CTRUE);
	NX_CLKPWR_DoSoftwareReset();
}

extern void (*arm_pm_restart)(char str, const char *cmd);

void nxp_cpu_init(void)
{
	cpu_setup_base();
	cpu_setup_bus();

	/* redefine poweroff */
	pm_power_off = cpu_shutdown;
	arm_pm_restart = cpu_reset;
}


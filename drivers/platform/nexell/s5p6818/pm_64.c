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
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/cpu_pm.h>
#include <asm/suspend.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/sections.h> 	/*_stext, _end*/

#include <nexell/platform.h>
#include <nexell/pm.h>

#if 0
#define	pr_debug		printk
#define	ll_debug		early_printk
#else
#define	ll_debug(m...)
#endif

//#define PM_IO_FORCE_INIT

/*-----------------------------------------------------------------------------*/
#define SRAM_MEM_BASE		0xFFFF0000
#define SRAM_MEM_SIZE		0x10000		/* 64 Kbyte */
#define GPIO_GROUP_NUM		5			/* A,B,C,D,E */
#define WAKE_ALIVE_NR		6			/* A,B,C,D,E */
#define WAKE_REG_CLEAR		(0xffffffff)

#define	PM_SAVE_ADDR		__pa(_stext)
#define	PM_SAVE_VIRT		_stext
#define	PM_SAVE_SIZE		SUSPEND_SAVE_SIZE

struct pm_reg_gpio {
	u32 data;			/* 0x00 */
	u32 output;			/* 0x04 */
	u32 iofn[2];		/* 0x20, 0x24 */
	u32 mode[3];		/* 0x08, 0x0C, 0x28 */
	u32 mask;			/* 0x10, 0x3C */
	u32 val[10];  		/* 0x40 ~ 0x64 */
};

struct pm_reg_alive {
	u32  detmod[6];
	u32  detenb;
	u32  irqenb;
	u32  outenb;
	u32  outval;
	u32  pullen;
};

struct pm_soc_data {
	unsigned long  sram_data[SRAM_MEM_SIZE/sizeof(unsigned long)];
	unsigned long  sram_base;
	unsigned long *sram_map;
	unsigned long  sram_len;
	u32 wake_event;
	u32 wake_rtc;
	u32 wake_input[WAKE_ALIVE_NR][2];
	/* registers */
	u32 gpio_fn[GPIO_GROUP_NUM][2];
	struct pm_reg_gpio  gpio[GPIO_GROUP_NUM];
	struct pm_reg_alive alive;
};

static struct pm_soc_data *pm_data = NULL;
static struct pm_suspend_ops *pm_ops = NULL;
void (*pm_suspend_signatrue)(int) = NULL;

static const char * str_wake_event[] = {
	[0] = "VDDPWRTOGGLE",
	[1] = "RTC",
	[2] = "ALIVE 0",
	[3] = "ALIVE 1",
	[4] = "ALIVE 2",
	[5] = "ALIVE 3",
	[6] = "ALIVE 4",
	[7] = "ALIVE 5",
	[8] = "ALIVE 6",
	[9] = "ALIVE 7",
};
#define	WAKE_EVENT_NUM	ARRAY_SIZE(str_wake_event)

#define	RTC_REG_INTENB	(0x010)
#define	RTC_REG_INTPND	(0x014)

static void dump_wake_event(void)
{
	struct pm_soc_data *pm = pm_data;
	int i = 0;

	for (i = 0; WAKE_EVENT_NUM > i; i++) {
		if (pm->wake_event & 1<<i)
			early_printk("Resume from [%s]\n", str_wake_event[i]);
	}
}

int pm_wake_is_power_key(int pin)
{
	struct pm_soc_data *pm = pm_data;

	if ((1<<pin) & pm->wake_event)
		return 1;

	return 0;
}

static void suspend_core_dt(struct pm_soc_data *pm)
{
	struct device_node *np;
	const __be32 *list;
	int i = 0;

	np = of_find_node_by_name(NULL, "pm_alive_config");
	if (np) {
		const char *name[] = {
				"pm,alive_0", "pm,alive_1",
				"pm,alive_2", "pm,alive_3",
				"pm,alive_4", "pm,alive_5"
				};

		for (i = 0; WAKE_ALIVE_NR > i; i++) {
			list = of_get_property(np, name[i], NULL);
			pm->wake_input[i][0] = be32_to_cpu(*list++);	/* ture/false */
			pm->wake_input[i][1] = be32_to_cpu(*list++);	/* type */
			printk("PM: ALIVE %d %s type %d (%s)\n", i,
				pm->wake_input[i][0]?"on ":"off", pm->wake_input[i][1], name[i]);
		}
	}

	np = of_find_node_by_name(NULL, "pm_rtc_config");
	if (np) {
		const char *name = "pm,rtc";
		list = of_get_property(np, name, NULL);
		pm->wake_rtc = be32_to_cpu(*list++);	/* ture/false */
		printk("PM: RTC %s (%s)\n", pm->wake_rtc?"on ":"off", name);
	}
}

static int suspend_core_init(void)
{
	struct pm_soc_data *pm;
	u32 (*iofn)[2];
	int i = 0, j = 0;
	int ionum = GPIO_GROUP_NUM;

	pm = kzalloc(sizeof(struct pm_soc_data), GFP_KERNEL);
	if (NULL == pm) {
		printk(KERN_ERR "Out of memory for PM (%ld) !!!\n",
			sizeof(struct pm_soc_data));
		return -ENOMEM;
	}
	pm_data = pm;

	pm->sram_base = SRAM_MEM_BASE;
	pm->sram_len = SRAM_MEM_SIZE;
	pm->sram_map = ioremap(pm->sram_base, pm->sram_len);

	if (NULL == pm->sram_map) {
		printk(KERN_ERR "Failed to ioremap PM (0x%lx) !!!\n", pm->sram_base);
		kfree(pm);
		return -EINVAL;
	}

	/* backup sram */
	memcpy(pm->sram_data, pm->sram_map, pm->sram_len);

	/* backup gpio alt */
	iofn = (u32(*)[2])pm->gpio_fn;

	for (i = 0; ionum > i; i++) {
		for (j = 0; j < GPIO_NUM_PER_BANK/2; j++)
			iofn[i][0] |= (GET_GPIO_ALTFUNC(i, j) << (j<<1));
		for (j = 0; j < GPIO_NUM_PER_BANK/2; j++)
			iofn[i][1] |= (GET_GPIO_ALTFUNC(i, j+16) << (j<<1));
		pr_debug("io.%d: 0x%08x, 0x%08x\n", i, iofn[i][0], iofn[i][1]);
	}

	suspend_core_dt(pm);

	return 0;
}

static void suspend_gpio(suspend_state_t stat)
{
	struct pm_soc_data *pm = pm_data;
	struct pm_reg_gpio *gpio = pm->gpio;
	void *iomap = __io_address(PHYS_BASE_GPIOA);
	int i = 0, j = 0;


	if (SUSPEND_SUSPEND == stat) {
		for (i = 0; GPIO_GROUP_NUM > i; i++, iomap += 0x1000) {
			gpio[i].data    = __raw_readl(iomap+0x00);
			gpio[i].output  = __raw_readl(iomap+0x04);
			gpio[i].iofn[0] = __raw_readl(iomap+0x20);
			gpio[i].iofn[1] = __raw_readl(iomap+0x24);
			gpio[i].mode[0] = __raw_readl(iomap+0x08);
			gpio[i].mode[1] = __raw_readl(iomap+0x0C);
			gpio[i].mode[2] = __raw_readl(iomap+0x28);
			gpio[i].mask    = __raw_readl(iomap+0x10);

			for (j = 0; j < 10; j++)
				gpio[i].val[j] = __raw_readl(iomap+0x40+(j<<2));

			__raw_writel(WAKE_REG_CLEAR, (iomap+0x14));	/* clear pend */
		}
	} else {
		for (i = 0; GPIO_GROUP_NUM > i; i++, iomap += 0x1000) {
	#ifndef CONFIG_S5P6818_PM_IDLE
			for (j = 0; j < 10; j++)
				__raw_writel(gpio[i].val[j], (iomap+0x40+(j<<2)));
	#endif
			__raw_writel(gpio[i].output, (iomap+0x04));
			__raw_writel(gpio[i].data,   (iomap+0x00));
			__raw_writel(gpio[i].iofn[0],(iomap+0x20));
			__raw_writel(gpio[i].iofn[1],(iomap+0x24));
			__raw_writel(gpio[i].mode[0],(iomap+0x08));
			__raw_writel(gpio[i].mode[1],(iomap+0x0C)),
			__raw_writel(gpio[i].mode[2],(iomap+0x28));
			__raw_writel(gpio[i].mask, (iomap+0x10));
			__raw_writel(gpio[i].mask, (iomap+0x3C));
			__raw_writel(WAKE_REG_CLEAR, (iomap+0x14));	/* clear pend */
		}
	}
}

static void suspend_alive(suspend_state_t stat)
{
	struct pm_soc_data *pm = pm_data;
	struct pm_reg_alive *alive = &pm->alive;
	void *iomap = __io_address(PHYS_BASE_ALIVE);
	int i = 0;

	NX_ALIVE_SetWriteEnable(CTRUE);
	if (SUSPEND_SUSPEND == stat) {
		for (i = 0; 6 > i; i++)
			alive->detmod[i] = __raw_readl(iomap + (i*0x0C) + 0x0C);

		alive->detenb = __raw_readl(iomap + 0x54);
		alive->irqenb = __raw_readl(iomap + 0x60);
		alive->outenb = __raw_readl(iomap + 0x7C);
		alive->outval = __raw_readl(iomap + 0x94);
		alive->pullen = __raw_readl(iomap + 0x88);
	} else {
		if (alive->outenb != __raw_readl(iomap + 0x7C)) {
			__raw_writel(WAKE_REG_CLEAR, iomap + 0x74);		/* reset */
			__raw_writel(alive->outenb, iomap + 0x78);		/* set */
		}
		if (alive->outval != __raw_readl(iomap + 0x94)) {
			__raw_writel(WAKE_REG_CLEAR, iomap + 0x8C);		/* reset */
			__raw_writel(alive->outval, iomap + 0x90);		/* set */
		}
		if (alive->pullen != __raw_readl(iomap + 0x88)) {
			__raw_writel(WAKE_REG_CLEAR, iomap + 0x80);		/* reset */
			__raw_writel(alive->pullen, iomap + 0x84);		/* set */
		}

		__raw_writel(WAKE_REG_CLEAR, iomap + 0x4C);			/* reset */
		__raw_writel(WAKE_REG_CLEAR, iomap + 0x58);			/* reset */

		for (i = 0; 6 > i; i++) {
			__raw_writel(WAKE_REG_CLEAR, iomap + (i*0x0C) + 0x04);		/* reset */
			__raw_writel(alive->detmod[i], iomap + (i*0x0C) + 0x08);	/* set */
		}

		__raw_writel(alive->detenb, iomap + 0x50);	/* set */
		__raw_writel(alive->irqenb, iomap + 0x5C);	/* set */
	}
}

static void suspend_sign(suspend_state_t stat, unsigned long addr, long size)
{
#ifndef CONFIG_S5P6818_PM_IDLE

	if (pm_suspend_signatrue) {
		pm_suspend_signatrue((SUSPEND_SUSPEND == stat ? 1: 0));
		return;
	}

	__raw_writel(WAKE_REG_CLEAR, __io_address(SCR_WAKE_FN_RESET));
	__raw_writel(WAKE_REG_CLEAR, __io_address(SCR_CRC_PHY_RESET));
	__raw_writel(WAKE_REG_CLEAR, __io_address(SCR_CRC_RET_RESET));
	__raw_writel(WAKE_REG_CLEAR, __io_address(SCR_CRC_LEN_RESET));
	__raw_writel(WAKE_REG_CLEAR, __io_address(SCR_SIGNAGURE_RESET));

	if (SUSPEND_SUSPEND == stat) {
		struct pm_suspend_sign sign = {
			.resume = (u32)virt_to_phys(cpu_resume),
			.signature = SUSPEND_SIGNATURE,
			.crc_addr = addr ? virt_to_phys((void*)addr) : (unsigned long)PM_SAVE_ADDR,
			.crc_size = size ? size : PM_SAVE_SIZE,
		};

		sign.crc_ret = pm_crc_calc((void*)phys_to_virt(sign.crc_addr), sign.crc_size);

		ll_debug("[PM crc: 0x%08x, fn:0x%08x phy:0x%08x, len:0x%x]\n",
				sign.crc_ret, sign.resume, sign.crc_addr, sign.crc_size);

		__raw_writel(sign.signature, __io_address(SCR_SIGNAGURE_SET));
		__raw_writel(sign.resume, __io_address(SCR_WAKE_FN_SET));
		__raw_writel(sign.crc_addr, __io_address(SCR_CRC_PHY_SET));
		__raw_writel(sign.crc_ret, __io_address(SCR_CRC_RET_SET));
		__raw_writel(sign.crc_size, __io_address(SCR_CRC_LEN_SET));
	}
#endif
}

static void suspend_cores(suspend_state_t stat)
{
	struct pm_soc_data *pm = pm_data;

	if (SUSPEND_SUSPEND == stat) {
		/* restore sram */
		memcpy(pm->sram_map, pm->sram_data, pm->sram_len);

	#ifndef CONFIG_S5P6818_PM_IDLE
		NX_CLKPWR_SetBaseAddress((void*)IO_ADDRESS(NX_CLKPWR_GetPhysicalAddress()));
		NX_CLKPWR_SetCPUResetMode(NX_CLKPWR_CPU_RESETMODE_SAFE);
		NX_CLKPWR_SetCPUPowerOn32(0x00);
	#endif
	}
}

static int resume_machine(void)
{
	struct pm_soc_data *pm = pm_data;
	void *rtcmap = __io_address(PHYS_BASE_RTC);
	u32 status, alive = 0, alarm = 0;

	ll_debug("%s\n", __func__);

	NX_ALIVE_SetWriteEnable(CTRUE);

	alive = NX_ALIVE_GetInterruptPending32();
	alarm = __raw_readl(rtcmap + RTC_REG_INTPND) &
			__raw_readl(rtcmap + RTC_REG_INTENB) & (1<<1);
	status = (alive << 2) | (alarm ? (1<<1) : 0);

	/* set wake event */
	pm->wake_event = status & ((1<<WAKE_EVENT_NUM) - 1);

	if (pm_ops && pm_ops->poweron)
		pm_ops->poweron();

	return 0;
}

/* return : 0 = wake up, 1 = goto suspend */
static int suspend_valid(suspend_state_t state)
{
	struct pm_soc_data *pm = pm_data;
	int ret = 1;

	/* clear events */
	pm->wake_event = 0;

#ifdef CONFIG_SUSPEND
	if (!suspend_valid_only_mem(state)) 
		return 0;
#endif
	if (pm_ops && pm_ops->valid)
		ret = pm_ops->valid(state);

	return ret;
}

/* return : 0 = goto suspend, 1 = wake up */
static int suspend_begin(suspend_state_t state)
{
	int ret = 0;

	if (pm_ops && pm_ops->begin)
		ret = pm_ops->begin(state);

	return ret;
}

/* return : 0 = goto suspend, 1 = wake up */
static int suspend_prepare(void)
{
	int ret = 0;

	if (pm_ops && pm_ops->prepare)
		ret = pm_ops->prepare();

	return ret;
}

static int suspend_enter(suspend_state_t state)
{
	int ret = 0;

	ll_debug("supend entered...\n");

	if (pm_ops && pm_ops->enter) {
		ret = pm_ops->enter(state);
		if (ret)
			return ret;
	}

	suspend_gpio(SUSPEND_SUSPEND);
	suspend_alive(SUSPEND_SUSPEND);
	suspend_sign(SUSPEND_SUSPEND, 0, 0);

	/* SMP power down */
	suspend_cores(SUSPEND_SUSPEND);
	ll_debug("enter suspend\n");

	/*
	 * goto sleep
	 */
	if (0 == ret)
		cpu_suspend(0);

	ll_debug("exit resume ...\n");

	suspend_sign(SUSPEND_RESUME, 0, 0);
	resume_machine();

	suspend_alive(SUSPEND_RESUME);
	suspend_gpio(SUSPEND_RESUME);
	suspend_cores(SUSPEND_RESUME);	/* last */

	dump_wake_event();

	return ret;
}

static void suspend_finish(void)
{
	ll_debug("%s:%d\n", __func__, __LINE__);
	if (pm_ops && pm_ops->finish)
		pm_ops->finish();
}

static void suspend_end(void)
{
	ll_debug("%s:%d\n", __func__, __LINE__);
	if (pm_ops && pm_ops->end)
		pm_ops->end();
}

static struct platform_suspend_ops suspend_ops = {
	.valid      = suspend_valid,     /* first suspend call */
	.begin      = suspend_begin,     /* before driver suspend */
	.prepare    = suspend_prepare,   /* after driver suspend */
	.enter      = suspend_enter,     /* goto suspend */
	.finish     = suspend_finish,    /* before driver resume */
	.end        = suspend_end,       /* after driver resume */
};

static int __init suspend_pm_init(void)
{
	int ret = suspend_core_init();
	if (0 > ret)
		return ret;

	suspend_set_ops(&suspend_ops);

	return 0;
}
arch_initcall(suspend_pm_init);

void pm_pre_ops_register(struct pm_suspend_ops *pm_ops)
{
	pm_ops = pm_ops;
}

int cpu_suspend_machine(unsigned long arg)
{
	struct pm_soc_data *pm = pm_data;
#ifdef PM_IO_FORCE_INIT
	void *iomap = __io_address(PHYS_BASE_GPIOA);
	u32 (*iofn)[2] = (u32(*)[2])pm->gpio_fn;
#endif
	void *rtcmap = __io_address(PHYS_BASE_RTC);
	u32 (*input)[2] = (u32(*)[2])pm->wake_input;
	int i = 0, n = 0, ret = 0;
	int mask = 0;

	NX_ALIVE_SetWriteEnable(CTRUE);
	NX_ALIVE_ClearWakeUpStatus();
	NX_ALIVE_ClearInterruptPendingAll();

	for (i = 0; WAKE_ALIVE_NR > i; i++) {
		if ((input[i][0] == CFALSE) || (input[i][1] > PWR_DECT_BOTHEDGE))
			continue;

		mask |= input[i][0] ? (1 << i) : 0;

		if (input[i][1] < PWR_DECT_BOTHEDGE) {
			for (n = 0; PWR_DECT_BOTHEDGE > n; n++) {
				CBOOL MODE = (n == input[i][1]) ? CTRUE : CFALSE;
				NX_ALIVE_SetDetectMode(n, i, MODE);
			}
		} else {  /* both edge */
			NX_ALIVE_SetDetectMode(PWR_DECT_FALLINGEDGE, i, CTRUE);
			NX_ALIVE_SetDetectMode(PWR_DECT_RISINGEDGE , i, CTRUE);
		}
	}

	NX_ALIVE_SetInputEnable32(mask);
	NX_ALIVE_SetDetectEnable32(mask);
	NX_ALIVE_SetInterruptEnable32(mask);

	/* disable alarm wakeup */
	if (!pm->wake_rtc) {
		__raw_writel(__raw_readl(rtcmap+RTC_REG_INTENB) & ~(3<<0),
			(rtcmap+RTC_REG_INTENB));
		__raw_writel(__raw_readl(rtcmap+RTC_REG_INTPND) & ~(3<<0),
			(rtcmap+RTC_REG_INTPND));
	}

	if (NX_ALIVE_GetInterruptPending32() & mask)
		return -EINVAL;

	if (__raw_readl(rtcmap + RTC_REG_INTPND) &
		__raw_readl(rtcmap + RTC_REG_INTENB) & (1<<1))
		return -EINVAL;

	if (pm_ops && pm_ops->poweroff)
		ret = pm_ops->poweroff();

	if (0 > ret)
		return ret;

#ifdef PM_IO_FORCE_INIT
	/*
	 * Set GPIO input mode, when suspending
	 * Note, debug uart is disabled
	 */
	for (i = 0; GPIO_GROUP_NUM > i; i++, iomap += 0x1000) {
		__raw_writel(iofn[i][0], (iomap+0x20));
		__raw_writel(iofn[i][1], (iomap+0x24));
		__raw_writel(0, (iomap+0x04));	/* Input */
		__raw_writel(0, (iomap+0x58));	/* GPIOx_PULLSEL - Down */
		__raw_writel(0, (iomap+0x60)); 	/* GPIOx_PULLENB - Disable */
	}
#endif

	return ret;
}


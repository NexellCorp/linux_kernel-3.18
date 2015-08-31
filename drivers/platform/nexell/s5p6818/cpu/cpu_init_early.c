#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <asm/io.h>

#include <nexell/platform.h>

/*
#define	pr_debug	printk
*/
#define	PIN_FN_SIZE		4
static void cpu_early_setup_gpio(void __iomem *base,
							int index, u32 *pins, int size)
{
	int pin = 0;

	NX_GPIO_SetBaseAddress(index, base);
	NX_GPIO_ClearInterruptPendingAll(index);

	while (size > pin) {

		int io = pins[pin], alt = PAD_GET_FUNC(io);
		int mod = PAD_GET_MODE(io), lv = PAD_GET_LEVEL(io);
		int str = PAD_GET_STRENGTH(io), plup = PAD_GET_PULLUP(io);

		switch (alt) {
		case PAD_GET_FUNC(PAD_FUNC_ALT0): alt = NX_GPIO_PADFUNC_0;	break;
		case PAD_GET_FUNC(PAD_FUNC_ALT1): alt = NX_GPIO_PADFUNC_1;	break;
		case PAD_GET_FUNC(PAD_FUNC_ALT2): alt = NX_GPIO_PADFUNC_2;	break;
		case PAD_GET_FUNC(PAD_FUNC_ALT3): alt = NX_GPIO_PADFUNC_3;	break;
		default: pr_err("Error, unknown alt (%d.%02d=%d) def 0x%08x\n",
				index, pin, alt, io);
			continue;
		}

		NX_GPIO_SetPadFunction(index, pin, alt);
		NX_GPIO_SetOutputEnable(index, pin,
				(mod == PAD_GET_MODE(PAD_MODE_OUT) ? CTRUE : CFALSE));
		NX_GPIO_SetOutputValue(index, pin,  (lv ? CTRUE : CFALSE));
		NX_GPIO_SetInterruptMode(index, pin, lv);
		NX_GPIO_SetPullEnable(index, pin, (NX_GPIO_PULL)plup);
		NX_GPIO_SetDriveStrength(index, pin, (NX_GPIO_DRVSTRENGTH)str); /* pad strength */

		pr_debug("[Gpio %d.%2d: DIR %s, ALT %d, Level %s, Pull %s, Str %d]\n",
			index, pin, mod == PAD_GET_MODE(PAD_MODE_OUT)?"OUT":" IN",
			alt, lv?"H":"L", plup==0?" DN":(plup==1?" UP":"OFF"), str);
		pin++;
	}
}

static void cpu_early_setup_alive(void __iomem *base,
							int index, u32 *pins, int size)
{
	int pin = 0;

	NX_ALIVE_SetBaseAddress(base);

	while (size > pin) {

		int io = pins[pin], mod = PAD_GET_MODE(io);
		int lv = PAD_GET_LEVEL(io), plup = PAD_GET_PULLUP(io);
		int det = 0;

		NX_ALIVE_ClearInterruptPending(pin);
		NX_ALIVE_SetOutputEnable(pin,
				(mod == PAD_GET_MODE(PAD_MODE_OUT) ? CTRUE : CFALSE));
		NX_ALIVE_SetOutputValue (pin, lv);
		NX_ALIVE_SetPullUpEnable(pin, (plup & 1 ? CTRUE : CFALSE));

		for (det = 0; 6 > det; det++) {
			mod == PAD_GET_MODE(PAD_MODE_INT) ?
			NX_ALIVE_SetDetectMode(det, pin, (lv == det ? CTRUE : CFALSE)) :
			NX_ALIVE_SetDetectMode(det, pin, CFALSE);
		}

		NX_ALIVE_SetDetectEnable(pin, (mod == PAD_MODE_INT ? CTRUE : CFALSE));
		pr_debug("[ALive%d: DIR %s, evel %s, Pull %s]\n",
			pin, mod == PAD_GET_MODE(PAD_MODE_OUT)?"OUT":" IN",
			lv?"H":"L", lv==0?" DN": (lv==1?" UP":"OFF"));
		pin++;
	}

	NX_ALIVE_SetWriteEnable(CTRUE);
}

static int __init cpu_early_initcall_setup(void)
{
	struct device_node *np = of_find_node_by_name(NULL, "pin_config");
	struct device_node *child;
	const __be32 *list;
	void __iomem *base;
	u32 pins[32];
	int index = 0, i = 0, size = 0;

	if (!np) {
		printk("*** WARNING: Not exist pin DTS for init gpio/alive. ***\n");
		return -EINVAL;
	}

	for_each_child_of_node(np, child) {
		list = of_get_property(child, "pin,functions", &size);
		size /= PIN_FN_SIZE;

		if (!list ||!size)
			return 0;

		base = of_iomap(child, 0);
		for (i = 0; size > i; i++)
			pins[i] = be32_to_cpu(*list++);

		if (!of_node_cmp(child->type, "gpio"))
			cpu_early_setup_gpio (base, index, pins, size);
		else if (!of_node_cmp(child->type, "alive"))
			cpu_early_setup_alive(base, index, pins, size);
		else
			continue;

		iounmap(base);
		index++;
	}
	return 0;
}
early_initcall(cpu_early_initcall_setup);

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
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/irq.h>

#include "irqchip.h"

#include <nexell/platform.h>

/*
#define pr_debug	printk
*/

struct cpu_irq_domain_data {
	struct list_head link;
	struct device_node *node;
	void __iomem *base;
	struct irq_chip *chip;
	void (*handler)(unsigned int, struct irq_desc *);
	struct irq_domain *domain;
	unsigned int irqno;		/* physical irq number */
	unsigned int hwirq;		/* domain hwirq */
	unsigned int first_irq;	/* domain first_irq */
	unsigned int size;
};

static int low_first_irq = S32_MAX;
static LIST_HEAD(irq_domain_link);

/*
 *  ALIVE irq chain handler
 *  start  -> request_irq -> alive irq_unmask
 *  do IRQ -> alive handler -> alive irq_mask -> alive irq_ack -> driver handler -> alive irq_unmask ->
 *  end    -> disable
 */
#define	ALIVE_MOD_REST		(0x04)	// detect mode reset
#define	ALIVE_MOD_SET		(0x08)	// detect mode
#define	ALIVE_MOD_READ		(0x0C)	// detect mode read
#define	ALIVE_DET_RESET		(0x4C)
#define	ALIVE_DET_SET		(0x50)
#define	ALIVE_DET_READ		(0x54)
#define	ALIVE_INT_RESET		(0x58)	// interrupt reset 	: disable
#define	ALIVE_INT_SET		(0x5C)	// interrupt set	: enable
#define	ALIVE_INT_SET_READ	(0x60)	// interrupt set read
#define	ALIVE_INT_STATUS	(0x64)	// interrupt detect pending and clear
#define	ALIVE_OUT_RESET		(0x74)
#define	ALIVE_OUT_SET		(0x78)
#define	ALIVE_OUT_READ		(0x7C)

#ifdef CONFIG_ARM64
#define ARM_DMB()		dmb(sy)
#else
#define ARM_DMB()		dmb()
#endif

#define	irqdata_to_bit(d, p)	(d->hwirq - p->hwirq)

static void irq_alive_ack(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	pr_debug("%s: alive irq=%d, io=%d\n", __func__, d->irq, bit);
	writel((1<<bit), base + ALIVE_INT_STATUS);	/* ack:irq pend clear */
	ARM_DMB();
}

static void irq_alive_mask(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	pr_debug("%s: alive irq=%d, io=%d\n", __func__, d->irq, bit);
	writel((1<<bit), base + ALIVE_INT_RESET);	/* mask:irq reset (disable) */
}

static void irq_alive_unmask(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	pr_debug("%s: alive irq=%d, io=%d\n", __func__, d->irq, bit);
	writel((1<<bit), base + ALIVE_INT_SET); ARM_DMB();
}

static int irq_alive_set_type(struct irq_data *d, unsigned int type)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	int offs = 0, i = 0;
	NX_ALIVE_DETECTMODE mode = 0;

	pr_debug("%s: alive irq=%d, io=%d, type=0x%x\n", __func__, d->irq, bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:	pr_warn("%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_FALLING:	mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE; break;
	case IRQ_TYPE_EDGE_RISING:	mode = NX_ALIVE_DETECTMODE_SYNC_RISINGEDGE;	break;
	case IRQ_TYPE_EDGE_BOTH:	mode = NX_ALIVE_DETECTMODE_SYNC_FALLINGEDGE; break;	/* and Rising Edge */
	case IRQ_TYPE_LEVEL_LOW:	mode = NX_ALIVE_DETECTMODE_ASYNC_LOWLEVEL; break;
	case IRQ_TYPE_LEVEL_HIGH:	mode = NX_ALIVE_DETECTMODE_ASYNC_HIGHLEVEL; break;
	default:
		pr_err("%s: No such irq type %d", __func__, type);
		return -1;
	}

	for ( ; 6 > i; i++, offs += 0x0C) {
		u32 reg = (i == mode ? ALIVE_MOD_SET : ALIVE_MOD_REST);
		writel(1<<bit, (base + reg  + offs));	/* set o reset mode */
	}

	/*
	 * set risingedge mode for both edge
	 * 0x2C : Risingedge
	 */
	if (IRQ_TYPE_EDGE_BOTH == type)
		writel(1<<bit, (base + 0x2C));

	writel(1<<bit, base + ALIVE_DET_SET);
	writel(1<<bit, base + ALIVE_INT_SET);
	writel(1<<bit, base + ALIVE_OUT_RESET);

	return 0;
}

static void irq_alive_enable(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	pr_debug("%s: alive irq=%d, io=%d\n", __func__, d->irq, bit);
	writel((1<<bit), base + ALIVE_INT_SET); ARM_DMB();	/* unmask:irq set (enable) */
}

static void irq_alive_disable(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	pr_debug("%s: alive irq=%d, io=%d\n", __func__, d->irq, bit);
	writel((1<<bit), base + ALIVE_INT_RESET);	/* mask:irq reset (disable) */
}

static struct irq_chip irq_alive_chip = {
	.name			= "ALIVE",
	.irq_ack		= irq_alive_ack,
	.irq_mask		= irq_alive_mask,
	.irq_unmask		= irq_alive_unmask,
	.irq_set_type	= irq_alive_set_type,
	.irq_enable		= irq_alive_enable,
	.irq_disable	= irq_alive_disable,
};

static void irq_alive_handler(unsigned int irq, struct irq_desc *desc)
{
	struct cpu_irq_domain_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_get_chip(irq);
	void __iomem *base = data->base;
	u32 stat, mask;
	int hwirq, bit;

	mask = readl(base + ALIVE_INT_SET_READ);
	stat = readl(base + ALIVE_INT_STATUS) & mask;
	bit = ffs(stat) - 1;
	hwirq = data->hwirq + bit;

	pr_debug("Alive irq=%d:%d (hw %d), stat=0x%02x, mask=0x%02x @%p\n",
		irq, bit, hwirq, stat, mask, data);

	if (-1 == bit) {
		pr_err( "Unknown Alive irq=%d, stat=0x%08x, mask=0x%02x\r\n",
			irq, stat, mask);
		writel(-1, (base + ALIVE_INT_STATUS));	/* clear alive status all */
		goto out;
	}

	irq = irq_find_mapping(data->domain, hwirq); /* data->first_irq + bit */

	if (unlikely(!irq))
		handle_bad_irq(irq, desc);
	else
		generic_handle_irq(irq);
out:
	/* EOI */
 	chained_irq_exit(chip, desc);
}

/*
 *  GPIO irq chain handler
 *  start  -> request_irq -> gpio irq_unmask
 *  do IRQ -> gpio handler -> gpio irq_mask -> gpio irq_ack -> driver handler -> gpio irq_unmask ->
 *  end    -> disable
 */
static const char *io_name[] = { "GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", };

#define	PIO_IRQ_BASE	IRQ_GPIOA
#define	VIO_IRQ_BASE	IRQ_GPIO_START
#define	VIO_NAME(i)		(io_name[(i-VIO_IRQ_BASE)/32])
#define	PIO_NAME(i)		(io_name[(i-PIO_IRQ_BASE)])

#define	GPIO_INT_OUT	(0x04)
#define	GPIO_INT_MODE0	(0x08)	// 0x08,0x0C
#define	GPIO_INT_MODE1	(0x28)
#define	GPIO_INT_ENB	(0x10)
#define	GPIO_INT_STATUS	(0x14)
#define	GPIO_INT_ALT	(0x20)	// 0x20,0x24
#define	GPIO_INT_DET	(0x3C)

static void irq_gpio_ack(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	writel((1<<bit), base + GPIO_INT_STATUS);	/* irq pend clear */
	ARM_DMB();
}

static void irq_gpio_mask(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* mask:irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1<<bit), base + GPIO_INT_DET);
}

static void irq_gpio_unmask(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* unmask:irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1<<bit), base + GPIO_INT_DET);
	ARM_DMB();
}

static int irq_gpio_set_type(struct irq_data *d, unsigned int type)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);
	u32 val, alt;
	ulong reg;

	NX_GPIO_INTMODE mode = 0;

	pr_debug("%s: gpio irq=%d, %s.%d, type=0x%x\n",
		__func__, d->irq, VIO_NAME(d->irq), bit, type);

	switch (type) {
	case IRQ_TYPE_NONE:	pr_warn("%s: No edge setting!\n", __func__);
		break;
	case IRQ_TYPE_EDGE_RISING:	mode = NX_GPIO_INTMODE_RISINGEDGE;	break;
	case IRQ_TYPE_EDGE_FALLING:	mode = NX_GPIO_INTMODE_FALLINGEDGE;	break;
	case IRQ_TYPE_EDGE_BOTH:	mode = NX_GPIO_INTMODE_BOTHEDGE;	break;
	case IRQ_TYPE_LEVEL_LOW:	mode = NX_GPIO_INTMODE_LOWLEVEL;	break;
	case IRQ_TYPE_LEVEL_HIGH:	mode = NX_GPIO_INTMODE_HIGHLEVEL;	break;
	default:
		pr_err("%s: No such irq type %d", __func__, type);
		return -1;
	}

	/* gpio out : output disable */
	writel(readl(base + GPIO_INT_OUT) & ~(1<<bit), base + GPIO_INT_OUT);

	/* gpio mode : interrupt mode */
	reg = (ulong)(base + GPIO_INT_MODE0 + (bit/16) * 4);
	val = (readl((void*)reg) & ~(3<<((bit&0xf) * 2))) | ((mode&0x3) << ((bit&0xf) * 2));
	writel(val, (void*)reg);

	reg = (ulong)(base + GPIO_INT_MODE1);
	val = (readl((void*)reg) & ~(1<<bit)) | (((mode>>2) & 0x1) << bit);
	writel(val, (void*)reg);

	/* gpio alt : gpio mode for irq */
	reg  = (ulong)(base + GPIO_INT_ALT + (bit/16) * 4);
	val  = readl((void*)reg) & ~(3<<((bit&0xf) * 2));
	alt  = GET_GPIO_ALTFUNC((d->irq-VIO_IRQ_BASE)/32, bit);
	val |= alt << ((bit&0xf) * 2);
	writel(val, (void*)reg);

	return 0;
}

static void irq_gpio_enable(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* unmask:irq enable */
	writel(readl(base + GPIO_INT_ENB) | (1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) | (1<<bit), base + GPIO_INT_DET);
}

static void irq_gpio_disable(struct irq_data *d)
{
	struct cpu_irq_domain_data *data = irq_data_get_irq_chip_data(d);
	void __iomem *base = data->base;
	int bit = irqdata_to_bit(d, data);

	pr_debug("%s: gpio irq=%d, %s.%d\n", __func__, d->irq, VIO_NAME(d->irq), bit);

	/* mask:irq disable */
	writel(readl(base + GPIO_INT_ENB) & ~(1<<bit), base + GPIO_INT_ENB);
	writel(readl(base + GPIO_INT_DET) & ~(1<<bit), base + GPIO_INT_DET);
}

static struct irq_chip irq_gpio_chip = {
	.name			= "GPIO",
	.irq_ack		= irq_gpio_ack,
	.irq_mask		= irq_gpio_mask,
	.irq_unmask		= irq_gpio_unmask,
	.irq_set_type	= irq_gpio_set_type,
	.irq_enable		= irq_gpio_enable,
	.irq_disable	= irq_gpio_disable,
};

static void irq_gpio_handler(unsigned int irq, struct irq_desc *desc)
{
	struct cpu_irq_domain_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_get_chip(irq);
	void __iomem *base = data->base;
	u32 stat, mask;
	int hwirq, bit;

	mask = readl(base + GPIO_INT_ENB);
	stat = readl(base + GPIO_INT_STATUS) & mask;
	bit  = ffs(stat) - 1;
	hwirq = data->hwirq + bit;

	pr_debug("Gpio irq=%d [%s.%d] (hw %d), stat=0x%08x, mask=0x%08x\n",
		irq, PIO_NAME(irq), bit, hwirq, stat, mask);

	if (-1 == bit) {
		pr_err("Unknown Gpio irq=%d, status=0x%08x, mask=0x%08x\r\n",
			irq, stat, mask);
		writel(-1, (base + GPIO_INT_STATUS));	/* clear gpio status all */
		goto out;
	}

	irq = irq_find_mapping(data->domain, hwirq); /* data->first_irq + bit */
	if (unlikely(!irq))
		handle_bad_irq(irq, desc);
	else
		generic_handle_irq(irq);
out:
	/* EOI */
 	chained_irq_exit(chip, desc);
}

static int irq_cpu_domain_gic_xlate(struct irq_domain *d,
					struct device_node *controller,
					const u32 *intspec, unsigned int intsize,
					unsigned long *out_hwirq, unsigned int *out_type)
{
	u32 irq = intspec[1];
	u32 new = irq;

	if (d->of_node != controller)
		return -EINVAL;

	if (3 > intsize)
		return -EINVAL;

	*out_hwirq = new;
	*out_type = intspec[2]; /* irq trigger type */

	pr_debug("gic xlate [%d -> %d]\n", irq, new);

	return 0;
}

static int irq_cpu_domain_xlate(struct irq_domain *d,
					struct device_node *controller,
					const u32 *intspec, unsigned int intsize,
					unsigned long *out_hwirq, unsigned int *out_type)
{
	u32 irq = intspec[1];
	u32 new = irq - low_first_irq;;

	if (d->of_node != controller)
		return -EINVAL;

	if (3 > intsize)
		return -EINVAL;

	*out_hwirq = new;
	*out_type = intspec[2]; /* irq trigger type */

	pr_debug("io  xlate [%d -> %d] low %d\n", irq, new, low_first_irq);

	return 0;
}

static int irq_cpu_domain_match(struct irq_domain *d,
					struct device_node *node)
{
	struct device_node *np = d->of_node;

	/*
	 * refer to dtsi file's gpio:io-interrupt DT
	 */
	int ret = strcmp(node->name, np->name) ? 0 : 1;

	pr_debug("domain_match: %p node %s:%s ret:%d (%p:%p)\n",
			d, node->name, np->name, ret, d->ops, d->ops->xlate);

	return ret;	/* 1: OK, 0: False */
}

static int irq_cpu_domain_map(struct irq_domain *d,
					unsigned int irq, irq_hw_number_t hw)
{
	struct cpu_irq_domain_data *data = NULL;

	list_for_each_entry(data, &irq_domain_link, link) {
		int start = (int)data->first_irq;
		int end = (int)(data->first_irq + data->size);
		if (end > irq && irq >= start)
			break;
	}
	pr_debug("domain_map: %p, io irq %d virt %d map hw %d\n",
		d, data->irqno, irq, (int)hw);

	irq_set_chip_data(irq, data);
	irq_set_chip_and_handler(irq, data->chip, handle_level_irq);
	set_irq_flags(irq, (IRQF_VALID | IRQF_PROBE));

	return 0;
}

static struct irq_domain_ops cpu_irq_domain_ops = {
	.match  = irq_cpu_domain_match,
	.map  = irq_cpu_domain_map,
	.xlate = irq_cpu_domain_xlate,
};

static void __init irq_cpu_hw_irq_handler_data(struct device_node *np,
					struct irq_domain *domain,
					struct cpu_irq_domain_data *data,
					int first_irq, int hwirq, int size, int offs_irq)
{
	if (!data)
		return;

	data->domain = domain;
	hwirq += offs_irq;

	if (irq_set_handler_data(hwirq, data))
		BUG();

	irq_set_chained_handler(hwirq, data->handler);

	printk("IRQ HW %d map to %d size %d\n", hwirq, first_irq, size);
}

static int irq_cpu_set_affinity(struct irq_data *d,
					const struct cpumask *mask_val, bool force)
{
	return IRQ_SET_MASK_OK;
}

static void __init irq_cpu_interface_setup(struct device_node *np,
					struct device_node *parent)
{
    struct irq_chip *d = NULL;
    struct irq_data *data;
    struct irq_domain_ops *ops = NULL;
	void __iomem *base;
    u32 cpumask = 0xff;	/* transfer interrupt to all cores */
    int nr_irq, i = 0;

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	/* get interface base */
 	base = of_iomap(parent, 0);
	WARN(!base, "%s: unable to map gic cpu registers\n", __func__);

#if 1
	nr_irq = readl_relaxed(base + GIC_DIST_CTR) & 0x1f;
	nr_irq = (nr_irq + 1) * 32;
	if (nr_irq > 1020)
		nr_irq = 1020;

	for (i = 32; i < nr_irq; i += 4)
		writel_relaxed(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);
	printk("GIC %d: core interrupt unmask\n", nr_irq);

	/* replace gic affinity for multiple irq */
	d = irq_get_chip(16);
	if (d->irq_set_affinity)
		d->irq_set_affinity = &irq_cpu_set_affinity;
#endif

	/* replace gic xlate */
	data = irq_get_irq_data(16);
	if (data->domain) {
		ops = (struct irq_domain_ops *)data->domain->ops;
		if (ops->xlate)
			ops->xlate = &irq_cpu_domain_gic_xlate;
	}
}

static int __init irq_cpu_of_parse_dt(struct device_node *node,
					struct cpu_irq_domain_data *data, int *offs_irq)
{
	struct device_node *child;
	int nr_irqs = 0;

	if (of_property_read_u32(node, "gic-interrupt-offset", offs_irq))
		*offs_irq = 0;

	for_each_child_of_node(node, child) {
		void __iomem *base;
		int irqno, first_irq;
		int size;

		if (!(base = of_iomap(child, 0)))
			continue;

		if (of_property_read_u32(child, "hw_irq", &irqno))
			continue;

		if (of_property_read_u32(child, "first_irq", &first_irq))
			continue;

		if (of_property_read_u32(child, "nr_irqs",  &size))
			continue;

		if (!of_node_cmp(child->type, "gpio")) {
			data->chip = &irq_gpio_chip;
			data->handler = &irq_gpio_handler;
		} else if (!of_node_cmp(child->type, "alive")) {
			data->chip = &irq_alive_chip;
			data->handler = &irq_alive_handler;
		//	irq_set_irq_wake(irqno, 1);
		} else {
			continue;
		}

		data->node = child;
		data->base = base;
		data->irqno = irqno;
		data->hwirq = nr_irqs;
		data->first_irq = first_irq;
		data->size = size;

		list_add(&data->link, &irq_domain_link);

		if (low_first_irq > first_irq)
			low_first_irq = first_irq;

		data++, nr_irqs += size;
		printk("HW IRQ[%d] %d %2dEA [Max:%3d] @%p\n",
			irqno, first_irq, size, nr_irqs, (void*)data);
	}

	return nr_irqs;
}

static int __init irq_cpu_of_setup(struct device_node *node,
					struct device_node *parent)
{
	struct irq_domain *domain;
	struct cpu_irq_domain_data *data;
	int offs_irq = 0, irqs = 0;
	int i = 0;
	int cnt = of_get_child_count(node);

	data = kzalloc(sizeof(*data)*cnt, GFP_KERNEL);
	if (!data) {
		pr_err("%s: Failed to kzalloc for interrupt!\n", __func__);
		return -EINVAL;
	}

	irqs = irq_cpu_of_parse_dt(node, data, &offs_irq);
	if (!irqs)
		return 0;

	domain = irq_domain_add_simple(node, irqs,
				data[0].first_irq, &cpu_irq_domain_ops, data);
	if (WARN_ON(!domain)) {
		pr_err("%s: irq domain init failed\n", __func__);
		kfree(data);
		return -EINVAL;
	}

	for (i = (cnt-1) ; i >= 0; i--)
		irq_cpu_hw_irq_handler_data(node, domain, &data[i],
				data[i].first_irq, data[i].irqno, data[i].size,
				offs_irq);

	irq_cpu_interface_setup(node, parent);

#ifdef CONFIG_FIQ
	init_FIQ();
#endif

	/* wake up source from idle */
#ifdef PM_RTC_WAKE
	irq_set_irq_wake(IRQ_RTC, 1);
#endif
	return 0;
}
IRQCHIP_DECLARE(s5p6818_gic, "nexell,s5p6818-gic-intc", irq_cpu_of_setup);

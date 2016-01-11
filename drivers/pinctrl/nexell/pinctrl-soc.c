#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/err.h>

#include "pinctrl-nxp.h"
#include "pinctrl-soc.h"

struct soc_irq_chip {
	struct irq_chip chip;

	u32 eint_con;
	u32 eint_mask;
	u32 eint_pend;
};

static inline struct soc_irq_chip *to_soc_irq_chip(struct irq_chip *chip)
{
	return container_of(chip, struct soc_irq_chip, chip);
}

static struct nexell_pin_bank_type bank_type_off = {
	.fld_width = { 4, 1, 2, 2, 2, 2, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, },
};

static struct nexell_pin_bank_type bank_type_alive = {
	.fld_width = { 4, 1, 2, 2, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

/* list of external wakeup controllers supported */
static const struct of_device_id soc_wkup_irq_ids[] = {
	{ .compatible = "nexell,soc4210-wakeup-eint", },
	{ }
};

static void soc_irq_mask(struct irq_data *irqd)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned long reg_mask = our_chip->eint_mask + bank->eint_offset;
	unsigned long mask;
	unsigned long flags;

	spin_lock_irqsave(&bank->slock, flags);

	mask = readl(d->virt_base + reg_mask);
	mask |= 1 << irqd->hwirq;
	writel(mask, d->virt_base + reg_mask);

	spin_unlock_irqrestore(&bank->slock, flags);
}

static void soc_irq_ack(struct irq_data *irqd)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned long reg_pend = our_chip->eint_pend + bank->eint_offset;

	writel(1 << irqd->hwirq, d->virt_base + reg_pend);
}

static void soc_irq_unmask(struct irq_data *irqd)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned long reg_mask = our_chip->eint_mask + bank->eint_offset;
	unsigned long mask;
	unsigned long flags;

	/*
	 * Ack level interrupts right before unmask
	 *
	 * If we don't do this we'll get a double-interrupt.  Level triggered
	 * interrupts must not fire an interrupt if the level is not
	 * _currently_ active, even if it was active while the interrupt was
	 * masked.
	 */
	if (irqd_get_trigger_type(irqd) & IRQ_TYPE_LEVEL_MASK)
		soc_irq_ack(irqd);

	spin_lock_irqsave(&bank->slock, flags);

	mask = readl(d->virt_base + reg_mask);
	mask &= ~(1 << irqd->hwirq);
	writel(mask, d->virt_base + reg_mask);

	spin_unlock_irqrestore(&bank->slock, flags);
}

static int soc_irq_set_type(struct irq_data *irqd, unsigned int type)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned int shift = SOC_EINT_CON_LEN * irqd->hwirq;
	unsigned int con, trig_type;
	unsigned long reg_con = our_chip->eint_con + bank->eint_offset;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		trig_type = SOC_EINT_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trig_type = SOC_EINT_EDGE_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		trig_type = SOC_EINT_EDGE_BOTH;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trig_type = SOC_EINT_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trig_type = SOC_EINT_LEVEL_LOW;
		break;
	default:
		pr_err("unsupported external interrupt type\n");
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_BOTH)
		__irq_set_handler_locked(irqd->irq, handle_edge_irq);
	else
		__irq_set_handler_locked(irqd->irq, handle_level_irq);

	con = readl(d->virt_base + reg_con);
	con &= ~(SOC_EINT_CON_MASK << shift);
	con |= trig_type << shift;
	writel(con, d->virt_base + reg_con);

	return 0;
}

static int soc_irq_request_resources(struct irq_data *irqd)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pin_bank_type *bank_type = bank->type;
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned int shift = SOC_EINT_CON_LEN * irqd->hwirq;
	unsigned long reg_con = our_chip->eint_con + bank->eint_offset;
	unsigned long flags;
	unsigned int mask;
	unsigned int con;
	int ret;

	ret = gpio_lock_as_irq(&bank->gpio_chip, irqd->hwirq);
	if (ret) {
		dev_err(bank->gpio_chip.dev, "unable to lock pin %s-%lu IRQ\n",
			bank->name, irqd->hwirq);
		return ret;
	}

	reg_con = bank->pctl_offset + bank_type->reg_offset[PINCFG_TYPE_FUNC];
	shift = irqd->hwirq * bank_type->fld_width[PINCFG_TYPE_FUNC];
	mask = (1 << bank_type->fld_width[PINCFG_TYPE_FUNC]) - 1;

	spin_lock_irqsave(&bank->slock, flags);

	con = readl(d->virt_base + reg_con);
	con &= ~(mask << shift);
	con |= SOC_EINT_FUNC << shift;
	writel(con, d->virt_base + reg_con);

	spin_unlock_irqrestore(&bank->slock, flags);

	soc_irq_unmask(irqd);

	return 0;
}

static void soc_irq_release_resources(struct irq_data *irqd)
{
	struct irq_chip *chip = irq_data_get_irq_chip(irqd);
	struct soc_irq_chip *our_chip = to_soc_irq_chip(chip);
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	struct nexell_pin_bank_type *bank_type = bank->type;
	struct nexell_pinctrl_drv_data *d = bank->drvdata;
	unsigned int shift = SOC_EINT_CON_LEN * irqd->hwirq;
	unsigned long reg_con = our_chip->eint_con + bank->eint_offset;
	unsigned long flags;
	unsigned int mask;
	unsigned int con;

	reg_con = bank->pctl_offset + bank_type->reg_offset[PINCFG_TYPE_FUNC];
	shift = irqd->hwirq * bank_type->fld_width[PINCFG_TYPE_FUNC];
	mask = (1 << bank_type->fld_width[PINCFG_TYPE_FUNC]) - 1;

	soc_irq_mask(irqd);

	spin_lock_irqsave(&bank->slock, flags);

	con = readl(d->virt_base + reg_con);
	con &= ~(mask << shift);
	con |= FUNC_INPUT << shift;
	writel(con, d->virt_base + reg_con);

	spin_unlock_irqrestore(&bank->slock, flags);

	gpio_unlock_as_irq(&bank->gpio_chip, irqd->hwirq);
}

/*
 * irq_chip for gpio interrupts.
 */
static struct soc_irq_chip soc_gpio_irq_chip = {
	.chip = {
		.name = "soc_gpio_irq_chip",
		.irq_unmask = soc_irq_unmask,
		.irq_mask = soc_irq_mask,
		.irq_ack = soc_irq_ack,
		.irq_set_type = soc_irq_set_type,
		.irq_request_resources = soc_irq_request_resources,
		.irq_release_resources = soc_irq_release_resources,
	},
	.eint_con = SOC_GPIO_ECON_OFFSET,
	.eint_mask = SOC_GPIO_EMASK_OFFSET,
	.eint_pend = SOC_GPIO_EPEND_OFFSET,
};

static int soc_gpio_irq_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	struct nexell_pin_bank *b = h->host_data;

	irq_set_chip_data(virq, b);
	irq_set_chip_and_handler(virq, &soc_gpio_irq_chip.chip,
					handle_level_irq);
	set_irq_flags(virq, IRQF_VALID);
	return 0;
}

/*
 * irq domain callbacks for external gpio interrupt controller.
 */
static const struct irq_domain_ops soc_gpio_irqd_ops = {
	.map	= soc_gpio_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static irqreturn_t soc_eint_gpio_irq(int irq, void *data)
{
	struct nexell_pinctrl_drv_data *d = data;
	struct nexell_pin_ctrl *ctrl = d->ctrl;
	struct nexell_pin_bank *bank = ctrl->pin_banks;
	unsigned int svc, group, pin, virq;

	svc = readl(d->virt_base + SOC_SVC_OFFSET);
	group = SOC_SVC_GROUP(svc);
	pin = svc & SOC_SVC_NUM_MASK;

	if (!group)
		return IRQ_HANDLED;
	bank += (group - 1);

	virq = irq_linear_revmap(bank->irq_domain, pin);
	if (!virq)
		return IRQ_NONE;
	generic_handle_irq(virq);
	return IRQ_HANDLED;
}

struct soc_eint_gpio_save {
	u32 eint_con;
	u32 eint_fltcon0;
	u32 eint_fltcon1;
};

/*
 * s5p6818_eint_gpio_init() - setup handling of external gpio interrupts.
 * @d: driver data of nexell pinctrl driver.
 */
static int s5p6818_eint_gpio_init(struct nexell_pinctrl_drv_data *d)
{
	struct nexell_pin_bank *bank;
	struct device *dev = d->dev;
	int ret;
	int i;

	if (!d->irq) {
		dev_err(dev, "irq number not available\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, d->irq, soc_eint_gpio_irq,
					0, dev_name(dev), d);
	if (ret) {
		dev_err(dev, "irq request failed\n");
		return -ENXIO;
	}

	bank = d->ctrl->pin_banks;
	for (i = 0; i < d->ctrl->nr_banks; ++i, ++bank) {
		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;
		bank->irq_domain = irq_domain_add_linear(bank->of_node,
				bank->nr_pins, &soc_gpio_irqd_ops, bank);
		if (!bank->irq_domain) {
			dev_err(dev, "gpio irq domain add failed\n");
			ret = -ENXIO;
			goto err_domains;
		}

		bank->soc_priv = devm_kzalloc(d->dev,
			sizeof(struct soc_eint_gpio_save), GFP_KERNEL);
		if (!bank->soc_priv) {
			irq_domain_remove(bank->irq_domain);
			ret = -ENOMEM;
			goto err_domains;
		}
	}

	return 0;

err_domains:
	for (--i, --bank; i >= 0; --i, --bank) {
		if (bank->eint_type != EINT_TYPE_GPIO)
			continue;
		irq_domain_remove(bank->irq_domain);
	}

	return ret;
}

static u32 soc_eint_wake_mask = 0xffffffff;

u32 soc_get_eint_wake_mask(void)
{
	return soc_eint_wake_mask;
}

static int soc_wkup_irq_set_wake(struct irq_data *irqd, unsigned int on)
{
	struct nexell_pin_bank *bank = irq_data_get_irq_chip_data(irqd);
	unsigned long bit = 1UL << (2 * bank->eint_offset + irqd->hwirq);

	pr_info("wake %s for irq %d\n", on ? "enabled" : "disabled", irqd->irq);

	if (!on)
		soc_eint_wake_mask |= bit;
	else
		soc_eint_wake_mask &= ~bit;

	return 0;
}

/*
 * irq_chip for wakeup interrupts
 */
static struct soc_irq_chip soc_wkup_irq_chip = {
	.chip = {
		.name = "soc_wkup_irq_chip",
		.irq_unmask = soc_irq_unmask,
		.irq_mask = soc_irq_mask,
		.irq_ack = soc_irq_ack,
		.irq_set_type = soc_irq_set_type,
		.irq_set_wake = soc_wkup_irq_set_wake,
		.irq_request_resources = soc_irq_request_resources,
		.irq_release_resources = soc_irq_release_resources,
	},
	.eint_con = SOC_WKUP_ECON_OFFSET,
	.eint_mask = SOC_WKUP_EMASK_OFFSET,
	.eint_pend = SOC_WKUP_EPEND_OFFSET,
};

/* interrupt handler for wakeup interrupts 0..15 */
static void soc_irq_eint0_15(unsigned int irq, struct irq_desc *desc)
{
	struct soc_weint_data *eintd = irq_get_handler_data(irq);
	struct nexell_pin_bank *bank = eintd->bank;
	struct irq_chip *chip = irq_get_chip(irq);
	int eint_irq;

	chained_irq_enter(chip, desc);
	chip->irq_mask(&desc->irq_data);

	if (chip->irq_ack)
		chip->irq_ack(&desc->irq_data);

	eint_irq = irq_linear_revmap(bank->irq_domain, eintd->irq);
	generic_handle_irq(eint_irq);
	chip->irq_unmask(&desc->irq_data);
	chained_irq_exit(chip, desc);
}

static inline void soc_irq_demux_eint(unsigned long pend,
						struct irq_domain *domain)
{
	unsigned int irq;

	while (pend) {
		irq = fls(pend) - 1;
		generic_handle_irq(irq_find_mapping(domain, irq));
		pend &= ~(1 << irq);
	}
}

/* interrupt handler for wakeup interrupt 16 */
#if 0
static void soc_irq_demux_eint16_31(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_get_chip(irq);
	struct soc_muxed_weint_data *eintd = irq_get_handler_data(irq);
	struct nexell_pinctrl_drv_data *d = eintd->banks[0]->drvdata;
	unsigned long pend;
	unsigned long mask;
	int i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < eintd->nr_banks; ++i) {
		struct nexell_pin_bank *b = eintd->banks[i];
		pend = readl(d->virt_base + SOC_WKUP_EPEND_OFFSET
				+ b->eint_offset);
		mask = readl(d->virt_base + SOC_WKUP_EMASK_OFFSET
				+ b->eint_offset);
		soc_irq_demux_eint(pend & ~mask, b->irq_domain);
	}

	chained_irq_exit(chip, desc);
}
#endif

static int soc_wkup_irq_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &soc_wkup_irq_chip.chip,
					handle_level_irq);
	irq_set_chip_data(virq, h->host_data);
	set_irq_flags(virq, IRQF_VALID);
	return 0;
}

/*
 * irq domain callbacks for external wakeup interrupt controller.
 */
static const struct irq_domain_ops soc_wkup_irqd_ops = {
	.map	= soc_wkup_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

/*
 * soc_eint_wkup_init() - setup handling of external wakeup interrupts.
 * @d: driver data of nexell pinctrl driver.
 */
static int soc_eint_wkup_init(struct nexell_pinctrl_drv_data *d)
{
	struct device *dev = d->dev;
	struct device_node *wkup_np = NULL;
	struct device_node *np;
	struct nexell_pin_bank *bank;
	struct soc_weint_data *weint_data;
	struct soc_muxed_weint_data *muxed_data;
	unsigned int muxed_banks = 0;
	unsigned int i;
	int idx, irq;

	for_each_child_of_node(dev->of_node, np) {
		if (of_match_node(soc_wkup_irq_ids, np)) {
			wkup_np = np;
			break;
		}
	}
	if (!wkup_np)
		return -ENODEV;

	bank = d->ctrl->pin_banks;
	for (i = 0; i < d->ctrl->nr_banks; ++i, ++bank) {
		if (bank->eint_type != EINT_TYPE_WKUP)
			continue;

		bank->irq_domain = irq_domain_add_linear(bank->of_node,
				bank->nr_pins, &soc_wkup_irqd_ops, bank);
		if (!bank->irq_domain) {
			dev_err(dev, "wkup irq domain add failed\n");
			return -ENXIO;
		}

		if (!of_find_property(bank->of_node, "interrupts", NULL)) {
			bank->eint_type = EINT_TYPE_WKUP_MUX;
			++muxed_banks;
			continue;
		}

		weint_data = devm_kzalloc(dev, bank->nr_pins
					* sizeof(*weint_data), GFP_KERNEL);
		if (!weint_data) {
			dev_err(dev, "could not allocate memory for weint_data\n");
			return -ENOMEM;
		}

		for (idx = 0; idx < bank->nr_pins; ++idx) {
			irq = irq_of_parse_and_map(bank->of_node, idx);
			if (!irq) {
				dev_err(dev, "irq number for eint-%s-%d not found\n",
							bank->name, idx);
				continue;
			}
			weint_data[idx].irq = idx;
			weint_data[idx].bank = bank;
			irq_set_handler_data(irq, &weint_data[idx]);
			irq_set_chained_handler(irq, soc_irq_eint0_15);
		}
	}

	if (!muxed_banks)
		return 0;

	irq = irq_of_parse_and_map(wkup_np, 0);
	if (!irq) {
		dev_err(dev, "irq number for muxed EINTs not found\n");
		return 0;
	}

	muxed_data = devm_kzalloc(dev, sizeof(*muxed_data)
		+ muxed_banks*sizeof(struct nexell_pin_bank *), GFP_KERNEL);
	if (!muxed_data) {
		dev_err(dev, "could not allocate memory for muxed_data\n");
		return -ENOMEM;
	}

	//irq_set_chained_handler(irq, soc_irq_demux_eint16_31);
	irq_set_handler_data(irq, muxed_data);

	bank = d->ctrl->pin_banks;
	idx = 0;
	for (i = 0; i < d->ctrl->nr_banks; ++i, ++bank) {
		if (bank->eint_type != EINT_TYPE_WKUP_MUX)
			continue;

		muxed_data->banks[idx++] = bank;
	}
	muxed_data->nr_banks = muxed_banks;

	return 0;
}

static void soc_pinctrl_suspend_bank(
				struct nexell_pinctrl_drv_data *drvdata,
				struct nexell_pin_bank *bank)
{
	struct soc_eint_gpio_save *save = bank->soc_priv;
	void __iomem *regs = drvdata->virt_base;

	save->eint_con = readl(regs + SOC_GPIO_ECON_OFFSET
						+ bank->eint_offset);
	save->eint_fltcon0 = readl(regs + SOC_GPIO_EFLTCON_OFFSET
						+ 2 * bank->eint_offset);
	save->eint_fltcon1 = readl(regs + SOC_GPIO_EFLTCON_OFFSET
						+ 2 * bank->eint_offset + 4);

	pr_debug("%s: save     con %#010x\n", bank->name, save->eint_con);
	pr_debug("%s: save fltcon0 %#010x\n", bank->name, save->eint_fltcon0);
	pr_debug("%s: save fltcon1 %#010x\n", bank->name, save->eint_fltcon1);
}

static void soc_pinctrl_suspend(struct nexell_pinctrl_drv_data *drvdata)
{
	struct nexell_pin_ctrl *ctrl = drvdata->ctrl;
	struct nexell_pin_bank *bank = ctrl->pin_banks;
	int i;

	for (i = 0; i < ctrl->nr_banks; ++i, ++bank)
		if (bank->eint_type == EINT_TYPE_GPIO)
			soc_pinctrl_suspend_bank(drvdata, bank);
}

static void soc_pinctrl_resume_bank(
				struct nexell_pinctrl_drv_data *drvdata,
				struct nexell_pin_bank *bank)
{
	struct soc_eint_gpio_save *save = bank->soc_priv;
	void __iomem *regs = drvdata->virt_base;

	pr_debug("%s:     con %#010x => %#010x\n", bank->name,
			readl(regs + SOC_GPIO_ECON_OFFSET
			+ bank->eint_offset), save->eint_con);
	pr_debug("%s: fltcon0 %#010x => %#010x\n", bank->name,
			readl(regs + SOC_GPIO_EFLTCON_OFFSET
			+ 2 * bank->eint_offset), save->eint_fltcon0);
	pr_debug("%s: fltcon1 %#010x => %#010x\n", bank->name,
			readl(regs + SOC_GPIO_EFLTCON_OFFSET
			+ 2 * bank->eint_offset + 4), save->eint_fltcon1);

	writel(save->eint_con, regs + SOC_GPIO_ECON_OFFSET
						+ bank->eint_offset);
	writel(save->eint_fltcon0, regs + SOC_GPIO_EFLTCON_OFFSET
						+ 2 * bank->eint_offset);
	writel(save->eint_fltcon1, regs + SOC_GPIO_EFLTCON_OFFSET
						+ 2 * bank->eint_offset + 4);
}

static void soc_pinctrl_resume(struct nexell_pinctrl_drv_data *drvdata)
{
	struct nexell_pin_ctrl *ctrl = drvdata->ctrl;
	struct nexell_pin_bank *bank = ctrl->pin_banks;
	int i;

	for (i = 0; i < ctrl->nr_banks; ++i, ++bank)
		if (bank->eint_type == EINT_TYPE_GPIO)
			soc_pinctrl_resume_bank(drvdata, bank);
}

/* pin banks of s5p6818 pin-controller 0 */
static struct nexell_pin_bank s5p6818_pin_banks[] = {
	SOC_PIN_BANK_EINTN(32, 0xA000, "gpioa"),
	SOC_PIN_BANK_EINTN(32, 0xB000, "gpiob"),
	SOC_PIN_BANK_EINTN(32, 0xC000, "gpioc"),
	SOC_PIN_BANK_EINTN(32, 0xD000, "gpiod"),
	SOC_PIN_BANK_EINTN(32, 0xE000, "gpioe"),
};

/*
 * Nexell pinctrl driver data for SoC.
 */
struct nexell_pin_ctrl s5p6818_pin_ctrl[] = {
	{
		/* pin-controller instance 0 data */
		.pin_banks	= s5p6818_pin_banks,
		.nr_banks	= ARRAY_SIZE(s5p6818_pin_banks),
		//.eint_gpio_init = s5p6818_eint_gpio_init,
		//.suspend	= s5p6818_pinctrl_suspend,
		//.resume		= s5p6818_pinctrl_resume,
		.label		= "s5p6818-gpio-ctrl",
	}
};

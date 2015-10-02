/*
 * Allwinner SoCs Reset Controller driver
 *
 * Copyright 2013 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/delay.h>

#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

/*
#define	pr_debug	printk
*/


#define	DEALY(s)	mdelay(s)

#define nx_assert(p) do {	\
	if (!(p)) {	\
		pr_err("BUG at %s:%d assert(%s)\n",	\
		       __FILE__, __LINE__, #p);			\
		BUG();	\
	}		\
} while (0)


struct nexell_reset_data {
	spinlock_t					lock;
	void __iomem				*base;					/* mapped address */
	struct reset_controller_dev	rcdev;
};

static int nexell_reset_reset(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct nexell_reset_data *data = container_of(rcdev,
						     struct nexell_reset_data,
						     rcdev);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%ld [0x%p]\n", __func__, id, data->base);
	nx_assert(RESET_ID_END >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return -EINVAL;
	}

	spin_lock_irqsave(&data->lock, flags);

	NX_RSTCON_SetBaseAddress(data->base);

	#if 1
	if (NX_RSTCON_GetRST(RSTIndex) == 1)		/* pass through if reset already deasserted */
	{
		pr_debug("reset already deasserted\n");
	}
	else
	#endif
	{
		NX_RSTCON_SetRST(RSTIndex, RSTCON_ASSERT);
		DEALY(1);
		NX_RSTCON_SetRST(RSTIndex, RSTCON_NEGATE);
	}

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}


static int nexell_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct nexell_reset_data *data = container_of(rcdev,
						     struct nexell_reset_data,
						     rcdev);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%ld [0x%p]\n", __func__, id, data->base);
	nx_assert(RESET_ID_END >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return -EINVAL;
	}

	spin_lock_irqsave(&data->lock, flags);

	NX_RSTCON_SetBaseAddress(data->base);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_ASSERT);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int nexell_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct nexell_reset_data *data = container_of(rcdev,
						     struct nexell_reset_data,
						     rcdev);
	unsigned long flags;
	U32 RSTIndex = id;

	pr_debug("%s: id=%ld [0x%p]\n", __func__, id, data->base);
	nx_assert(RESET_ID_END >= RSTIndex);

	if (RESET_ID_CPU1   == RSTIndex ||
		RESET_ID_CPU2   == RSTIndex ||
		RESET_ID_CPU3   == RSTIndex ||
		RESET_ID_DREX   == RSTIndex ||
		RESET_ID_DREX_A == RSTIndex ||
		RESET_ID_DREX_C == RSTIndex) {
		printk("Invalid reset id %d ...\n", RSTIndex);
		return -EINVAL;
	}

	spin_lock_irqsave(&data->lock, flags);

	NX_RSTCON_SetBaseAddress(data->base);
	NX_RSTCON_SetRST(RSTIndex, RSTCON_NEGATE);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}


static struct reset_control_ops nexell_reset_ops = {
	.reset		= nexell_reset_reset,
	.assert		= nexell_reset_assert,
	.deassert	= nexell_reset_deassert,
};


static const struct of_device_id nexell_reset_dt_ids[] = {
	 { .compatible = "nexell,nxp-reset", },
	 { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, nexell_reset_dt_ids);

static int nexell_reset_probe(struct platform_device *pdev)
{
	struct nexell_reset_data *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	spin_lock_init(&data->lock);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = resource_size(res) * 32;
	data->rcdev.ops = &nexell_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;
	dev_info(&pdev->dev, "nexell reset: nr_resets [%d], base [%p]\n", data->rcdev.nr_resets, data->base);

	return reset_controller_register(&data->rcdev);
}

static int nexell_reset_remove(struct platform_device *pdev)
{
	struct nexell_reset_data *data = platform_get_drvdata(pdev);

	reset_controller_unregister(&data->rcdev);

	return 0;
}

static struct platform_driver nexell_reset_driver = {
	.probe	= nexell_reset_probe,
	.remove	= nexell_reset_remove,
	.driver = {
		.name		= "nxp-reset",
		.owner		= THIS_MODULE,
		.of_match_table	= nexell_reset_dt_ids,
	},
};
//module_platform_driver(nexell_reset_driver);
static int __init nexell_reset_init(void)
{
	return platform_driver_register(&nexell_reset_driver);
}
core_initcall(nexell_reset_init);

MODULE_AUTHOR("KOO Bon-Gyu <freestyle@nexell.co.kr");
MODULE_DESCRIPTION("Reset Controller Driver for Nexell");
MODULE_LICENSE("GPL");

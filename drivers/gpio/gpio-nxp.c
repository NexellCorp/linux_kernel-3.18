/*
 * Copyright (C) 2008, 2009 Provigent Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver for the ARM PrimeCell(tm) General Purpose Input/Output (PL061)
 *
 * Data sheet: ARM DDI 0190B, September 2000
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/pm.h>

#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

/*
#define pr_debug	printk
*/

struct nxp_gpio_data {
	int index; 		/* Bank Index : A(0), B(1), C(2), D(3), E(4), ALIVE(5) */
	spinlock_t	lock;		/* GPIO registers */
	struct gpio_chip chip;
};

static const char *io_name[] = {
	"GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", "GPIOALV"
};
#define	chip_to_gpio(c)	container_of(c, struct nxp_gpio_data, chip)

static int nxp_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	int io = gpio->index * GPIO_NUM_PER_BANK + offset;
	int fn = GET_GPIO_ALTFUNC(gpio->index, offset);

	pr_debug("gpio_request: %s.%02d=%3d\n",
		io_name[gpio->index], offset, io);

	nxp_soc_gpio_set_io_func(io, fn);

	return 0;
}

static int nxp_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	int io = gpio->index * GPIO_NUM_PER_BANK + offset;
	int fn = GET_GPIO_ALTFUNC(gpio->index, offset);

	pr_debug("direction_input: %s.%02d=%3d\n",
		io_name[gpio->index], offset, io);

	nxp_soc_gpio_set_io_func(io, fn);
	nxp_soc_gpio_set_io_dir(io, 0);

	return 0;
}

static int nxp_gpio_direction_output(struct gpio_chip *chip,
						unsigned offset, int value)
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	int io = gpio->index * GPIO_NUM_PER_BANK + offset;
	int fn = GET_GPIO_ALTFUNC(gpio->index, offset);

	pr_debug("direction_output: %s.%2d=%3d, val=%d\n",
		io_name[gpio->index], offset, io, value);

	nxp_soc_gpio_set_io_func(io, fn);
	nxp_soc_gpio_set_out_value(io, value);
	nxp_soc_gpio_set_io_dir(io, 1);

	return 0;
}

static int nxp_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	int io = gpio->index * GPIO_NUM_PER_BANK + offset;

	pr_debug("get_value: %s.%2d=%3d\n", io_name[gpio->index], offset, io);

	return nxp_soc_gpio_get_in_value(io);
}

static void nxp_gpio_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	int io = gpio->index * GPIO_NUM_PER_BANK + offset;

	nxp_soc_gpio_set_io_dir(io, 1);
	nxp_soc_gpio_set_out_value(io, value);

	pr_debug("set_value: %s.%2d=%3d, val=%d\n",
		io_name[gpio->index], offset, io, value);
}

static int nxp_gpio_to_irq( struct gpio_chip *chip , unsigned offset )
{
	struct nxp_gpio_data *gpio = chip_to_gpio(chip);
	unsigned int io = gpio->index * GPIO_NUM_PER_BANK + offset;

	return (io + IRQ_GPIO_START);
}

#ifdef CONFIG_PM
static int nxp_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	pm_dbgout("%s\n", __func__);
	return 0;
}

static int nxp_gpio_resume(struct platform_device *pdev)
{
	pm_dbgout("%s\n", __func__);
	return 0;
}
#else
#define	nxp_gpio_suspend	NULL
#define	nxp_gpio_resume		NULL
#endif

#ifdef CONFIG_OF
static void nxp_gpio_parse_dt(struct device_node *np,struct platform_device *pdev)
{
	u32 value;

	pdev->resource->start = 0;
	pdev->resource->end = 0;

	if (!of_property_read_u32(np, "nexell,gpionums", &value))
		pdev->resource->end = value;

	if (!of_property_read_u32(np, "nexell,gpioid", &value));
		pdev->id = value;

	pr_debug("%s.%d: %d ~ %2d\n", np->name, pdev->id,
		(int)pdev->resource->start, (int)pdev->resource->end);
}
#endif
static int nxp_gpio_probe(struct platform_device *pdev)
{
	struct nxp_gpio_data *gpio = NULL;
	struct resource *res = NULL;
	int ret;

	nxp_gpio_parse_dt(pdev->dev.of_node, pdev);

	res = pdev->resource;
	if (!res) {
		printk("Error: not allocated gpio resource [%d]\n", pdev->id);
		return -EINVAL;
	}

	gpio = kzalloc(sizeof(*gpio), GFP_KERNEL);
	if (gpio == NULL)
		return -ENOMEM;

	spin_lock_init(&gpio->lock);

	gpio->index = pdev->id;
	gpio->chip.request = nxp_gpio_request;
	gpio->chip.to_irq = nxp_gpio_to_irq;
	gpio->chip.direction_input = nxp_gpio_direction_input;
	gpio->chip.direction_output = nxp_gpio_direction_output;
	gpio->chip.get = nxp_gpio_get_value;
	gpio->chip.set = nxp_gpio_set_value;
	gpio->chip.ngpio = res->end - res->start;
	gpio->chip.label = dev_name(&pdev->dev);
	gpio->chip.dev = &pdev->dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = pdev->id * GPIO_NUM_PER_BANK;

	/* register GPIOLib */
	ret = gpiochip_add(&gpio->chip);
	if (ret)
		goto free_mem;

	return ret;

free_mem:
	kfree(gpio);
	return ret;
}

static int nxp_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nxp_gpio_match[] = {
    { .compatible = "nexell,nxp-gpio" },
    {},
};
MODULE_DEVICE_TABLE(of, nxp_gpio_match);
#else
#define nxp_gpio_match NULL
#endif

static struct platform_driver nxp_gpio_driver = {
	.probe		= nxp_gpio_probe,
	.remove		= nxp_gpio_remove,
	.suspend	= nxp_gpio_suspend,
	.resume		= nxp_gpio_resume,
	.driver		= {
		.name	= "nxp-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = nxp_gpio_match,
	},
};

static int __init nxp_gpio_init(void)
{
	return platform_driver_register(&nxp_gpio_driver);
}
subsys_initcall(nxp_gpio_init);

MODULE_DESCRIPTION("GPIO driver for the Nexell");
MODULE_LICENSE("GPL");

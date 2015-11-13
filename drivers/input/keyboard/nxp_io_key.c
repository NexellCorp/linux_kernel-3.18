/*
 * (C) Copyright 2010
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <nexell/platform.h>
#include <nexell/io_key.h>

/*
#define pr_debug	printk
*/

#define	DEV_NAME_KEYPAD			"nxp-keypad"

#define	KEY_IO_PRESS			(0)
#define	KEY_IO_RELEASE			(1)
#define	DELAY_JIFFIES 			(1)
#define	POWER_KEY_DELAY 		(300)	/* ms */
#define	KEY_STATUS(p)			(KEY_IO_PRESS == p? "DN" : "UP")

struct key_pad_info {
	struct input_dev *input;
	struct key_pad_code *keys;
	int	key_num;
	struct delayed_work	resume_work;
	int	resume_delay;
};

struct key_pad_code {
	struct key_pad_info *info;
	int pin;
	int code;
	int press;		/* current detect mode */
	int active_high;	/* detect edge */
	int irq_disabled;
	struct delayed_work work;
	struct workqueue_struct *wq;
};

int __weak pm_wake_is_power_key(int pin)
{
	return 0;
}

static void key_pad_work(struct work_struct *work)
{
	struct key_pad_code *key =
			container_of(work, struct key_pad_code, work.work);
	struct key_pad_info *info = key->info;
	int press = 0;
	unsigned long flags;

	local_irq_save(flags);

	press = gpio_get_value_cansleep(key->pin);

	if (key->active_high)
		press = !press;

	local_irq_restore(flags);

	if(press == key->press)
		return;

	key->press = press;
	input_report_key(info->input, key->code, (KEY_IO_PRESS == press ? 1 : 0));
	input_sync(info->input);

	pr_debug("key pin:%d = %4d %s\n", key->pin, key->code, KEY_STATUS(press));
}

static irqreturn_t key_pad_interrupt(int irqno, void *dev_id)
{
	struct key_pad_code *key = dev_id;
	int delay = DELAY_JIFFIES;
	queue_delayed_work(key->wq, &key->work, delay);
	return IRQ_HANDLED;
}

static void key_pad_resume_work(struct work_struct *work)
{
	struct key_pad_info *info =
		container_of(work, struct key_pad_info, resume_work.work);
	int i = 0;

	for ( ; info->key_num > i; i++) {
		if (info->keys[i].code == KEY_POWER &&
			info->keys[i].irq_disabled) {
			info->keys[i].irq_disabled = 0;
			enable_irq(gpio_to_irq(info->keys[i].pin));
			break;
		}
	}
}

static int key_pad_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct key_pad_info *info = platform_get_drvdata(pdev);
	int i = 0;

	for ( ; info->key_num > i; i++) {
		if (info->keys[i].code == KEY_POWER &&
			info->keys[i].irq_disabled) {
			info->keys[i].irq_disabled = 0;
			enable_irq(gpio_to_irq(info->keys[i].pin));
			break;
		}
	}
	return 0;
}

static int key_pad_resume(struct platform_device *pdev)
{
	struct key_pad_info *info = platform_get_drvdata(pdev);
	int delay = info->resume_delay ?
				info->resume_delay : POWER_KEY_DELAY;
	int i = 0;

	pm_dbgout("%s\n", __func__);

	for (i = 0; info->key_num > i; i++) {
		if (info->keys[i].code != KEY_POWER ||
			!pm_wake_is_power_key(info->keys[i].pin))
			continue;

		input_report_key(info->input, KEY_POWER, 1);
		input_sync(info->input);
		input_report_key(info->input, KEY_POWER, 0);
    	input_sync(info->input);

    	info->keys[i].irq_disabled = 1;
		disable_irq(gpio_to_irq(info->keys[i].pin));
		schedule_delayed_work(&info->resume_work, msecs_to_jiffies(delay));
	}

	return 0;
}

#ifdef CONFIG_OF
#define	CONFIG_KEY_IO_MAX_BUTTONS	32
#define	KEY_MAX_BUTTONS	CONFIG_KEY_IO_MAX_BUTTONS

static unsigned int key_buttons[3][KEY_MAX_BUTTONS];
static struct key_pad_plat_data key_dt_data = {
	.buttons 	  = key_buttons[0],
	.keycodes	  = key_buttons[1],
	.active_high  = key_buttons[2],
	.autorepeat	  = 0,
};

static const struct of_device_id key_dt_match[] = {
	{
	.compatible = "nexell,nxp-keypad",
	.data = (void*)&key_dt_data,
	}, {},
};
MODULE_DEVICE_TABLE(of, key_dt_match);

#define	LIST_ARRAY_SIZE		(4)

static void *key_pad_get_dt_data(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct key_pad_plat_data *pdata;
	const struct of_device_id *match;
	const __be32 *list;
	int value, i, size = 0;

	list = of_get_property(node, "buttons", &size);
	size /= LIST_ARRAY_SIZE;

	if (!size || size > KEY_MAX_BUTTONS)
		return NULL;

	match = of_match_node(key_dt_match, node);
	if (!match)
		return NULL;

	size /= 3;
	pdata = (struct key_pad_plat_data*)match->data;
	pdata->nbuttons = size;

	for (i = 0; size > i; i++) {
		pdata->buttons[i]     = be32_to_cpu(*list++);
		pdata->keycodes[i]    = be32_to_cpu(*list++);
		pdata->active_high[i] = be32_to_cpu(*list++);
		pr_debug("DTS [%d] key %3d, code %4d active %s\n",
			i, pdata->buttons[i], pdata->keycodes[i], pdata->active_high[i]
			? "High" : "Low");
	}

	if (!of_property_read_u32(node, "autorepeat", &value))
		pdata->autorepeat = value;

	if (!of_property_read_u32(node, "resume_delay_ms", &value))
		pdata->resume_delay_ms = value;

	return pdata;
}
#else
#define key_dt_match NULL
#endif

static struct key_pad_info *key_pad_setup(struct key_pad_plat_data *pdata,
						struct input_dev *input)
{

	struct key_pad_info *info = NULL;
	const char *name = DEV_NAME_KEYPAD;
	int key_num = pdata->nbuttons;
	int i = 0, ret = 0;

	info = kzalloc((sizeof(struct key_pad_info) +
				(sizeof(struct key_pad_code)*key_num)), GFP_KERNEL);
	if (!info) {
		pr_err("fail, %s allocate driver info ...\n", name);
		return NULL;
	}

	info->input = input;
	info->keys = (struct key_pad_code*)((long)info + sizeof(struct key_pad_info));
	info->key_num = pdata->nbuttons;
	info->resume_delay = pdata->resume_delay_ms;

	INIT_DELAYED_WORK(&info->resume_work, key_pad_resume_work);

	for (i = 0; info->key_num > i; i++) {
		info->keys[i].pin = pdata->buttons[i];
		info->keys[i].code = pdata->keycodes[i];
		info->keys[i].active_high = pdata->active_high ? pdata->active_high[i] : 0;
		info->keys[i].info = info;
		info->keys[i].press = KEY_IO_RELEASE;

	    info->keys[i].wq = create_singlethread_workqueue(name);
		if (!info->keys[i].wq)
    	   goto err_irq;

		INIT_DELAYED_WORK(&info->keys[i].work, key_pad_work);

		ret = request_irq(gpio_to_irq(info->keys[i].pin),
					key_pad_interrupt, (IRQF_SHARED | IRQ_TYPE_EDGE_BOTH),
					name, &info->keys[i]);
		if (ret) {
			pr_err("fail, gpio[%d] %s request irq...\n", info->keys[i].pin, name);
			goto err_irq;
		}
		__set_bit(info->keys[i].code, input->keybit);

		printk("KEY [%d] key %3d, code %4d irq %d active %s\n",
			i, info->keys[i].pin, info->keys[i].code, gpio_to_irq(info->keys[i].pin),
			info->keys[i].active_high ? "High" : "Low");
	}

	return info;

err_irq:
	for (--i; i >= 0; i--) {
		cancel_delayed_work_sync(&info->keys[i].work);
	    destroy_workqueue(info->keys[i].wq);
		free_irq(gpio_to_irq(info->keys[i].pin), &info->keys[i]);
	}

	if (info)
		kfree(info);

	return NULL;
}

static int key_pad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct key_pad_plat_data *pdata = dev_get_platdata(dev);
	struct key_pad_info *info = NULL;
	struct input_dev *input = NULL;
	int ret = 0;

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		pdata = key_pad_get_dt_data(pdev);
		if (!pdata)
			return -EINVAL;
	}
#endif

	input = input_allocate_device();
	if (!input) {
		pr_err("fail, %s allocate input device\n", pdev->name);
		return -ENOMEM;
	}

	info = key_pad_setup(pdata, input);
	if (!info) {
		pr_err("fail, %s allocate driver info ...\n", pdev->name);
		goto err_out;
	}
	platform_set_drvdata(pdev, info);

	input->name	= "Nexell Keypad";
	input->phys = "nexell/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0002;
	input->id.version = 0x0100;
	input->dev.parent = &pdev->dev;
	input->keycode = pdata->keycodes;
	input->keycodesize = sizeof(pdata->keycodes[0]);
	input->keycodemax = pdata->nbuttons * 2;	// for long key
	input->evbit[0] = BIT_MASK(EV_KEY);

	if (pdata->autorepeat)
		 input->evbit[0] |= BIT_MASK(EV_REP);

	input_set_capability(input, EV_MSC, MSC_SCAN);
	input_set_drvdata(input, info);

	ret = input_register_device(input);
	if (ret) {
		pr_err("fail, %s register for input device ...\n", pdev->name);
		goto err_out;
	}

	return ret;

err_out:
	if (input)
		input_free_device(input);

	if (info)
		kfree(info);

	return ret;
}

static int key_pad_remove(struct platform_device *pdev)
{
	struct key_pad_info *info = platform_get_drvdata(pdev);
	int i = 0;

	input_free_device(info->input);

	for (i = 0; i < info->key_num; i++) {
		cancel_delayed_work_sync(&info->keys[i].work);
	    destroy_workqueue(info->keys[i].wq);
		free_irq(gpio_to_irq(info->keys[i].pin), &info->keys[i]);
	}

	if (info)
		kfree(info);

	return 0;
}

static struct platform_driver key_plat_driver = {
	.probe		= key_pad_probe,
	.remove		= key_pad_remove,
	.suspend	= key_pad_suspend,
	.resume		= key_pad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DEV_NAME_KEYPAD,
		.of_match_table	= of_match_ptr(key_dt_match),
	},
};

static int __init key_pad_init(void)
{
	return platform_driver_register(&key_plat_driver);
}

static void __exit key_pad_exit(void)
{
	platform_driver_unregister(&key_plat_driver);
}

module_init(key_pad_init);
module_exit(key_pad_exit);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Keypad driver for the Nexell board");
MODULE_LICENSE("GPL");


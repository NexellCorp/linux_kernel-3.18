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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hwmon.h>
#include <linux/hwmon-vid.h>
#include <linux/hwmon-sysfs.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

#include <nexell/platform.h>
#include <nexell/thermal.h>
#include <nexell/soc-s5pxx18.h>

/*
#define	pr_debug	printk
*/

#define DRVNAME	"nxp-tmu-hwmon"

#define	THERMAL_EFUSE_TRIM		1
#define	THERMAL_UP_OFFS			2
#define THERMAL_POLL_TIME		100		/* ms */
#define THERMAL_EVENT_MAX		3
#define THERMAL_FREQ_STEP		100000	/* khz */
#define	DELAY(x)				msecs_to_jiffies(x)

/*
 * if efuse is zero, assume limit 85 ~= 115 : C = R - efuse + 25
 */
#define	EFUSE_ZERO_TRIM			55

struct nxp_thermal_event {
	int   type;		/* passive=1, hot=2, critical=3 */
	int	  temp;
	long  period;	/* ms */
	long  frequency;
	unsigned long expire;
	unsigned long enabled;
	int poweroff;

};

struct nxp_thermal_dev {
	struct device *hwmon_dev;
	int channel;
	int irqno;
	struct nxp_thermal_event *events;
	int event_size;
	int poll_period;
	int temp_label;
	int temp_max;
	long max_freq;
	long min_freq;
	long new_freq;
	struct notifier_block nb;
	struct cpumask allowed_cpus;
	/* TMU HW */
	int tmu_trimv;
	int tmu_trimv85;
	u32 temp_rise, temp_inten;
	/* TMU func */
	int step_up;
	struct delayed_work dn_work, up_work;
	unsigned long state;
};

#define	TMU_STAT_STOP		(1<<0)		/* bit position */
#define	TMU_STAT_DOWN		(1<<1)		/* bit position */
#define TMU_EVENT_BIT		(1<<0)

static DEFINE_MUTEX(thermal_mlock);
/*
 * cpu frequency
 */
static int thermal_cpufreq_notifier(struct notifier_block *nb,
						unsigned long event, void *data)
{
	struct nxp_thermal_dev *thermal =
			container_of(nb, struct nxp_thermal_dev, nb);
	struct cpufreq_policy *policy = data;
	unsigned long max_freq = 0;

	if (event != CPUFREQ_ADJUST || 0 == thermal->event_size)
		return 0;

	if (cpumask_test_cpu(policy->cpu, &thermal->allowed_cpus))
		max_freq = thermal->new_freq;

	/* Never exceed user_policy.max */
	if (max_freq > policy->user_policy.max)
		max_freq = policy->user_policy.max;

	pr_debug("Thermal.%d notify max = %lu (%ld)kHz (EVENT:%lu)\n",
		thermal->channel, max_freq, thermal->new_freq, event);

	if (policy->max != max_freq)
		cpufreq_verify_within_limits(policy, 0, max_freq);

	return 0;
}

static void thermal_cpufreq_register(struct nxp_thermal_dev *thermal)
{
	struct notifier_block *nb;
	struct cpumask mask_val;

	if (0 == thermal->event_size)
		return;

	cpumask_set_cpu(0, &mask_val);
	cpumask_copy(&thermal->allowed_cpus, &mask_val);

	nb = &thermal->nb;
	nb->notifier_call = &thermal_cpufreq_notifier;

	cpufreq_register_notifier(nb, CPUFREQ_POLICY_NOTIFIER);
}

static inline long cpufreq_get_max(struct nxp_thermal_dev *thermal)
{
	unsigned int cpuid = 0;
	return cpufreq_quick_get_max(cpuid);
}

static inline void cpufreq_set_max(struct nxp_thermal_dev *thermal, long new)
{
	unsigned int cpuid = 0;
	thermal->new_freq = new;
	cpufreq_update_policy(cpuid);
}

/*
 * TMU operation
 */
#define	TIME_100US	0x6B3	// 0x4305
#define	TIME_20us 	0x16A	// 0xE29
#define	TIME_2us 	0x024	// 0x170

static int nxp_thermal_dev_start(struct nxp_thermal_dev *thermal)
{
	int channel = thermal->channel;
	u32 mode = 7;
	int time = 1000;

	NX_TMU_SetBaseAddress(channel,
			(void*)IO_ADDRESS(NX_TMU_GetPhysicalAddress(channel)));
	NX_TMU_ClearInterruptPendingAll(channel);
	NX_TMU_SetInterruptEnableAll(channel, CFALSE);

	// Set CounterValue0, CounterValue1
	NX_TMU_SetCounterValue0(channel, ((TIME_20us<<16) | TIME_100US));
	NX_TMU_SetCounterValue1(channel, ((TIME_100US<<16) | TIME_2us));
	NX_TMU_SetSamplingInterval(channel, 0x1);

	// Emulstion mode enable
	NX_TMU_SetTmuEmulEn(channel, CFALSE);
	NX_TMU_SetP0IntEn(channel, 0x0);
	NX_TMU_SetTmuTripMode(channel, mode);
	NX_TMU_SetTmuTripEn(channel, 0x0);

	// Check sensing operation is idle
	while (time-- > 0 && NX_TMU_IsBusy(channel)) { msleep(1); }

	NX_TMU_SetTmuStart(channel, CTRUE);
	return 0;
}

static void nxp_thermal_dev_stop(struct nxp_thermal_dev *thermal)
{
	int channel = thermal->channel;
	NX_TMU_SetTmuStart(channel, CFALSE);
}

static void nxp_thermal_dev_trim(struct nxp_thermal_dev *thermal)
{
	int channel = thermal->channel;
	int trimv = 0, trimv85 = 0;
	u32 ret = 0;

#if THERMAL_EFUSE_TRIM
	int count = 10;

	while (count-- > 0) {
	    // Program the measured data to e-fuse
    	u32 val = readl((void*)IO_ADDRESS(PHY_BASEADDR_TIEOFF_MODULE + (76*4)));
    	val = val | 0x3;;
    	writel(val, (void*)IO_ADDRESS(PHY_BASEADDR_TIEOFF_MODULE + (76*4)));

    	// e-fuse Sensing Done. Check.
    	val = readl((void*)IO_ADDRESS(PHY_BASEADDR_TIEOFF_MODULE + (76*4)));
    	ret = (((val>>3) & 0x3) == 0x3);
    	if (ret)
    		break;
    	mdelay(1);
    }

	trimv = NX_TMU_GetTriminfo25(channel);
	trimv85 = NX_TMU_GetTriminfo85(channel);
#endif

	if (0 == trimv || !ret)
		trimv = EFUSE_ZERO_TRIM;

	thermal->tmu_trimv = trimv;
	thermal->tmu_trimv85 = trimv85;

	printk("Thermal.%d = trim %d:%d, %s\n",
		channel, trimv, trimv85, ret?"exist":"empty");
}

static inline int is_triggered(int channel, int bit)
{
	return ((1<<(bit*4)) & NX_TMU_GetP0IntStat(channel)) ? 1 : 0;
}

static inline void disable_trigger(int channel, int bit)
{
	u32 mask = NX_TMU_GetP0IntEn(channel);
	NX_TMU_SetP0IntClear(channel, 1<<(bit*4));
	NX_TMU_SetP0IntEn(channel, mask & ~(1<<(bit*4)));
}

static inline void enable_trigger(int channel, int bit)
{
	NX_TMU_SetP0IntClear(channel, 1<<(bit*4));
	NX_TMU_SetP0IntEn(channel, NX_TMU_GetP0IntEn(channel) | 1<<(bit*4));
}

static inline int nxp_thermal_dev_temp(struct nxp_thermal_dev *thermal)
{
	int channel = thermal->channel;
	int val = NX_TMU_GetCurrentTemp0(channel);	// can't use temp1
	int ret = (val - thermal->tmu_trimv + 25);
/*
	pr_debug("Thermal.%d = %d (%d - %d + 25)\n",
			channel, ret, val, thermal->tmu_trimv);
*/
	return ret;
}

#define __SWAP__(x, y, t) ((t) = (x), (x) = (y), (y) = (t))
static void thermal_sort(u32 *v, int size)
{
	u32 t = 0;
	int i = 0;

	if (0 > size)
		return;

	for (i = 0; (size-1) > i; i++) {
		if (v[i] > v[i+1]) {
			__SWAP__(v[i], v[i+1], t);
			thermal_sort(&v[i+1], size-(i+1));
		}
	}
	thermal_sort(v, (size-1));
};

static void nxp_thermal_boot_temp(struct nxp_thermal_dev *thermal)
{
	struct nxp_thermal_event *events = thermal->events;
	int i = 0;

	for (i = 0; thermal->event_size > i; i++)
		set_bit(TMU_EVENT_BIT, &events[i].enabled);

	schedule_delayed_work(&thermal->dn_work, DELAY(thermal->poll_period));
}

static irqreturn_t nxp_thermal_interrupt(int irq, void *desc)
{
	struct nxp_thermal_dev *thermal = desc;
	struct nxp_thermal_event *event = thermal->events;
	int channel = thermal->channel;
	int i = 0;

	pr_debug("Thermal.%d irq %d C stat 0x%x\n\n",
		channel, nxp_thermal_dev_temp(thermal), NX_TMU_GetP0IntStat(channel));

	/* disable triggered irq */
	for (i = 0; THERMAL_EVENT_MAX > i; i++) {
		if (is_triggered(channel, i)) {
			disable_trigger(channel, i);
			event[i].expire = ktime_to_ms(ktime_get()) + event[i].period;
			set_bit(TMU_EVENT_BIT, &event[i].enabled);
		}
	}
	schedule_work(&thermal->dn_work.work);

	return IRQ_HANDLED;
}

static void nxp_thermal_step_up(struct work_struct *work)
{
	struct nxp_thermal_dev *thermal =
			container_of(work, struct nxp_thermal_dev, up_work.work);
	long curr, next;

	mutex_lock(&thermal_mlock);

	if (test_bit(TMU_STAT_DOWN, &thermal->state) ||
		test_bit(TMU_STAT_STOP, &thermal->state))
		goto end_up;

	curr = cpufreq_get_max(thermal);
	next = curr + THERMAL_FREQ_STEP;

	if (next > thermal->max_freq)
		goto end_up;

	pr_debug("Thermal.%d upper  %6ld ...\n\n", thermal->channel, next);

	cpufreq_set_max(thermal, next);
	schedule_delayed_work(&thermal->up_work, DELAY(thermal->poll_period));

end_up:
	mutex_unlock(&thermal_mlock);
	return;
}

static void nxp_thermal_step_down(struct work_struct *work)
{
	struct nxp_thermal_dev *thermal =
			container_of(work, struct nxp_thermal_dev, dn_work.work);
	struct nxp_thermal_event *event = thermal->events;
	int event_size = thermal->event_size;
	int channel = thermal->channel;
	long curr, next;
	unsigned long time = 0;
	int up_work = 0, down_work = 0;
	int step_down = 0;
	int temp, i = 0;

	mutex_lock(&thermal_mlock);
	if (test_bit(TMU_STAT_STOP, &thermal->state)) {
		mutex_unlock(&thermal_mlock);
		return;
	}

	temp = nxp_thermal_dev_temp(thermal);
	time = ktime_to_ms(ktime_get());
	thermal->temp_label = temp;

	curr = cpufreq_get_max(thermal);
	next = curr;

	if (thermal->min_freq > next)
		next = thermal->min_freq;

	if (temp > thermal->temp_max)
		thermal->temp_max = temp;

	for (i = 0; event_size > i; i++, event++) {

		if (!test_bit(TMU_EVENT_BIT, &event->enabled))
			continue;
		/*
		pr_debug("Thermal.%d event[%d] %3d C, %3d\n",
				channel, i, event->temp, temp);
		*/
		if ((event->temp - THERMAL_UP_OFFS) > temp) {
			/*
			 * up
			 */
			if (event->frequency > next)
				next = event->frequency;
			event->expire = 0;
			clear_bit(TMU_EVENT_BIT, &event->enabled);
			enable_trigger(channel, i);
			up_work = 1;
		} else {
			/*
			 * down
			 */
			down_work = 1, up_work = 0;
			if (ktime_to_ms(ktime_get()) >= event->expire) {
				next = curr - THERMAL_FREQ_STEP;
				step_down = 1;

				if (next > event->frequency)
					next = event->frequency;

				if (event->poweroff) {
					printk("Thermal.%d critical temperature reached (%d C)\n",
						channel, event->temp);
					printk("shutting down ...\n");
					orderly_poweroff(true);
				}
			}
		}
	}

	if (down_work) {
		set_bit(TMU_STAT_DOWN, &thermal->state);
		if (step_down && (curr > thermal->min_freq)) {
			pr_debug("Thermal.%d step down %6ld\n\n", channel, next);
			cpufreq_set_max(thermal, next);
		}

		set_bit(TMU_STAT_DOWN, &thermal->state);
		schedule_delayed_work(&thermal->dn_work, DELAY(thermal->poll_period));

	} else
	if (up_work) {
		clear_bit(TMU_STAT_DOWN, &thermal->state);
		if (thermal->max_freq > next) {
			pr_debug("Thermal.%d step up  %6ld ...\n\n",
				channel, thermal->step_up?next:thermal->max_freq);
			if (!thermal->step_up) {
				cpufreq_set_max(thermal, thermal->max_freq);
			} else {
				cpufreq_set_max(thermal, next);
				schedule_delayed_work(&thermal->up_work,
					DELAY(thermal->poll_period));
			}
		}
	}

	mutex_unlock(&thermal_mlock);

	return;
}

static int nxp_thermal_setup(struct nxp_theramal_plat_data *pdata,
						struct nxp_thermal_dev *thermal)
{
	struct nxp_thermal_trigger *triggers = pdata->triggers;
	struct nxp_thermal_event *events = NULL;
	int channel = thermal->channel;
	int event_size = thermal->event_size;
	u32 trim_rise = 0, temp_array[THERMAL_EVENT_MAX] = { 0, };
	int err = 0, i = 0;

	/*
	 * Thermal start
	 */
	nxp_thermal_dev_start(thermal);
	nxp_thermal_dev_trim(thermal);

	if (!triggers || !event_size)
		return 0;

	if (event_size > 3) {
		printk("tmu.%d max trigger count %d\n", channel, THERMAL_EVENT_MAX);
		return -EINVAL;
	}

	/*
	 * Thermal events
	 */
	events = kzalloc(sizeof(*events)*event_size, GFP_KERNEL);
	if (NULL == events) {
		pr_err("%s: Out of memory\n", __func__);
		return -ENOMEM;
	}
	thermal->events = events;

	if (0 > thermal->irqno)
		return -EINVAL;

	err = request_irq(thermal->irqno, &nxp_thermal_interrupt,
				IRQF_DISABLED, DRVNAME, thermal);
	if (err) {
		pr_err("Fail, tmu.%d request interrupt %d ...\n", channel, IRQ_TMU0);
		return -EINVAL;
	}

	for (i = 0; event_size > i; i++) {
		if (i && pdata->triggers[i-1].trig_temp > pdata->triggers[i].trig_temp) {
			pr_warn("Thermal.%d invalid, trigger temp %d",
				channel, pdata->triggers[i].trig_temp);
			pr_warn("is less than previous trigger temp %d\n",
				pdata->triggers[i-1].trig_temp);
			continue;
		}

		events[i].temp = pdata->triggers[i].trig_temp & 0xFF;
		events[i].period = pdata->triggers[i].trig_duration;
		events[i].frequency = pdata->triggers[i].trig_frequency;
		events[i].expire = 0;
		events[i].poweroff = !events[i].frequency ? 1 : 0;
		temp_array[i] = events[i].temp;

		pr_debug("Thermal.%d [%d] irq.%2d %3d C (%ldms) -> %8ld kzh\n",
			channel, i, thermal->irqno, events[i].temp, events[i].period,
			events[i].frequency);
	}

	thermal_sort(temp_array, event_size);
	for (i = 0; event_size > i; i++) {
		trim_rise = temp_array[i] - 25 + thermal->tmu_trimv;
		thermal->temp_rise  |= trim_rise << (i*8);
		thermal->temp_inten |= 1<<(i*4);
	}

	NX_TMU_SetThresholdTempRise(channel, thermal->temp_rise);
	NX_TMU_ClearInterruptPendingAll(channel);
	NX_TMU_SetP0IntEn(channel, thermal->temp_inten);

	pr_debug("Thermal.%d rise 0x%08x, int 0x%08x\n",
		channel, thermal->temp_rise, thermal->temp_inten);

	/* check for boot temp */
	nxp_thermal_boot_temp(thermal);

	return 0;
}

#ifdef CONFIG_PM
static int nxp_thermal_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct nxp_thermal_dev *thermal = platform_get_drvdata(pdev);
	int channel = thermal->channel;

	mutex_lock(&thermal_mlock);
	set_bit(TMU_STAT_STOP, &thermal->state);

	nxp_thermal_dev_stop(thermal);
	NX_TMU_ClearInterruptPendingAll(channel);
	NX_TMU_SetP0IntEn(channel, 0x0);

	cpufreq_set_max(thermal, thermal->max_freq);

	mutex_unlock(&thermal_mlock);
	return 0;
}

static int nxp_thermal_resume(struct platform_device *pdev)
{
	struct nxp_thermal_dev *thermal = platform_get_drvdata(pdev);
	int channel = thermal->channel;

	mutex_lock(&thermal_mlock);
	clear_bit(TMU_STAT_STOP, &thermal->state);

	nxp_thermal_dev_start(thermal);

	if (thermal->events) {
		NX_TMU_SetThresholdTempRise(channel, thermal->temp_rise);
		NX_TMU_ClearInterruptPendingAll(channel);
		NX_TMU_SetP0IntEn(channel, thermal->temp_inten);

		/* check for resume temp */
		nxp_thermal_boot_temp(thermal);
	}

	mutex_unlock(&thermal_mlock);
	return 0;
}

#else
#define nxp_thermal_suspend 	NULL
#define nxp_thermal_resume 		NULL
#endif

/*
 * Sysfs
 */
enum { SHOW_TEMP, SHOW_LABEL, SHOW_NAME };

static ssize_t thermal_show_temp(struct device *dev,
			 			struct device_attribute *devattr, char *buf)
{
	struct nxp_thermal_dev *thermal = dev_get_drvdata(dev);
	char *s = buf;

	mutex_lock(&thermal_mlock);
	s += sprintf(s, "%4d\n", nxp_thermal_dev_temp(thermal));
	mutex_unlock(&thermal_mlock);

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t thermal_show_max(struct device *dev,
			 			struct device_attribute *devattr, char *buf)
{
	struct nxp_thermal_dev *thermal = dev_get_drvdata(dev);
	char *s = buf;
	int temp;

	mutex_lock(&thermal_mlock);

	temp = nxp_thermal_dev_temp(thermal);

	if (temp > thermal->temp_max)
		thermal->temp_max = temp;
	else
		temp = thermal->temp_max;

	s += sprintf(s, "%4d\n", temp);

	mutex_unlock(&thermal_mlock);

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t thermal_show_trim(struct device *dev,
			 			struct device_attribute *devattr, char *buf)
{
	struct nxp_thermal_dev *thermal = dev_get_drvdata(dev);
	char *s = buf;

	mutex_lock(&thermal_mlock);
	s += sprintf(s, "%d:%d\n", thermal->tmu_trimv, thermal->tmu_trimv85);
	mutex_unlock(&thermal_mlock);

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static SENSOR_DEVICE_ATTR(temp_label, S_IRUGO, thermal_show_temp , NULL, SHOW_LABEL);
static SENSOR_DEVICE_ATTR(temp_max  , S_IRUGO, thermal_show_max  , NULL, SHOW_LABEL);
static SENSOR_DEVICE_ATTR(temp_trim , S_IRUGO, thermal_show_trim , NULL, SHOW_LABEL);

static struct attribute *temp_attr[] = {
	&sensor_dev_attr_temp_label.dev_attr.attr,
	&sensor_dev_attr_temp_max.dev_attr.attr,
	&sensor_dev_attr_temp_trim.dev_attr.attr,
	NULL
};

static const struct attribute_group thermal_attr_group = {
	.attrs = temp_attr,
};

#ifdef CONFIG_OF
static struct nxp_theramal_plat_data nxp_thermal_dt_data;

static const struct of_device_id nxp_thermal_dt_match[] = {
	{
	.compatible = "nexell,nxp-tmu-hwmon",
	.data = (void*)&nxp_thermal_dt_data,
	}, {}
};
MODULE_DEVICE_TABLE(of, nxp_thermal_dt_match);
#define	LIST_ARRAY_SIZE		(4)
#define	LIST_ELEMENT_SIZE	(3)	/* temp, cpufreq, time */

static void *nxp_thermal_get_dt_data(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	struct nxp_theramal_plat_data *pdata;
	const __be32 *list;
	int i = 0, value, size = 0;

	match = of_match_node(nxp_thermal_dt_match, node);
	if (!match)
		return NULL;

	pdata = (struct nxp_theramal_plat_data*)match->data;

	memset(pdata, 0, sizeof(struct nxp_theramal_plat_data));
	pdata->poll_duration = THERMAL_POLL_TIME;

	if (!of_property_read_u32(node, "channel", &value))
		pdata->channel = value;

	if (!of_property_read_u32(node, "polling-delay", &value))
		pdata->poll_duration = value;

	if (!of_property_read_u32(node, "trigger-step-up", &value))
		pdata->trig_step_up = value;

	list = of_get_property(node, "triggers", &size);
	size /= (LIST_ARRAY_SIZE * LIST_ELEMENT_SIZE);
	pdata->trigger_size = size;

	pr_debug("Thermal.%d parse DTS triggers %d ...\n",
		pdata->channel, size);
	for (i = 0; size > i; i++) {
		pdata->triggers[i].trig_type = 0;
		pdata->triggers[i].trig_temp = be32_to_cpu(*list++);
		pdata->triggers[i].trig_frequency = be32_to_cpu(*list++);
		pdata->triggers[i].trig_duration = be32_to_cpu(*list++);
		pr_debug("Thermal.%d event[%d] %3d C, time %6ld, freq %6ld\n",
			pdata->channel, i, pdata->triggers[i].trig_temp,
			pdata->triggers[i].trig_duration,
			pdata->triggers[i].trig_frequency);
	}

	return (void*)pdata;
}
#else
#define nxp_thermal_dt_match NULL
#endif

static int nxp_thermal_probe(struct platform_device *pdev)
{
	struct nxp_theramal_plat_data *pdata = pdev->dev.platform_data;
	struct nxp_thermal_dev *thermal = NULL;
	int err = -1;
	struct cpufreq_policy policy = { .cpuinfo = { .min_freq = 0, .min_freq = 0 }, };

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		pdata = nxp_thermal_get_dt_data(pdev);
		if (!pdata)
			return -EINVAL;
	}
#endif

	thermal = kzalloc(sizeof(struct nxp_thermal_dev), GFP_KERNEL);
	if (!thermal) {
		pr_err("Failed tmu-hwmon Out of memory ...\n");
		return -ENOMEM;
	}

	cpufreq_get_policy(&policy, 0);

	thermal->channel =	pdata->channel;
	thermal->channel =	pdata->channel;
	thermal->poll_period = pdata->poll_duration ?
			pdata->poll_duration : THERMAL_POLL_TIME;
	thermal->min_freq = policy.cpuinfo.min_freq;
	thermal->max_freq = policy.cpuinfo.max_freq;
	thermal->new_freq = thermal->max_freq;
	thermal->event_size = pdata->trigger_size;
	thermal->step_up = pdata->trig_step_up;
	thermal->irqno = platform_get_irq(pdev, 0);

	clear_bit(TMU_STAT_STOP, &thermal->state);

	INIT_DELAYED_WORK(&thermal->dn_work, nxp_thermal_step_down);
	INIT_DELAYED_WORK(&thermal->up_work, nxp_thermal_step_up);

	thermal_cpufreq_register(thermal);

	/* setup thermal unit */
	err = nxp_thermal_setup(pdata, thermal);
	if (0 > err)
		goto exit_free;

	platform_set_drvdata(pdev, thermal);

	/* thermal sys interface */
	err = sysfs_create_group(&pdev->dev.kobj, &thermal_attr_group);
	if (err)
		goto exit_free;

	/* add thermal to hwmon */
	thermal->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(thermal->hwmon_dev)) {
		err = PTR_ERR(thermal->hwmon_dev);
		pr_err("%s: Class registration failed (%d)\n", __func__, err);
		goto exit_remove;
	}

	printk("TMU.%d: register to hwmon (%ld ~ %ldkhz)\n",
		thermal->channel, thermal->max_freq, thermal->min_freq);
	return 0;

exit_remove:
	sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);

exit_free:
	platform_set_drvdata(pdev, NULL);
	if (thermal->events)
		kfree(thermal->events);

	kfree(thermal);

	return err;
}

static int nxp_thermal_remove(struct platform_device *pdev)
{
	struct nxp_thermal_dev *thermal = platform_get_drvdata(pdev);
	struct nxp_thermal_event *events = thermal->events;

	set_bit(TMU_STAT_STOP, &thermal->state);

	nxp_thermal_dev_stop(thermal);

	cancel_delayed_work(&thermal->dn_work);
	cancel_delayed_work(&thermal->up_work);

	hwmon_device_unregister(thermal->hwmon_dev);
	cpufreq_unregister_notifier(&thermal->nb, CPUFREQ_POLICY_NOTIFIER);
	sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);

	platform_set_drvdata(pdev, NULL);

	if (thermal)
		kfree(thermal);

	if (events)
		kfree(events);

	return 0;
}

static struct platform_driver nxp_thermal_driver = {
	.driver = {
		.name   = DRVNAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nxp_thermal_dt_match),
	},
	.probe = nxp_thermal_probe,
	.remove	= nxp_thermal_remove,
	.suspend = nxp_thermal_suspend,
	.resume = nxp_thermal_resume,
};

static int __init nxp_thermal_init(void)
{
    return platform_driver_register(&nxp_thermal_driver);
}
late_initcall(nxp_thermal_init);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("SLsiAP temperature monitor");
MODULE_LICENSE("GPL");

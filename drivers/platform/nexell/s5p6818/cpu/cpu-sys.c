#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>

#include <nexell/platform.h>

/*
#define	pr_debug	printk
*/

static inline int wait_key_ready(void)
{
	while (!NX_ECID_GetKeyReady()) {
		if (time_after(jiffies, jiffies + 1)) {
			if (NX_ECID_GetKeyReady())
				break;
			pr_err("Error: id not key ready \n");
			return -EINVAL;
		}
		cpu_relax();
	}
	return 0;
}

int nxp_cpu_id_guid(u32 guid[4])
{
	if (0 > wait_key_ready())
		return -EINVAL;
	NX_ECID_GetGUID((NX_GUID*)guid);
	return 0;
}

int nxp_cpu_id_ecid(u32 ecid[4])
{
	if (0 > wait_key_ready())
		return -EINVAL;
	NX_ECID_GetECID(ecid);
	return 0;
}

int nxp_cpu_id_string(u32 *string)
{
	if (0 > wait_key_ready())
		return -EINVAL;
	NX_ECID_GetChipName((char*)string);
	return 0;
}

/* Notify cpu GUID: /sys/devices/platform/cpu,  guid, uuid,  name  */
static ssize_t sys_id_show(struct device *pdev,
					struct device_attribute *attr, char *buf)
{
	struct attribute *at = &attr->attr;
	char *s = buf;
	u32 uid[4] = {0, };
	u8  name[12*4] = {0,};
	int string = -EINVAL;

	pr_debug("[%s : name =%s ]\n", __func__, at->name);

	if (!strcmp(at->name, "uuid"))
		nxp_cpu_id_ecid(uid);
	else
	if (!strcmp(at->name, "guid"))
		nxp_cpu_id_guid(uid);
	else
	if (!strcmp(at->name, "name"))
		string = nxp_cpu_id_string((u32*)name);
	else
		return -EINVAL;

	if (!string) {
		if (isprint(name[0])) {
			s += sprintf(s, "%s\n", name);
		} else {
			#define _W	(12)		// width
			int i;
			for (i = 0; i < sizeof(name); i++) {
				s += sprintf(s, "%02x", name[i]);
				if ((i+1) % _W == 0)
					s += sprintf(s, " ");
			}
			s += sprintf(s, "\n");
		}
	} else {
		s += sprintf(s, "%08x:%08x:%08x:%08x\n", uid[0], uid[1], uid[2], uid[3]);
	}

	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

#define	ATTR_MODE	0644
static struct device_attribute __guid__ = __ATTR(guid, ATTR_MODE, sys_id_show, NULL);
static struct device_attribute __uuid__ = __ATTR(uuid, ATTR_MODE, sys_id_show, NULL);
static struct device_attribute __name__ = __ATTR(name, ATTR_MODE, sys_id_show, NULL);

static struct attribute *sys_attrs[] = {
	&__guid__.attr,
	&__uuid__.attr,
	&__name__.attr,
	NULL,
};

static struct attribute_group sys_attr_group = {
	.attrs = (struct attribute **)sys_attrs,
};

static int __init cpu_sys_id_setup(void)
{
	struct kobject *kobj;
	u32 uid[4] = {0, };
	int ret = 0;

	kobj = kobject_create_and_add("cpu", &platform_bus.kobj);
	if (!kobj) {
		pr_err("Failed create cpu kernel object ....\n");
		return -ret;
	}

	ret = sysfs_create_group(kobj, &sys_attr_group);
	if (ret) {
		pr_err("Failed create cpu sysfs group ...\n");
		kobject_del(kobj);
		return -ret;
	}

	if (0 > nxp_cpu_id_ecid(uid))
		printk("FAIL: ecid !!!\n");

	printk("ECID: %08x:%08x:%08x:%08x\n", uid[0], uid[1], uid[2], uid[3]);
	return ret;
}

/*
 *  "sys/devices/platform/pll.N"
 */
static ssize_t clk_pll_show(struct device *pdev,
					struct device_attribute *attr, char *buf)
{
	struct attribute *at = &attr->attr;
	struct clk *clk;
	const char *c;
	char *s = buf, name[16] = { 0, };
	long rate;
	int pll;

	c = &at->name[strlen("pll.")];
	pll = simple_strtoul(c, NULL, 10);
	sprintf(name, "pll%d", pll);

	clk = clk_get(NULL, name);
	if (IS_ERR(clk)) {
		printk("Fail, not support pll.%d (0~3)\n", pll);
		return 0;
	}
	rate = clk_get_rate(clk);
	clk_put(clk);

	s += sprintf(s, "%ld\n", rate/1000);	/* khz */

	return (s - buf);
}

static ssize_t clk_pll_store(struct device *pdev,
					struct device_attribute *attr, const char *buf, size_t n)
{
	struct attribute *at = &attr->attr;
	struct clk *clk;
	char name[16] = { 0, };
	const char *c;
	unsigned long freq;
	int pll;

	c = &at->name[strlen("pll.")];
	pll = simple_strtoul(c, NULL, 10);
	sprintf(name, "pll%d", pll);

	clk = clk_get(NULL, name);
	if (IS_ERR(clk)) {
		printk("Fail, not support pll.%d (0~3)\n", pll);
		return 0;
	}
	sscanf(buf,"%lu", &freq);

#if defined (CONFIG_ARM_NXP_CPUFREQ)
	clk_set_rate(clk, freq*1000);
#endif
	clk_put(clk);

	return n;
}

static struct device_attribute clk_pll_attr[] = {
	__ATTR(pll.0, 0664, clk_pll_show, clk_pll_store),
	__ATTR(pll.1, 0664, clk_pll_show, clk_pll_store),
	__ATTR(pll.2, 0664, clk_pll_show, clk_pll_store),
	__ATTR(pll.3, 0664, clk_pll_show, clk_pll_store),
};

static struct attribute *pll_attrs[] = {
	&clk_pll_attr[0].attr,
	&clk_pll_attr[1].attr,
	&clk_pll_attr[2].attr,
	&clk_pll_attr[3].attr,
	NULL,
};

static struct attribute_group pll_attr_group = {
	.attrs = (struct attribute **)pll_attrs,
};

static int __init cpu_sys_clk_setup(void)
{
	struct kobject *kobj;
	int ret = 0;

	kobj = kobject_create_and_add("pll", &platform_bus.kobj);
	if (!kobj) {
		pr_err("Failed create pll kernel object ....\n");
		return -ret;
	}

	ret = sysfs_create_group(kobj, &pll_attr_group);
	if (ret) {
		pr_err("Failed create pll sysfs group ...\n");
		kobject_del(kobj);
		return -ret;
	}
	return ret;
}

static int __init cpu_sys_init_setup(void)
{
	cpu_sys_id_setup();
	cpu_sys_clk_setup();
	return 0;
}
core_initcall(cpu_sys_init_setup);

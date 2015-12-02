#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/pwm.h>
#include <linux/hardirq.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/clk.h>

#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

/*
#define	pr_debug	printk
*/

/*
 *	CLK In / TCNT = Frequency
 *	TCNT   / TCMP = Duty persent
 */
struct pwm_device_hw {
	int ch;
	int clk_tclk;		/* 0:pclk, 1:tclk */
	unsigned long request;
	unsigned long rate;
	int 		  duty;		/* unit % 0% ~ 100% */
	unsigned long pwm_hz;
	unsigned int  tmux;
	unsigned int  counter;
	unsigned int  compare;
	unsigned int  io;
	unsigned int  fn_io;	/* gpio function */
	unsigned int  fn_pwm;	/* pwm  function */
	struct clk *clk;
	struct mutex lock;
};

static struct pwm_device_hw devs_pwm[] = {
	[0] = {
		.ch	    = 0,
		.io     = PAD_GPIO_D +  1,
		.fn_io  = NX_GPIO_PADFUNC_0,
		.fn_pwm = NX_GPIO_PADFUNC_1,
#ifdef CFG_PWM0_CLK_SRC
		.clk_tclk= 1,
#endif
	},
	[1] = {
		.ch	    = 1,
		.io     = PAD_GPIO_C + 13,
		.fn_io  = NX_GPIO_PADFUNC_1,
		.fn_pwm = NX_GPIO_PADFUNC_2,
#ifdef CFG_PWM1_CLK_SRC
		.clk_tclk= 1,
#endif
	},
	[2] = {
		.ch	    = 2,
		.io     = PAD_GPIO_C + 14,
		.fn_io  = NX_GPIO_PADFUNC_1,
		.fn_pwm = NX_GPIO_PADFUNC_2,
#ifdef CFG_PWM2_CLK_SRC
		.clk_tclk= 1,
#endif
	},
	[3] = {
		.ch	    = 3,
		.io     = PAD_GPIO_D +  0,
		.fn_io  = NX_GPIO_PADFUNC_0,
		.fn_pwm = NX_GPIO_PADFUNC_2,
#ifdef CFG_PWM3_CLK_SRC
		.clk_tclk= 1,
#endif
	},
};
#define	PWN_CHANNELS	(4)

#define	_LOCK_(c)		{ if (!preempt_count() && !in_interrupt()) mutex_lock(&devs_pwm[c].lock); }
#define	_UNLOCK_(c)		{ if (mutex_is_locked(&devs_pwm[c].lock)) mutex_unlock(&devs_pwm[c].lock); }

#define RET_ASSERT_VAL(_expr_, _ret_)	{			\
	if (!(_expr_)) {								\
		printk(KERN_ERR "%s(%d) : %s %s \n",		\
			__func__, __LINE__, "ASSERT", #_expr_);	\
		if ( _ret_) return _ret_; else return 0;	\
	}											\
}

#define RET_ASSERT(_expr_)	{			\
	if (!(_expr_)) {								\
		printk(KERN_ERR "%s(%d) : %s %s \n",		\
			__func__, __LINE__, "ASSERT", #_expr_);	\
		return;										\
	}											\
}

#define	PWM_CFG0		(0x00)
#define	PWM_CFG1		(0x04)
#define	PWM_TCON		(0x08)
#define	PWM_CNTB		(0x0C)
#define	PWM_CMPB		(0x10)
#define	PWM_CNTO		(0x14)
#define	PWM_STAT		(0x44)

#define	TCON_AUTO		(1<<3)
#define	TCON_INVT		(1<<2)
#define	TCON_UP			(1<<1)
#define	TCON_RUN		(1<<0)
#define CFG0_CH(ch)		(ch == 0 || ch == 1 ? 0 : 8)
#define CFG1_CH(ch)		(ch * 4)
#define TCON_CH(ch)		(ch ? ch * 4  + 4 : 0)
#define TINT_CH(ch)		(ch)
#define TINT_CS_CH(ch)	(ch + 5)
#define	TINT_CS_MASK	(0x1F)
#define PWM_CH_OFFS	 	(0xC)

static unsigned long    clk_in_max = 0;
#define	DUTY_MIN_VAL	2

/*
 * PWM HW
 */
static void __iomem *base;   

#define	PWM_BASE	base
#define	PWM_RESET()	do {	\
		if (!nxp_soc_peri_reset_status(RESET_ID_PWM))	\
			nxp_soc_peri_reset_set(RESET_ID_PWM);	\
		} while (0)

static inline void pwm_clock(int ch, int mux, int scl)
{
	volatile U32 val;

	val  = readl(PWM_BASE + PWM_CFG0);
	val &= ~(0xFF   << CFG0_CH(ch));
	val |=  ((scl-1)<< CFG0_CH(ch));
	writel(val, PWM_BASE + PWM_CFG0);

	val  = readl(PWM_BASE + PWM_CFG1);
	val &= ~(0xF << CFG1_CH(ch));
	val |=  (mux << CFG1_CH(ch));
	writel(val, PWM_BASE + PWM_CFG1);
}

static inline void pwm_count(int ch, unsigned int cnt, unsigned int cmp)
{
	writel((cnt-1), PWM_BASE + PWM_CNTB + (PWM_CH_OFFS * ch));
	writel((cmp-1), PWM_BASE + PWM_CMPB + (PWM_CH_OFFS * ch));
}

static inline void pwm_start(int ch, int irqon)
{
	volatile U32 val;
	int on = irqon ? 1 : 0;

	val  = readl(PWM_BASE + PWM_STAT);
	val &= ~(TINT_CS_MASK<<5 | 0x1 << TINT_CH(ch));
	val |=  (0x1 << TINT_CS_CH(ch) | on << TINT_CH(ch));
	writel(val, PWM_BASE + PWM_STAT);

	val = readl(PWM_BASE + PWM_TCON);
	val &= ~(0xA << TCON_CH(ch));
	val |=  (TCON_UP << TCON_CH(ch));
	writel(val, PWM_BASE + PWM_TCON);

	val &= ~(TCON_UP << TCON_CH(ch));
	val |=  ((TCON_AUTO | TCON_RUN) << TCON_CH(ch));	/* set pwm out invert ? */
	writel(val, PWM_BASE + PWM_TCON);
}

static inline void pwm_stop(int ch, int irqon)
{
	volatile U32 val;
	int on = irqon ? 1 : 0;

	val  = readl(PWM_BASE + PWM_STAT);
	val &= ~(TINT_CS_MASK<<5 | 0x1 << TINT_CH(ch));
	val |=  (0x1 << TINT_CS_CH(ch) | on << TINT_CH(ch));
	writel(val, PWM_BASE + PWM_STAT);

	val  = readl(PWM_BASE + PWM_TCON);
	val &= ~(TCON_RUN << TCON_CH(ch));
	writel(val, PWM_BASE + PWM_TCON);
}

static void pwm_set_device(struct pwm_device_hw *pwm)
{
	int ch = pwm->ch;
	unsigned int tmux = pwm->tmux, tscl = 1;
	pr_debug("%s (ch:%d, rate:%ld, pwm_hz:%ld, count:%u, cmp:%u, tmux:%u)\n",
	 	__func__, ch, pwm->rate, pwm->pwm_hz, pwm->counter, pwm->compare, tmux);

	_LOCK_(ch);

	/* for wakeup */
	PWM_RESET();

	if (pwm->counter == pwm->compare || 0 == pwm->compare) {
		nxp_soc_gpio_set_out_value(pwm->io, (0 == pwm->compare ? 0 : 1));
		nxp_soc_gpio_set_io_dir(pwm->io, 1);
		nxp_soc_gpio_set_io_func(pwm->io, pwm->fn_io);

		pwm_stop(ch, 0);
		clk_disable_unprepare(pwm->clk);
	} else {
		if(pwm->clk_tclk)
			clk_set_rate(pwm->clk, pwm->rate);
		clk_prepare_enable(pwm->clk);

		pwm_clock(ch, tmux, tscl);
		pwm_count(ch, pwm->counter, pwm->compare);	/* TCMPB : Need Invert */
		pwm_start(ch, 0);

		nxp_soc_gpio_set_io_func(pwm->io, pwm->fn_pwm);
	}

	_UNLOCK_(ch);
}

/*------------------------------------------------------------------------------
 * 	Description	: set pwm frequncy, and save frequency info
 *	In[ch]		: pwm channel, 0 ~ 3
 *	In[freq]	: pwm frequency, unit (Hz), : 30Hz ~ 237KHz
 *	In[duty]	: pwm pulse width, unit (%), 0 % ~ 100 %
 *				: if 0, pulse is low out.
 *	Return 		: if return is less than 0, invalid request.
 *		 		: else pwm out frequency value.
 */
#define PWM_COMPARE(c, d) (((10 > c ? c * 10 : c) * d) / (100 * (10 > c ? 10 : 1)))

unsigned long nxp_soc_pwm_set_frequency(int ch, unsigned int request, unsigned int duty)
{
	struct pwm_device_hw *pwm = &devs_pwm[ch];
	struct clk *clk = clk_get(NULL, "pclk");
	volatile unsigned long rate, freq, clock = 0;
	volatile unsigned long hz = 0, pwmhz = 0, mout;
	volatile unsigned int tmux = 0, smux = 0, tscl = 1, tcnt;
	int i, n, end = 0;

	RET_ASSERT_VAL(PWN_CHANNELS > ch, -EINVAL);
	RET_ASSERT_VAL(ch >= 0, -EINVAL);
	RET_ASSERT_VAL(100 >= duty && duty >= 0, -EINVAL);

	if (request > (clk_in_max/DUTY_MIN_VAL)) {
		pr_err("Invalid request=%u hz over max pwm out %ld hz\n",
			request, (clk_in_max/DUTY_MIN_VAL));
		return -EINVAL;
	}
	if (0 == duty) {
		pwm->compare = 0;
		pwm->request = request;
		pwm_set_device(pwm);
		return request;
	}

	/* for suspend -> resume */
    if (pwm->request == request) {
        clock = pwm->rate;
        pwmhz = pwm->pwm_hz;
    }

	/* change only dutycycle */
	if (pwm->request == request && pwm->duty != duty) {
		clock = pwm->rate;
		pwmhz = pwm->pwm_hz;
	} else {
		if(pwm->clk_tclk) {
		/* set new pwm out -- USE TCLK CLKGEN */
			for (n = 1; !end; n *= 10) {
				for (i = (n == 1 ? DUTY_MIN_VAL : 1); 10 > i; i++) {
					freq = request * i * n;
					if (freq > clk_in_max) {
						end = 1;
						break;
					}
					rate = clk_round_rate(pwm->clk, freq);
					tcnt = rate/request;
					if(tcnt == 1){
						printk("generate lower frequency pwm clock than request clock\n");
						tcnt = 2;
					}
					hz = rate/tcnt;
					if (0 == rate%request) {
						clock = rate, pwmhz = hz, end = 1;
						break;
					}

					if (hz && (abs(hz-request) >= abs(pwmhz-request)))
						continue;
					clock = rate, pwmhz = hz;
				}
			}
			tmux = 5;
		}
		else {
		/* set new pwm out -- USE PRESCALE AND MUX BASED ON PCLK*/
			rate = clk_get_rate(clk);
			for (smux = 0; 5 > smux; smux++) {
				mout = rate/tscl/(1<<smux);
				tcnt = mout/request;
				if(tcnt == 1){
					printk("generate lower frequency pwm clock than request clock\n");
					tcnt = 2;
				}
				hz = mout/tcnt;
				if (0 == mout%request) {
					clock = mout, pwmhz = hz, tmux = smux;
					break;
				}

				pr_debug("diff(%ld) %d\n", abs(hz-request), smux);
				if (hz && (abs(hz-request) >= abs(pwmhz-request)))
					continue;
				clock = mout, pwmhz = hz, tmux = smux;
			}
		}
	}

	if (clock) {
		pwm->request = request;
		pwm->rate = clock;
		pwm->duty = duty;
		pwm->pwm_hz = pwmhz;
		pwm->tmux = tmux;
		pwm->counter = clock/pwmhz;
		pwm->compare = PWM_COMPARE(pwm->counter, duty) ? : 1;
		pwm_set_device(pwm);
	} else {
        printk("%s: can't find clock!!!\n", __func__);
    }

	return clock ? pwm->pwm_hz : 0;
}
EXPORT_SYMBOL_GPL(nxp_soc_pwm_set_frequency);

/*------------------------------------------------------------------------------
 * 	Description	: get pwm frequncy info
 *	In[ch]		: pwm channel, 0 ~ 3
 *	out[freq]	: pwm frequency, unit (Hz), : 30Hz ~ 237KHz
 *	out[duty]	: pwm pulse width, unit (%), 0 % ~ 100 %
 *	Return 		: none.
 */
void nxp_soc_pwm_get_frequency(int ch, unsigned int *freq, unsigned int *duty)
{
	struct pwm_device_hw *pwm = &devs_pwm[ch];
	RET_ASSERT(PWN_CHANNELS > ch);

	*freq = pwm->request;
	*duty = pwm->duty;
}
EXPORT_SYMBOL_GPL(nxp_soc_pwm_get_frequency);


/*
 *  SYS Interface
 *
 *  /sys/devices/platform/pwm/pwm.N
 *
 */
#ifdef CONFIG_PWM_SYSFS
static ssize_t pwm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pwm_device_hw *pwm = NULL;
	struct attribute *at = &attr->attr;
	const char *c;
	char *s = buf;
	int ch;

	c  = &at->name[strlen("pwm.")];
	ch = simple_strtoul(c, NULL, 10);
	pwm = &devs_pwm[ch];

	s += sprintf(s, "%ld,%d%% (%u,%u)\n", pwm->pwm_hz, pwm->duty, pwm->counter, pwm->compare);
	if (s != buf)
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t pwm_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t n)
{
	struct pwm_device_hw *pwm = NULL;
	struct attribute *at = &attr->attr;
	const char *c;
	long request = 0, duty = 0;
	int ch;

	c  = &at->name[strlen("pwm.")];
	ch = simple_strtoul(c, NULL, 10);
	pwm = &devs_pwm[ch];

	sscanf(buf,"%ld,%ld", &request, &duty);
	nxp_soc_pwm_set_frequency(ch, request, duty);

	return n;
}

static struct device_attribute pwm0_attr = __ATTR(pwm.0, S_IRUGO | S_IWUSR, pwm_show, pwm_store);
static struct device_attribute pwm1_attr = __ATTR(pwm.1, S_IRUGO | S_IWUSR, pwm_show, pwm_store);
static struct device_attribute pwm2_attr = __ATTR(pwm.2, S_IRUGO | S_IWUSR, pwm_show, pwm_store);
static struct device_attribute pwm3_attr = __ATTR(pwm.3, S_IRUGO | S_IWUSR, pwm_show, pwm_store);

/* sys attribte group */
static struct attribute *attrs[] = {
	&pwm0_attr.attr, &pwm1_attr.attr, &pwm2_attr.attr, &pwm3_attr.attr,	NULL,
};

static struct attribute_group attr_group = {
		.attrs = (struct attribute **)attrs,
};
#endif

/*
 * soc-specific driver
 */

#ifdef CONFIG_PWM_SYSFS
	struct kobject *kobj = NULL;
#endif

static int nxp_soc_pwm_init(void)
{
	struct pwm_device_hw *pwm = &devs_pwm[0];
	struct clk *clk = clk_get(NULL, "pclk");
	char id[32] = {0,};
	int i, ret = 0;

	PWM_RESET();

	for (i = 0; PWN_CHANNELS > i; i++, pwm++) {
#ifdef CFG_PWM_FORCE_INIT
		/* set gpio out low */
		nxp_soc_gpio_set_out_value(pwm->io, 0);
		nxp_soc_gpio_set_io_dir(pwm->io, 1);
		nxp_soc_gpio_set_io_func(pwm->io, pwm->fn_io);
#endif
		sprintf(id, "%s%d", "pwm", i);
		pwm->ch  = i;
		pwm->clk = clk_get(NULL, id);
		mutex_init(&pwm->lock);
	}

	clk_in_max  = clk_get_rate(clk);
#if defined(CONFIG_NXP_DFS_BCLK)
	clk_in_max  = 20*1000*1000;
#else
	clk_in_max /= 2;
#endif

	/* create attribute interface */
#ifdef CONFIG_PWM_SYSFS
	if (kobj == NULL) {
		kobj = kobject_create_and_add("pwm", &platform_bus.kobj);
		if (! kobj) {
			printk(KERN_ERR "Fail, create kobject for display\n");
			return -ret;
		}
		ret = sysfs_create_group(kobj, &attr_group);
		if (ret) {
			printk(KERN_ERR "Fail, create sysfs group for display\n");
			kobject_del(kobj);
			return -ret;
		}
	}
#endif
	printk("pwm: max = %ld hz\n", clk_in_max/DUTY_MIN_VAL);
	return 0;
}

//=====================================================================================

struct nxp_pwm_chip {
    struct pwm_chip	chip;
    struct device   *dev;

    struct clk      *clk;

	void __iomem    *mmio_base;
	unsigned int 	period_ns;
	unsigned int 	duty_ns;
	int				ch;
};

/* refer pwm prototype */
#define	PWM_MODULES		4
#define NS_IN_HZ (1000000000UL)
#define TO_PERCENT(duty, period)		(((duty)*100)/(period))
#define TO_HZ(period)					(NS_IN_HZ/(period))

static int nxp_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct nxp_pwm_chip *pc = container_of(chip, struct nxp_pwm_chip, chip);
	u32 duty_percent = 0;
	u32 period_hz = 0;

	period_hz = TO_HZ(pc->period_ns);
	duty_percent = TO_PERCENT(pc->duty_ns, pc->period_ns);

	nxp_soc_pwm_set_frequency(pc->ch, period_hz, duty_percent);

	return 0;
}

static void nxp_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct nxp_pwm_chip *pc = container_of(chip, struct nxp_pwm_chip, chip);

	pwm_stop(pc->ch, 0);
	clk_disable_unprepare(pc->clk);
}

static int nxp_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	struct nxp_pwm_chip *pc = container_of(chip, struct nxp_pwm_chip, chip);
	u32 duty_percent = 0;
	u32 period_hz = 0;

	if (period_ns > NS_IN_HZ || duty_ns > NS_IN_HZ)
		return -ERANGE;

	if (duty_ns > period_ns) {
		printk("%s error: duty is too big!!!(%d/%d)\n", __func__, duty_ns, period_ns);
		return -ERANGE;
	}

	if (period_ns == pc->period_ns && duty_ns == pc->duty_ns) {
		return 0;
	}

	pc->period_ns = period_ns;
	pc->duty_ns = duty_ns;

	period_hz = TO_HZ(pc->period_ns);
	duty_percent = TO_PERCENT(pc->duty_ns, pc->period_ns);

	pr_debug("set duty(%u percent), frequency(%u HZ)\n",
		duty_percent, period_hz);

	pr_debug("pwm ch[%d]: set duty(%u percent), frequency(%u HZ)\n",
		pc->ch, duty_percent, period_hz);

	nxp_soc_pwm_set_frequency(pc->ch, period_hz, duty_percent);

	return 0;
}

static const struct pwm_ops nxp_pwm_ops = { 
    .config = nxp_pwm_config,
    .enable = nxp_pwm_enable,
    .disable = nxp_pwm_disable,
    .owner = THIS_MODULE,
};

static int nxp_pwm_probe(struct platform_device *pdev)
{
	struct nxp_pwm_chip *pwm;
    struct resource *r;
	int ret;
	unsigned int id = pdev->id;

	id = of_alias_get_id(pdev->dev.of_node, "pwmdev");

	if (id > (PWM_MODULES - 1)) {
		dev_err(&pdev->dev, "id %d is out of range\n", id);
		return -EINVAL;
	}

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pdev->dev, "failed to allocate pwm_device\n");
		return -ENOMEM;
	}

	pwm->dev = &pdev->dev;

	if (base == NULL) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (r == NULL) {
			dev_err(&pdev->dev, "no memory resource specified\n");
			return -ENOENT;
		}

	    pwm->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	    if (!pwm->mmio_base) {
	        printk(KERN_ERR "failed to ioremap memory resource for pwm\n");
	        return -ENOMEM;
	    }    
	    pr_debug("resource %p\n", pwm->mmio_base);

		base = pwm->mmio_base;
	
		nxp_soc_pwm_init();
	} else 
		pwm->mmio_base = base;


	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &nxp_pwm_ops;
	pwm->chip.base = -1;
	pwm->chip.npwm = PWM_MODULES;
	pwm->ch = id;

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pwm);

	return 0;
}

static int nxp_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm = platform_get_drvdata(pdev);
	kfree(pwm);

	return 0;
}

#ifdef CONFIG_PM
static int nxp_pwm_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nxp_pwm_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pwm_suspend 		NULL
#define pwm_resume			NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id nxp_pwm_match[] = {
		{ .compatible = "nexell,nxp-pwm" },
			{},
};
MODULE_DEVICE_TABLE(of, nxp_pwm_match);
#else
#define nxp_pwm_match NULL
#endif

static struct platform_driver nxp_pwm_driver = {
	.probe		= nxp_pwm_probe,
	.remove		= nxp_pwm_remove,
	.suspend	= nxp_pwm_suspend,
	.resume		= nxp_pwm_resume,
	.driver = {
		.name 	= "nxp-pwm",
		.owner 	= THIS_MODULE,
		.of_match_table	= of_match_ptr(nxp_pwm_match),
	},
};

static int __init nxp_pwm_init(void)
{
	return platform_driver_register(&nxp_pwm_driver);
}
arch_initcall(nxp_pwm_init);

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

//#define DEBUG

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/reset.h>


#include <nexell/platform.h>
#include <nexell/soc-s5pxx18.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>

#include "nxp_adc.h"

#ifdef CONFIG_ARCH_S5P4418
#define ADC_USING_PROTOTYPE
#else /* CONFIG_ARCH_S5P6818 */
//#define ADC_USING_PROTOTYPE
#endif

#ifdef CONFIG_ARCH_S5P4418
#define ADC_LOCK_INIT(LOCK)			spin_lock_init(LOCK)
#define ADC_LOCK(LOCK, FLAG)		spin_lock_irqsave(LOCK, FLAG)
#define ADC_UNLOCK(LOCK, FLAG)		spin_unlock_irqrestore(LOCK, FLAG)
#else	/* CONFIG_ARCH_S5P6818 */
#define ADC_LOCK_INIT(LOCK)			do { } while (0)
#define ADC_LOCK(LOCK, FLAG)		do { } while (0)
#define ADC_UNLOCK(LOCK, FLAG)		do { } while (0)
#endif


#ifndef ADC_USING_PROTOTYPE
#define USING_RESET_DRIVER			/* since kernel 3.18 : driver/reset/... */
//#undef USING_RESET_DRIVER
#endif

#ifdef USING_RESET_DRIVER
#if 1
#define	ADC_HW_RESET()		do {				\
			reset_control_reset(adc->rst);		\
		} while (0)
#else
#define	ADC_HW_RESET()		do {				\
			reset_control_assert(adc->rst);		\
			udelay(10);				\
			reset_control_deassert(adc->rst);	\
		} while (0)
#endif
#else
#define	ADC_HW_RESET()		do { nxp_soc_peri_reset_set(RESET_ID_ADC); } while (0)
#endif

#ifdef CONFIG_ARM64
#define ARM_DMB()		dmb(sy)
#else
#define ARM_DMB()		dmb()
#endif




/*
 * ADC data
 */
struct nxp_adc_info {
	void __iomem *adc_base;
	ulong clock_rate;
	ulong sample_rate;
	ulong max_sampele_rate;
	ulong min_sampele_rate;
	int		value;
	int		prescale;
	spinlock_t	lock;
	struct completion completion;
	int support_interrupt;
	int irq;
	struct clk *clk;
	struct iio_map *map;
#if defined(CONFIG_ARM_NXP_CPUFREQ_BY_RESOURCE)
	struct iio_dev *iio;
	struct workqueue_struct *monitoring_wqueue;
	struct delayed_work monitoring_work;

	int board_temperature;
	int tmp_voltage;
	int prev_board_temperature;

	int isValid;
	int bFirst;
	int isCheckedCount;
	struct notifier_block pm_notifier;
	unsigned long resume_state;
#endif
#ifdef USING_RESET_DRIVER
	struct reset_control *rst;
#endif
};

#define	STATE_RESUME_DONE	0

#if 0
static const char *str_adc_ch[] = {
	"adc.0", "adc.1", "adc.2", "adc.3",
	"adc.4", "adc.5", "adc.6", "adc.7",
};
#endif

static const char *str_adc_label[] = {
	"ADC0", "ADC1", "ADC2", "ADC3",
	"ADC4", "ADC5", "ADC6", "ADC7",
};

#define ADC_CHANNEL_SPEC(_id) {	\
	.type = IIO_VOLTAGE,		\
	.indexed = 1,				\
	.channel = _id,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.scan_index = _id,			\
}

static struct iio_chan_spec nxp_adc_iio_channels [] = {
	ADC_CHANNEL_SPEC(0),
	ADC_CHANNEL_SPEC(1),
	ADC_CHANNEL_SPEC(2),
	ADC_CHANNEL_SPEC(3),
	ADC_CHANNEL_SPEC(4),
	ADC_CHANNEL_SPEC(5),
	ADC_CHANNEL_SPEC(6),
	ADC_CHANNEL_SPEC(7),
};


/*
 * for get channel
 * 		consumer_dev_name 	= iio_channel_get_all 	: name = nxp-adc
 * 		consumer_channel 	= iio_channel_get 		: channel_name = adc.0, 1, ...
 *
 * for find channel spec (iio_chan_spec)
 * 		adc_channel_label	= must be equal to iio_chan_spec->datasheet_name = ADC0, 1, ...
 */

static int nxp_adc_remove_devices(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nxp_adc_match[] = {
	{ .compatible = "nexell,nxp-adc" },
	{},
};
MODULE_DEVICE_TABLE(of, nxp_adc_match);
#endif

/*
 * ADC functions
 */
#ifdef CONFIG_NXP_ADC_INTERRUPT
static irqreturn_t nxp_adc_isr(int irq, void *dev_id)
{
	struct nxp_adc_info *adc = (struct nxp_adc_info *)dev_id;
	struct adc_register *reg = adc->adc_base;

	__raw_writel(1, &reg->ADCINTCLR);

	adc->value = __raw_readl(&reg->ADCDAT);	/* get */
	complete(&adc->completion);
	return IRQ_HANDLED;
}
#endif

/*
 * XXX Do not use in release version XXX
 */
#ifdef DEBUG
static void nxp_adc_dump_regs(struct nxp_adc_info *adc)
{
	struct adc_register *reg = adc->adc_base;
	struct adc_register adc_regs;

	adc_regs.ADCCON = reg->ADCCON;
	adc_regs.ADCDAT = reg->ADCDAT;
	adc_regs.ADCINTENB = reg->ADCINTENB;
	adc_regs.ADCINTCLR = reg->ADCINTCLR;
#ifdef CONFIG_ARCH_S5P6818
	adc_regs.ADCPRESCON = reg->ADCPRESCON;
#endif

#ifdef CONFIG_ARCH_S5P4418
	pr_info("CON:%08X, DAT:%08X, INTENB:%08X, INTCLR:%08x \n",
		adc_regs.ADCCON, adc_regs.ADCDAT, adc_regs.ADCINTENB, adc_regs.ADCINTCLR);
#else /* CONFIG_ARCH_S5P6818 */
	pr_info("CON:%08X, DAT:%08X, INTENB:%08X, INTCLR:%08x, PRESCON:%08x \n",
		adc_regs.ADCCON, adc_regs.ADCDAT, adc_regs.ADCINTENB, adc_regs.ADCINTCLR, adc_regs.ADCPRESCON);
#endif
}
#else
static void nxp_adc_dump_regs(struct nxp_adc_info *adc) { }
#endif


static int __turn_around_invalid_first_read(struct nxp_adc_info *adc)
{
	unsigned int adcon = 0;
	struct adc_register *reg = adc->adc_base;
	volatile int value = 0;
	unsigned long wait = loops_per_jiffy * (HZ/10);

	adcon  = __raw_readl(&reg->ADCCON) & ~(0x07 << ASEL_BITP) & ~(0x01 << ADEN_BITP);
	adcon |= 0 << ASEL_BITP;	// channel
	__raw_writel(adcon, &reg->ADCCON);
	adcon  = __raw_readl(&reg->ADCCON);
	adcon |=  1 << ADEN_BITP;	// start
	__raw_writel(adcon, &reg->ADCCON);

	while (wait > 0) {
		if (__raw_readl(&reg->ADCINTCLR) & (1<<AICL_BITP)) {
			__raw_writel(0x1, &reg->ADCINTCLR);	/* pending clear */
			value = __raw_readl(&reg->ADCDAT);	/* get value */
			break;
		}
		wait--;
	}
	return 0;
}

#ifdef ADC_USING_PROTOTYPE
#else
static int setup_adc_con(struct nxp_adc_info *adc)
{
	unsigned int adcon = 0;
	unsigned int pres = 0;
	struct adc_register *reg = adc->adc_base;

#ifdef CONFIG_ARCH_S5P4418
	adcon = ((adc->prescale & 0xFF) << APSV_BITP) |
			(1 << APEN_BITP) |
			(0 << ADCON_STBY) ;
	__raw_writel(adcon, &reg->ADCCON);
#else	/* CONFIG_ARCH_S5P6818 */
	adcon = ((DATA_SEL_VAL & 0xf) << DATA_SEL_BITP) |
			((CLK_CNT_VAL & 0xf)  << CLK_CNT_BITP) |
			(0 << ADCON_STBY);
	__raw_writel(adcon, &reg->ADCCON);

	pres = ((adc->prescale & 0x3FF) << PRES_BITP);
	__raw_writel(pres, &reg->ADCPRESCON);
	pres |= (1 << APEN_BITP);
	__raw_writel(pres, &reg->ADCPRESCON);
#endif

	/* *****************************************************
	 * Turn-around invalid value after Power On
	 * *****************************************************/
	__turn_around_invalid_first_read(adc);


	if (adc->support_interrupt) {
		__raw_writel(1, &reg->ADCINTCLR);
		__raw_writel(1, &reg->ADCINTENB);
		init_completion(&adc->completion);
	}

	return 0;
}
#endif


static int nxp_adc_setup(struct nxp_adc_info *adc, struct platform_device *pdev)
{
	ulong clk_rate, min_rate;
	uint32_t sample_rate;
	int irq = 0, interrupt = 0, prescale = 0;
	int ret = 0;
	int arr_size;

	adc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(adc->clk)) {
		pr_err("Fail: getting clock for ADC !!!\n");
		return -EINVAL;
	}
 	clk_rate = clk_get_rate(adc->clk);
	clk_prepare_enable(adc->clk);

#ifdef USING_RESET_DRIVER
	adc->rst = devm_reset_control_get(&pdev->dev, "reset");
	if (IS_ERR(adc->rst)) {
		dev_err(&pdev->dev, "failed to get reset\n");
		return PTR_ERR(adc->rst);
	}
#endif

	of_property_read_u32(pdev->dev.of_node, "sample_rate", &sample_rate);

	prescale = clk_rate/(sample_rate * ADC_MAX_SAMPLE_BITS);
	min_rate = clk_rate/(ADC_MAX_PRESCALE * ADC_MAX_SAMPLE_BITS);

	if (sample_rate > ADC_MAX_SAMPLE_RATE ||
		min_rate > sample_rate) {
		pr_err("ADC: not suport %u(%d ~ %lu) sample rate\n",
			sample_rate, ADC_MAX_SAMPLE_RATE, min_rate);
		clk_disable_unprepare(adc->clk);
		return -EINVAL;
	}

#ifdef CONFIG_NXP_ADC_INTERRUPT
	irq = platform_get_irq(pdev, 0);
	if ((irq >= 0) || (NR_IRQS > irq)) {
		ret = request_irq(irq, nxp_adc_isr, 0, dev_name(&pdev->dev), adc);
		if (0 == ret)
			interrupt = 1;
		ret = 0;
	}
#endif

	adc->clock_rate = clk_rate;
	adc->support_interrupt = interrupt;
	adc->irq = irq;
	adc->sample_rate = sample_rate;
	adc->max_sampele_rate = ADC_MAX_SAMPLE_RATE;
	adc->min_sampele_rate = min_rate;
	adc->prescale = prescale;
	ADC_LOCK_INIT(&adc->lock);

#ifdef ADC_USING_PROTOTYPE
	ADC_HW_RESET();

	NX_ADC_Initialize();
	NX_ADC_SetBaseAddress(0, (void *)__io_address(NX_ADC_GetPhysicalAddress(0)));
 	NX_ADC_OpenModule(0);

	NX_ADC_SetInputChannel(0, 0);
	NX_ADC_SetStandbyMode(0, CFALSE);
	NX_ADC_SetPrescalerValue(0, adc->prescale);
	NX_ADC_ClearInterruptPendingAll(0);
	NX_ADC_SetPrescalerEnable(0, CTRUE);
	NX_ADC_SetInterruptEnableAll(0, CTRUE);
	#ifdef CONFIG_ARCH_S5P6818
	NX_ADC_SetADCDataDelay(0, DATA_SEL_VAL);
	NX_ADC_SetSOCDelay(0, CLK_CNT_VAL);
	#endif

    pr_debug(" [Standy Mode            ] : %12s 			  \r\n", NX_ADC_GetStandbyMode( 0 ) ? "ADC Power Off(Stand By)" : "ADC Power On" ); 
    pr_debug(" [Prescaler Divide Value ] : %8d  			  \r\n", NX_ADC_GetPrescalerValue( 0 ) ); 
    pr_debug(" [Prescaler Divide Enable] : %8s  			  \r\n", NX_ADC_GetPrescalerEnable( 0 ) ? "ENABLE" : "DISABLE" );    
    pr_debug(" [Interrup Enable Bit    ] : %8s  			  \r\n", NX_ADC_GetInterruptEnable( 0, 0 ) ? "ENABLE" : "DISABLE" );

#else
	ADC_HW_RESET();

	setup_adc_con(adc);
#endif

	arr_size = ARRAY_SIZE(nxp_adc_iio_channels);
	pr_info("ADC: CHs %d, %ld(%ld ~ %ld) sample rate, %s mode, scale=%d(bit %d)\n",
		arr_size, adc->sample_rate,
		adc->max_sampele_rate, adc->min_sampele_rate,
		interrupt?"irq":"polling", prescale, ADC_MAX_SAMPLE_BITS);

	return ret;
}

static void nxp_adc_release(struct nxp_adc_info *adc)
{
	if (adc->support_interrupt)
		free_irq(adc->irq, adc);
}

static int nxp_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long mask)
{
	struct nxp_adc_info *adc = iio_priv(indio_dev);
	struct adc_register *reg = adc->adc_base;
	int ch = chan->channel;
	unsigned long wait = loops_per_jiffy * (HZ/10);
	volatile unsigned int adcon = 0;
	volatile int value = 0;
	unsigned long flags = flags;

	if (adc->support_interrupt) {
		mutex_lock(&indio_dev->mlock);

		adcon  = __raw_readl(&reg->ADCCON) & ~(0x07 << ASEL_BITP);
		adcon |= ch << ASEL_BITP;	// channel
		adcon |=  1 << ADEN_BITP;	// start
		__raw_writel(adcon, &reg->ADCCON);

		wait_for_completion(&adc->completion);
		*val = adc->value;

		mutex_unlock(&indio_dev->mlock);
	} else {
#ifdef ADC_USING_PROTOTYPE

		ADC_LOCK(&adc->lock, flags);

		NX_ADC_SetInputChannel(0, ch);
		NX_ADC_ClearInterruptPendingAll(0);
		NX_ADC_Start(0);

		while (wait > 0) {
			if (NX_ADC_GetInterruptPendingAll(0)) {
				value = NX_ADC_GetConvertedData(0);
				NX_ADC_ClearInterruptPendingAll(0);
				break;
			}
			wait--;
		}

		nxp_adc_dump_regs(adc);

		if (0 >= wait) {
			ADC_UNLOCK(&adc->lock, flags);
			return -EINVAL;
		}

		*val = value;
		ADC_UNLOCK(&adc->lock, flags);

#else
		ADC_LOCK(&adc->lock, flags);

		adcon  = __raw_readl(&reg->ADCCON) & ~(0x07 << ASEL_BITP) & ~(0x01 << ADEN_BITP);
		adcon |= ch << ASEL_BITP;	// channel
		__raw_writel(adcon, &reg->ADCCON);
		ARM_DMB();
		adcon |=  1 << ADEN_BITP;	// start
		__raw_writel(adcon, &reg->ADCCON);
		
		__raw_writel(0x1, &reg->ADCINTCLR);
		__raw_writel(0x1, &reg->ADCINTENB);

		/* *****************************************************
		 * Set register values direct for test.
		 * *****************************************************/
		//#ifdef CONFIG_ARCH_S5P6818
		//__raw_writel(0x80F9, &reg->ADCPRESCON);
		//#endif
		//__raw_writel(0x8180, &reg->ADCCON);
		ARM_DMB();
		while (wait > 0) {
			if (__raw_readl(&reg->ADCINTCLR) & (1<<AICL_BITP)) {
				__raw_writel(0x1, &reg->ADCINTCLR);	/* pending clear */
				value = __raw_readl(&reg->ADCDAT);	/* get value */
				break;
			}
			wait--;
		}

		nxp_adc_dump_regs(adc);

		*val = value;


		if (0 >= wait) {
			ADC_UNLOCK(&adc->lock, flags);
			return -EINVAL;
		}
		ADC_UNLOCK(&adc->lock, flags);
#endif
	}

	//usleep_range (1, 10);
	pr_debug("%s, ch=%d, val=0x%x\n", __func__, ch, *val);

	return IIO_VAL_INT;
}

static const struct iio_info nxp_adc_iio_info = {
	.read_raw = &nxp_read_raw,
	.driver_module = THIS_MODULE,
};

static int nxp_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nxp_adc_resume(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct nxp_adc_info *adc = iio_priv(indio_dev);

#ifdef ADC_USING_PROTOTYPE
	NX_ADC_SetBaseAddress(0, (void *)__io_address(NX_ADC_GetPhysicalAddress(0)));
 	NX_ADC_OpenModule(0);

	ADC_HW_RESET();
	//NX_ADC_SetInputChannel(0, ch);
	NX_ADC_SetStandbyMode(0, CFALSE);
	NX_ADC_SetPrescalerValue(0, adc->prescale) ;
	NX_ADC_ClearInterruptPendingAll(0);
	NX_ADC_SetPrescalerEnable(0, CTRUE);
	NX_ADC_SetInterruptEnableAll(0, CTRUE);
	#ifdef CONFIG_ARCH_S5P6818
	NX_ADC_SetADCDataDelay(0, DATA_SEL_VAL);
	NX_ADC_SetSOCDelay(0, CLK_CNT_VAL);
	#endif
#else
	ADC_HW_RESET();

	setup_adc_con(adc);
#endif
	return 0;
}


#if defined(CONFIG_ARM_NXP_CPUFREQ_BY_RESOURCE)
int eBoard_temperature = 0;
int NXP_Get_BoardTemperature(void)
{
	return eBoard_temperature;
}
EXPORT_SYMBOL_GPL(NXP_Get_BoardTemperature);

static int nxp_cpufreq_pm_notify(struct notifier_block *this,
        unsigned long mode, void *unused)
{
	struct nxp_adc_info *adc = container_of(this,
					struct nxp_adc_info, pm_notifier);

    switch(mode) {
    case PM_SUSPEND_PREPARE:
    	clear_bit(STATE_RESUME_DONE, &adc->resume_state);
    	break;
    case PM_POST_SUSPEND:
    	set_bit(STATE_RESUME_DONE, &adc->resume_state);
    	break;
    }
	return 0;
}

// initialize table for register value matching with temperature
static int drone_temperature_table[10][2] =
{
	{9900, 40}, // 0
	{9100, 45},
	{8400, 50},
	{7700, 55},
	{7000, 60}, // 4
	{6300, 65}, // 5
	{5700, 70},
	{5200, 75},
	{4700, 80},
	{4200, 85}  // 9
};

static void nxl_monitor_work_func(struct work_struct *work)
{
	struct nxp_adc_info *adc = container_of(work, struct nxp_adc_info, monitoring_work.work);
	struct iio_chan_spec const *chan;
	int val = 0;
	int val2 = 0;
	// calculate temperature
	int voltage_table_interval;
	int interval;
	int num_grade;


	if (!test_bit(STATE_RESUME_DONE, &adc->resume_state))
		goto exit_mon;

	chan = &nxp_adc_iio_channels[2];
	nxp_read_raw(adc->iio, chan, &val, &val2, 0);

	adc->tmp_voltage = (18*val*1000)/4096;

	//  according to Register Voltage table, calculate board temperature.
	for(num_grade=0, interval=0; num_grade<10; num_grade++)
	{
		if(adc->tmp_voltage > drone_temperature_table[num_grade][0])
		{
			if(num_grade != 0)
			{
				interval = (drone_temperature_table[num_grade-1][0] - drone_temperature_table[num_grade][0])/5;
				break;
			}
		}
	}

	if(num_grade == 10)
	{
		adc->board_temperature = 90;
	}
	else if(interval == 0)
	{
		adc->board_temperature = 40;
	}
	else
	{
		adc->board_temperature = drone_temperature_table[num_grade-1][1];
		voltage_table_interval = drone_temperature_table[num_grade-1][0] - interval;
		for(; voltage_table_interval>drone_temperature_table[num_grade][0]; voltage_table_interval-=interval)
		{
			if(adc->tmp_voltage > voltage_table_interval)
				break;
			adc->board_temperature++;
		}
		if(adc->board_temperature > drone_temperature_table[num_grade][1])
			adc->board_temperature = drone_temperature_table[num_grade][1];
	}

	// ignore the temperature value when booting.
	if(adc->isValid == 0)
	{
		if(adc->bFirst == 0)
		{
			adc->bFirst = 1;
			adc->prev_board_temperature = adc->board_temperature;
		}
		else
		{
			if(adc->prev_board_temperature == adc->board_temperature)
				adc->isCheckedCount++;
			else
				adc->isCheckedCount = 0;
			adc->prev_board_temperature = adc->board_temperature;

			if(adc->isCheckedCount == 3)
				adc->isValid = 1;
		}
		if(adc->isValid == 0)
		{
			queue_delayed_work(adc->monitoring_wqueue, &adc->monitoring_work, HZ);
			return;
		}
	}


	// adjust the temperature value .
	if(adc->prev_board_temperature <= adc->board_temperature)
	{
		int gap = adc->board_temperature - adc->prev_board_temperature;
		if(gap >= 5) // ignore.
		{
			adc->board_temperature = adc->prev_board_temperature;
		}
		else
		{
			adc->prev_board_temperature = adc->board_temperature;
		}
	}
	else
	{
		int gap = adc->prev_board_temperature  - adc->board_temperature;
		if(gap >= 5) // ignore.
		{
			adc->board_temperature = adc->prev_board_temperature;
		}
		else
		{
			adc->prev_board_temperature = adc->board_temperature;
		}
	}

	eBoard_temperature = adc->board_temperature;

exit_mon:
	queue_delayed_work(adc->monitoring_wqueue, &adc->monitoring_work, HZ);

}
#endif

static int nxp_adc_probe(struct platform_device *pdev)
{
#if defined(CONFIG_ARM_NXP_CPUFREQ_BY_RESOURCE)
	static struct notifier_block *pm_notifier;
#endif
	struct iio_dev *iio = NULL;
	struct nxp_adc_info *adc = NULL;
	struct iio_chan_spec *spec;
	struct resource	*mem;
	struct device_node *np = pdev->dev.of_node;
	int i = 0, ret = -ENODEV;

	if (!np)
		return ret;

	iio = iio_device_alloc(sizeof(struct nxp_adc_info));
	if (!iio) {
		pr_err("Fail: allocating iio ADC device\n");
		return -ENOMEM;
	}

	adc = iio_priv(iio);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adc->adc_base = devm_ioremap_resource(&pdev->dev, mem);
	if (!adc->adc_base) {
		ret = -ENOMEM;
		goto err_iio_free;
	}

	ret = nxp_adc_setup(adc, pdev);
	if (0 > ret) {
		pr_err("Fail: setup iio ADC device\n");
		goto err_iio_free;
	}

	platform_set_drvdata(pdev, iio);

	iio->name = dev_name(&pdev->dev);
	iio->dev.parent = &pdev->dev;
	iio->info = &nxp_adc_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = nxp_adc_iio_channels;
	iio->num_channels = ARRAY_SIZE(nxp_adc_iio_channels);
	iio->dev.of_node = pdev->dev.of_node;

	/*
	* sys interface : user interface
	*/
	spec = nxp_adc_iio_channels;
	for (i = 0; iio->num_channels > i; i++)
		spec[i].datasheet_name = str_adc_label[i];

	ret = iio_device_register(iio);
	if (ret)
		goto err_iio_release;


	ret = of_platform_populate(np, nxp_adc_match, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed adding child nodes\n");
		goto err_of_populate;
	}


#if defined(CONFIG_ARM_NXP_CPUFREQ_BY_RESOURCE)
	adc->isCheckedCount = 0;
	adc->isValid = 0;
	adc->bFirst = 0;

	adc->iio = iio;
	adc->monitoring_wqueue = create_singlethread_workqueue("monitoring_wqueue");
	INIT_DELAYED_WORK_DEFERRABLE(&adc->monitoring_work, nxl_monitor_work_func);
	queue_delayed_work(adc->monitoring_wqueue, &adc->monitoring_work, 15*HZ);

	pm_notifier = &adc->pm_notifier;
	pm_notifier->notifier_call = nxp_cpufreq_pm_notify;
	if (register_pm_notifier(pm_notifier)) {
		dev_err(&pdev->dev, "%s: Cannot pm notifier \n", __func__);
		return -1;
	}
	set_bit(STATE_RESUME_DONE, &adc->resume_state);
#endif

	pr_debug("ADC init success\n");

	return 0;

err_of_populate:
	device_for_each_child(&pdev->dev, NULL,
				nxp_adc_remove_devices);
	clk_disable_unprepare(adc->clk);
//err_iio_register:
	iio_device_unregister(iio);
err_iio_release:
	nxp_adc_release(adc);
err_iio_free:
	iio_device_free(iio);
//err_clk:
	pr_err("Fail: load ADC driver ...\n");
	return ret;
}

static int nxp_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct nxp_adc_info *adc = iio_priv(iio);

	device_for_each_child(&pdev->dev, NULL,
				nxp_adc_remove_devices);
	clk_disable_unprepare(adc->clk);
	iio_device_unregister(iio);
	nxp_adc_release(adc);
	platform_set_drvdata(pdev, NULL);

	iio_device_free(iio);
	return 0;
}

static struct platform_driver nxp_adc_driver = {
	.probe		= nxp_adc_probe,
	.remove		= nxp_adc_remove,
	.suspend	= nxp_adc_suspend,
	.resume		= nxp_adc_resume,
	.driver		= {
		.name	= "nxp-adc",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nxp_adc_match),
	},
};

static int __init nxp_adc_init(void)
{
	return platform_driver_register(&nxp_adc_driver);
}

static void __exit nxp_adc_exit(void)
{
	return platform_driver_unregister(&nxp_adc_driver);
}

subsys_initcall(nxp_adc_init);
module_exit(nxp_adc_exit);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("ADC driver for the Nexell");
MODULE_LICENSE("GPL");

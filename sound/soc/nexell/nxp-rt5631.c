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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/jack.h>

#include <nexell/soc-s5pxx18.h>
#include <nexell/sound.h>
#include "nxp-i2s.h"

#include "../codecs/rt5631.h"

/*
#define	pr_debug	printk
*/

#include <linux/gpio.h>

#define I2S_BASEADDR            PHYS_BASE_I2S0
#define I2S_CH_OFFSET           0x1000

static char str_dai_name[16] = "c0055000.i2s";
static int (*cpu_resume_fn)(struct snd_soc_dai *dai) = NULL;
static struct snd_soc_codec *rt5631 = NULL;
static int codec_bias_level = 0;
static int amp_gpio = 0;

static int rt5631_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int freq = params_rate(params) * 256;	/* 48K * 256 = 12.288 Mhz */
	unsigned int fmt  = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
						SND_SOC_DAIFMT_CBS_CFS;
	int ret = 0;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, freq, SND_SOC_CLOCK_IN);
	if (0 > ret)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (0 > ret)
		return ret;
	return ret;
}

static int rt5631_resume_pre(struct snd_soc_card *card)
{
	struct snd_soc_dai *cpu_dai = card->rtd->cpu_dai;
    struct snd_soc_codec *codec = rt5631;

	int ret = 0;
	pm_dbgout("+%s\n", __func__);

	/*
	 * first execute cpu(i2s) resume and execute codec resume.
	 */
	if (cpu_dai->driver->resume && ! cpu_resume_fn) {
		cpu_resume_fn  = cpu_dai->driver->resume;
		cpu_dai->driver->resume = NULL;
	}

	if (cpu_resume_fn)
		ret = cpu_resume_fn(cpu_dai);

	pm_dbgout("-%s\n", __func__);
    codec_bias_level = codec->dapm.bias_level;

	return ret;
}

static int rt5631_resume_post(struct snd_soc_card *card)
{
    struct snd_soc_codec *codec = rt5631;
    pm_dbgout("%s BAIAS=%d, PRE=%d\n", __func__, codec->dapm.bias_level, codec_bias_level);

    if (SND_SOC_BIAS_OFF != codec_bias_level)
        codec->driver->resume(codec);

    return 0;
}

static struct snd_soc_ops rt5631_ops = {
	.hw_params 		= rt5631_hw_params,
};

static int rt5631_spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	pr_debug("%s:%d (event:%d)\n", __func__, __LINE__, event);
	if (amp_gpio > 0) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			gpio_set_value(amp_gpio, 1);
		else
			gpio_set_value(amp_gpio, 0);
	}
	return 0;
}

/* rt5631 machine dapm widgets */
static const struct snd_soc_dapm_widget rt5631_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", rt5631_spk_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

/* Corgi machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route rt5631_audio_map[] = {
	/* headphone connected to HPOL, HPOR */
	{"Headphone Jack", NULL, "HPOL"},
	{"Headphone Jack", NULL, "HPOR"},

	/* speaker connected to SPOL, SPOR */
	{"Ext Spk", NULL, "SPOR"},
	{"Ext Spk", NULL, "SPOL"},
};

#if defined(CONFIG_PLAT_S5P4418_SECRET) || !defined(CONFIG_ANDROID)
/* Headphones jack detection DAPM pin */
static struct snd_soc_jack_pin jack_pins[] = {
	{
		.pin	= "Headphone Jack",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Ext Spk",
		.mask   = SND_JACK_HEADPHONE,
		.invert	= 1,				// when insert disalbe
	},
};
#endif

/* Headphones jack detection GPIO */
static struct snd_soc_jack_gpio jack_gpio = {
	.invert		= false,			// High detect : invert = false
	.name		= "hp-gpio",
	.report		= SND_JACK_HEADPHONE,
	.debounce_time	= 200,
};

static struct snd_soc_jack hp_jack;

static int rt5631_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_jack_gpio *jack = &jack_gpio;
	int ret;

	rt5631 = codec;

	/* set endpoints to not connected */
	snd_soc_dapm_nc_pin(dapm, "AUXO1");
	snd_soc_dapm_nc_pin(dapm, "AUXO2");
	snd_soc_dapm_nc_pin(dapm, "MONO");

	if (NULL == jack->name)
		return 0;

	/* Headset jack detection */
	ret = snd_soc_jack_new(codec, "Headphone Jack",
				SND_JACK_HEADPHONE, &hp_jack);
	if (ret)
		return ret;
#if defined(CONFIG_PLAT_S5P4418_SECRET) || !defined(CONFIG_ANDROID)
	printk("==enable jack switch for linux==\n");
	ret = snd_soc_jack_add_pins(&hp_jack, ARRAY_SIZE(jack_pins), jack_pins);
	if (ret)
		return ret;

	/* to power up rt5631 (HP Depop: hp_event) */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_sync(dapm);
#endif
	ret = snd_soc_jack_add_gpios(&hp_jack, 1, jack);
	if (ret)
		printk("Fail, register audio jack detect, io [%d]...\n", jack->gpio);

	return 0;
}

static struct snd_soc_dai_link rt5631_dai_link = {
	.name 			= "ASOC-RT5631",
	.stream_name 	= "rt5631 HiFi",
	.cpu_dai_name 	= str_dai_name,			/* nxp_snd_i2s_driver name */
	.platform_name  = "nxp-pcm",			/* nxp_snd_pcm_driver name */
	.codec_dai_name = "rt5631-hifi",		/* rt5631_dai's name */
	.codec_name 	= "rt5631.0-001a",		/* rt5631_i2c_driver name + '.' + bus + '-' + address(7bit) */
	.ops 			= &rt5631_ops,
	.symmetric_rates = 1,
	.init		= rt5631_dai_init,
};

static struct snd_soc_card rt5631_card = {
	.name 				= "I2S-RT5631",		/* proc/asound/cards */
	.owner 				= THIS_MODULE,
	.dai_link 			= &rt5631_dai_link,
	.num_links 			= 1,
	.resume_pre			= &rt5631_resume_pre,
	.resume_post		= &rt5631_resume_post,
	.dapm_widgets 		= rt5631_dapm_widgets,
	.num_dapm_widgets 	= ARRAY_SIZE(rt5631_dapm_widgets),
	.dapm_routes 		= rt5631_audio_map,
	.num_dapm_routes 	= ARRAY_SIZE(rt5631_audio_map),
};

/*
 * codec driver
 */
static int rt5631_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rt5631_card;
	struct snd_soc_jack_gpio *jack = &jack_gpio;
	struct snd_soc_dai_driver *i2s_dai = NULL;
	struct nxp_snd_jack_pin hpin = {0.};
	unsigned int rates = 0, format = 0;
	int ret;
    int ch;
	const char *format_name;

	/* set I2S name */
    of_property_read_u32(pdev->dev.of_node, "ch", &ch);
	sprintf(str_dai_name, "%x%s%d", (I2S_BASEADDR + (ch * I2S_CH_OFFSET)), ".i2s", ch);
	of_property_read_u32(pdev->dev.of_node, "sample_rate", &rates);	
	format_name = of_get_property(pdev->dev.of_node, "format", NULL);
    if (format_name != NULL) {
		if (strcmp(format_name,"S16") == 0)
			format = SNDRV_PCM_FMTBIT_S16_LE;
		else if(strcmp(format_name,"S24") == 0)
			format = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE;
	}
	of_property_read_u32(pdev->dev.of_node, "hpin_support", &hpin.support);	
	of_property_read_u32(pdev->dev.of_node, "hpin_gpio", &hpin.detect_io);	
	of_property_read_u32(pdev->dev.of_node, "hpin_level", &hpin.detect_level);	
	of_property_read_u32(pdev->dev.of_node, "amp_gpio", &amp_gpio);	

	if (hpin.support) {
		jack->gpio = hpin.detect_io;
		jack->invert = hpin.detect_level ?  false : true;
		jack->debounce_time = hpin.debounce_time ?
				hpin.debounce_time : 200;
	} else {
		jack->name = NULL;
	}
	if (amp_gpio > 0) {
		gpio_request(amp_gpio, "rt5631_amp_en");
		gpio_direction_output(amp_gpio, 0);
	}
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
		return ret;
	}
	if (card->rtd) {
		struct snd_soc_dai *cpu_dai = card->rtd->cpu_dai;
		if (cpu_dai)
			i2s_dai = cpu_dai->driver;
	}
	pr_debug("rt5631-dai: register card %s -> %s\n",
		card->dai_link->codec_dai_name, card->dai_link->cpu_dai_name);

	if (NULL == i2s_dai)
		return 0;
	/*
	 * Reset i2s sample rates
	 */
	if (rates) {
		rates = snd_pcm_rate_to_rate_bit(rates);
		if (SNDRV_PCM_RATE_KNOT == rates)
			printk("%s, invalid sample rates=%d\n", __func__, rates);
		else {
			i2s_dai->playback.rates = rates;
			i2s_dai->capture.rates = rates;
		}
	}
	/*
	 * Reset i2s format
	 */
	if (format) {
		i2s_dai->playback.formats = format;
		i2s_dai->capture.formats = format;
	}
	return ret;
}

static int rt5631_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	if (amp_gpio > 0) {
		gpio_free(amp_gpio);
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nxp_rt5631_match[] = { 
	{ .compatible = "nexell,nxp-rt5631" },
	{},
};
MODULE_DEVICE_TABLE(of, nxp_rt5631_match);
#else
#define nxp_rt5631_match NULL
#endif

static struct platform_driver rt5631_driver = {
	.driver		= {
		.name	= "rt5631-audio",
		.owner	= THIS_MODULE,
		.pm 	= &snd_soc_pm_ops,	/* for suspend */
		.of_match_table = nxp_rt5631_match,
	},
	.probe		= rt5631_probe,
	.remove		= rt5631_remove,
};
module_platform_driver(rt5631_driver);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound codec-rt5631 driver for the SLSI");
MODULE_LICENSE("GPL");

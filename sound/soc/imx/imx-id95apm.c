/*
 * imx-id95apm_codec-id95apm.c  --  i.MX U-MoBo Driver for IDT P95020 Codec
 *
 * Copyright (C) 2011-2013 U-MoBo. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "imx-ssi.h"

static struct imx_id95apm_audio_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
} card_priv;

static struct snd_soc_jack hs_jack;
static struct snd_soc_card imx_id95apm_audio;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Ext Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Int Mic",
		.mask = SND_JACK_MICROPHONE,
		.invert = 1,
	},
	{
		.pin = "Speaker",
		.mask = SND_JACK_HEADPHONE,
		.invert = 1,
	},
};

/* Headset jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpios = {
	/* gpio is set on per-platform basis */
	.name           = "hs-gpio",
	.report         = SND_JACK_HEADSET,
	.debounce_time	= 200,
	.polling_time	= 500,
};

/*
 * simplyfied version of the clock settings assuming:
 * - no changes to ssi's sys clock
 * - DIV2 = 0
 * - PSR (prescaler range) = 0
 */
static inline unsigned int get_prescaler_modulus(unsigned long sysclk_rate, unsigned int rate, short bits_per_sample, unsigned int channels)
{
	unsigned int bitclk, pm;
	unsigned int dummy, real_rate;

	bitclk = bits_per_sample * channels * rate;

	/* rounded divisions */
	pm = (sysclk_rate + bitclk) / (bitclk << 1);
	if (pm == 0)
		pm = 1;	/* minimum allowed */

	dummy = bits_per_sample * channels * pm;
	real_rate = (sysclk_rate + dummy) / (dummy << 1);

	if (rate != real_rate)
		pr_warning("imx-id95apm: requested %dbits(%s)@%dHz, provided %dHz\n", bits_per_sample, (channels == 1 ? "MONO" : "STEREO"), rate, real_rate);

	return --pm;
}


static int imx_id95apm_dai_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	unsigned int prescaler_modulus;

	/*
	 * The word length is fixed to 32 in I2S Master mode and the WL bits
	 * determine the number of bits that will contain valid data (out of
	 * the 32 transmitted/received bits in each channel).
	 * Ref i.MX53 Reference Manual, rev 2.1 06/2012,
	 * pag 4467, end of paragraph 73.7.1.4
	 */
	prescaler_modulus = get_prescaler_modulus(card_priv.sysclk, rate, 32, channels);

	/* ID95APM BCLK and LRC slave */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as output */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, card_priv.sysclk, SND_SOC_CLOCK_OUT);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_2, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_PM, prescaler_modulus);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, prescaler_modulus);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_PSR, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, 0);

	return 0;
}

/*
 * imx_id95apm_codec ID95APM audio DAI opserations.
 */
static struct snd_soc_ops imx_id95apm_dai_ops = {
	.hw_params = imx_id95apm_dai_hw_params,
};

static const struct snd_soc_dapm_widget imx_id95apm_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* comment out jacks that are not present to power down path not in use */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "MICL", NULL, "Int Mic" },

	{ "MICR", NULL, "Ext Mic" },

	{ "Headphone", NULL, "HPL" },
	{ "Headphone", NULL, "HPR" },

	{ "Speaker", NULL, "SPKRL" },
	{ "Speaker", NULL, "SPKRR" },
};

static int imx_id95apm_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* Add pecific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_id95apm_dapm_widgets,
				  ARRAY_SIZE(imx_id95apm_dapm_widgets));

	/* Set up specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone");
	snd_soc_dapm_enable_pin(&codec->dapm, "Ext Mic");
	snd_soc_dapm_enable_pin(&codec->dapm, "Speaker");
	snd_soc_dapm_enable_pin(&codec->dapm, "Int Mic");
	snd_soc_dapm_sync(&codec->dapm);

	if (hs_jack_gpios.gpio != -1) {
		/* Jack detection API stuff */
		ret = snd_soc_jack_new(codec, "HP Jack",
				       SND_JACK_HEADPHONE, &hs_jack);
		if (ret) {
			printk(KERN_ERR "failed to call snd_soc_jack_new: %d\n", ret);
			return ret;
		}

		ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
					hs_jack_pins);
		if (ret) {
			printk(KERN_ERR "failed to call snd_soc_jack_add_pins: %d\n", ret);
			return ret;
		}

		ret = snd_soc_jack_add_gpios(&hs_jack, 1, &hs_jack_gpios);
		if (ret)
			printk(KERN_WARNING "failed to call snd_soc_jack_add_gpios: %d\n", ret);
	}

	return 0;
}

static struct snd_soc_dai_link imx_id95apm_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "id95apm-dai",
		.codec_name	= "id95apm-codec.0",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_id95apm_dai_init,
		.ops		= &imx_id95apm_dai_ops,
	},
};

static struct snd_soc_card imx_id95apm_audio = {
	.name		= "id95apm-audio",
	.dai_link	= imx_id95apm_dai,
	.num_links	= ARRAY_SIZE(imx_id95apm_dai),
	.owner		= THIS_MODULE,
};

static struct platform_device *imx_id95apm_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_id95apm_audio_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int ret = 0;

	card_priv.pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	ret = -EINVAL;
	if (plat->init && plat->init())
		return ret;

	card_priv.sysclk = plat->sysclk;

	hs_jack_gpios.gpio = plat->hp_gpio;
	hs_jack_gpios.invert = plat->hp_active_low;

	return 0;
}

static int __devexit imx_id95apm_audio_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_id95apm_audio_driver = {
	.probe = imx_id95apm_audio_probe,
	.remove = imx_id95apm_audio_remove,
	.driver = {
		   .name = "imx-id95apm",
		   },
};

static struct platform_device *imx_id95apm_snd_device;

static int __init imx_id95apm_audio_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_id95apm_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_id95apm_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_id95apm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_id95apm_snd_device, &imx_id95apm_audio);

	ret = platform_device_add(imx_id95apm_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_id95apm_snd_device);
	}

	return ret;
}

static void __exit imx_id95apm_audio_exit(void)
{
	platform_driver_unregister(&imx_id95apm_audio_driver);
	platform_device_unregister(imx_id95apm_snd_device);
}

module_init(imx_id95apm_audio_init);
module_exit(imx_id95apm_audio_exit);

MODULE_AUTHOR("Pierluigi Passaro <p.passaro@u-mobo.com>");
MODULE_DESCRIPTION("ID95APM Driver for i.MX boards");
MODULE_LICENSE("GPL");

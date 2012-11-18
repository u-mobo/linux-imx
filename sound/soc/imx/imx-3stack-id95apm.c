/*
 * imx-3stack-id95apm.c  --  i.MX 3Stack Driver for IDT P95020 Codec
 *
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    21th Oct 2008   Initial version.
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include "imx-ssi.h"
#include "imx-pcm.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
#include <linux/mxc_asrc.h>

static unsigned int id95apm_rates[] = {
	0,
	8000,
	11025,
	16000,
	22050,
	32000,
	44100,
	48000,
	88200,
	96000,
};

struct asrc_esai {
	unsigned int cpu_dai_rates;
	unsigned int codec_dai_rates;
	enum asrc_pair_index asrc_index;
	unsigned int output_sample_rate;
};

static struct asrc_esai asrc_ssi_data;
#endif

/* ID95APM SSI BCLK and LRC slave */
#define ID95APM_SSI_MASTER	0

#define ID95APM_SYSCLK		0x00
#define ID95APM_LRCLK		0x01

extern struct snd_soc_dai id95apm_dai;
extern struct snd_soc_codec_device id95apm_soc_codec_dev;

struct imx_3stack_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
};

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
static int get_format_width(struct snd_pcm_hw_params *params)
{
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		return 8;

	case SNDRV_PCM_FORMAT_U16:
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		return 16;

	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_U24_BE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
		return 24;

	case SNDRV_PCM_FORMAT_S32:
	case SNDRV_PCM_FORMAT_U32:
		return 32;

	default:
		return 0;
	}

	return 0;
}
#endif

static struct imx_3stack_priv card_priv;

static int imx_3stack_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_3stack_priv *priv = &card_priv;
	unsigned int rate = params_rate(params);
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;

	unsigned int channels = params_channels(params);
	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (priv->hw)
		return 0;
	priv->hw = 1;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if ((asrc_ssi_data.output_sample_rate != 0)
	    && (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)) {
		unsigned int asrc_input_rate = rate;
		unsigned int channel = params_channels(params);
		struct mxc_runtime_data *pcm_data =
		    substream->runtime->private_data;
		struct asrc_config config = {0};
		struct mxc_audio_platform_data *plat;
		struct imx_3stack_priv *priv = &card_priv;
		int retVal = 0;
		retVal = asrc_req_pair(channel, &asrc_ssi_data.asrc_index);
		if (retVal < 0) {
			pr_err("asrc_req_pair fail\n");
			return -1;
		}
		config.pair = asrc_ssi_data.asrc_index;
		config.channel_num = channel;
		config.input_sample_rate = asrc_input_rate;
		config.output_sample_rate = asrc_ssi_data.output_sample_rate;
		config.inclk = INCLK_NONE;
		config.word_width = get_format_width(params);
		plat = priv->pdev->dev.platform_data;
		if (plat->src_port == 1)
			config.outclk = OUTCLK_SSI1_TX;
		else
			config.outclk = OUTCLK_SSI2_TX;
		retVal = asrc_config_pair(&config);
		if (retVal < 0) {
			pr_err("Fail to config asrc\n");
			asrc_release_pair(asrc_ssi_data.asrc_index);
			return retVal;
		}
		rate = asrc_ssi_data.output_sample_rate;
		pcm_data->asrc_index = asrc_ssi_data.asrc_index;
		pcm_data->asrc_enable = 1;
	}
#endif

	snd_soc_dai_set_sysclk(codec_dai, ID95APM_SYSCLK, priv->sysclk, 0);
	snd_soc_dai_set_sysclk(codec_dai, ID95APM_LRCLK, rate, 0);

#if ID95APM_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

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

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (asrc_ssi_data.output_sample_rate != 0) {
			struct snd_soc_pcm_runtime *rtd =
			    substream->private_data;
			struct snd_soc_dai_link *pcm_link = rtd->dai;
			struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
			struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
			asrc_ssi_data.cpu_dai_rates = cpu_dai->playback.rates;
			asrc_ssi_data.codec_dai_rates =
			    codec_dai->playback.rates;
			cpu_dai->playback.rates =
			    SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT;
			codec_dai->playback.rates =
			    SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT;
		}
	}
#endif
	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_3stack_priv *priv = &card_priv;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (asrc_ssi_data.output_sample_rate != 0) {
			struct snd_soc_pcm_runtime *rtd =
			    substream->private_data;
			struct snd_soc_dai_link *pcm_link = rtd->dai;
			struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
			struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
			codec_dai->playback.rates =
			    asrc_ssi_data.codec_dai_rates;
			cpu_dai->playback.rates = asrc_ssi_data.cpu_dai_rates;
			asrc_release_pair(asrc_ssi_data.asrc_index);
		}
	}
#endif

	priv->hw = 0;
}

/*
 * imx_3stack ID95APM audio DAI opserations.
 */
static struct snd_soc_ops imx_3stack_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_audio_hw_params,
};

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* ID95APM uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if ID95APM_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Speaker Jack", NULL, "SPKRL" },
	{ "Speaker Jack", NULL, "SPKRR" },

	{ "Line Out Jack", NULL, "LOUTL" },
	{ "Line Out Jack", NULL, "LOUTR" },

	{ "HP Jack", NULL, "HPL" },
	{ "HP Jack", NULL, "HPR" },

	{ "MICL", NULL, "Mic In Jack" },
	{ "MICR", NULL, "Mic In Jack" },

	{ "LINL", NULL, "Line In Jack" },
	{ "LINR", NULL, "Line In Jack" },
};

static int id95apm_jack_func;
static int id95apm_spk_func;
static int id95apm_line_in_func;

static void headphone_detect_handler(struct work_struct *work)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int hp_status;
	char *envp[3];
	char *buf;

	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
	hp_status = plat->hp_status();

	/* setup a message for userspace headphone in */
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s kmalloc failed\n", __func__);
		return;
	}
	envp[0] = "NAME=headphone";
	snprintf(buf, 32, "STATE=%d", hp_status);
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	if (hp_status)
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_FALLING);
	else
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_RISING);
	enable_irq(plat->hp_irq);
}

static DECLARE_DELAYED_WORK(hp_event, headphone_detect_handler);

static irqreturn_t imx_headphone_detect_handler(int irq, void *data)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	u16 hp_status;

	if (plat->hp_status) {
		/* determine whether hp is plugged in */
		hp_status = plat->hp_status();

		if (hp_status == 0)
			strcpy(buf, "speaker\n");
		else
			strcpy(buf, "headphone\n");
	} else
		strcpy(buf, "unknown\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static const char *jack_function[] = { "off", "on"};

static const char *spk_function[] = { "off", "on" };

static const char *line_in_function[] = { "off", "on" };

static const struct soc_enum id95apm_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, line_in_function),
};

static int id95apm_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = id95apm_jack_func;
	return 0;
}

static int id95apm_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (id95apm_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	id95apm_jack_func = ucontrol->value.enumerated.item[0];
	if (id95apm_jack_func)
		snd_soc_dapm_enable_pin(codec, "HP Jack");
	else
		snd_soc_dapm_disable_pin(codec, "HP Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int id95apm_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = id95apm_spk_func;
	return 0;
}

static int id95apm_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (id95apm_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	id95apm_spk_func = ucontrol->value.enumerated.item[0];
	if (id95apm_spk_func)
		snd_soc_dapm_enable_pin(codec, "Speaker Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Speaker Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int id95apm_get_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = id95apm_line_in_func;
	return 0;
}

static int id95apm_set_line_in(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (id95apm_line_in_func == ucontrol->value.enumerated.item[0])
		return 0;

	id95apm_line_in_func = ucontrol->value.enumerated.item[0];
	if (id95apm_line_in_func)
		snd_soc_dapm_enable_pin(codec, "Line In Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Line In Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_HP("HP Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_MIC("Mic In Jack", NULL),
};

static const struct snd_kcontrol_new id95apm_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", id95apm_enum[0], id95apm_get_jack,
		     id95apm_set_jack),
	SOC_ENUM_EXT("Speaker Function", id95apm_enum[1], id95apm_get_spk,
		     id95apm_set_spk),
	SOC_ENUM_EXT("Line In Function", id95apm_enum[2], id95apm_get_line_in,
		     id95apm_set_line_in),
};

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
static int asrc_func;

static const char *asrc_function[] = {
	"disable", "8KHz", "11.025KHz", "16KHz", "22.05KHz", "32KHz", "44.1KHz", "48KHz", "88.2kHz", "96KHz" };

static const struct soc_enum asrc_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, asrc_function),
};

static int asrc_get_rate(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = asrc_func;
	return 0;
}

static int asrc_set_rate(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	if (asrc_func == ucontrol->value.enumerated.item[0])
		return 0;

	asrc_func = ucontrol->value.enumerated.item[0];
	asrc_ssi_data.output_sample_rate = id95apm_rates[asrc_func];

	return 1;
}

static const struct snd_kcontrol_new asrc_controls[] = {
	SOC_ENUM_EXT("ASRC", asrc_enum[0], asrc_get_rate,
		     asrc_set_rate),
};
#endif

static int imx_3stack_id95apm_init(struct snd_soc_codec *codec)
{
	int i, ret;

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_MXC_ASRC_MODULE)
	for (i = 0; i < ARRAY_SIZE(asrc_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&asrc_controls[i], codec, NULL));
		if (ret < 0)
			return ret;
	}
	asrc_ssi_data.output_sample_rate = id95apm_rates[asrc_func];
#endif

	/* Add imx_3stack specific controls */
	for (i = 0; i < ARRAY_SIZE(id95apm_machine_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&id95apm_machine_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(codec, "Line In Jack");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* imx_3stack digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "IP95APM",
	.stream_name = "ID95APM",
	.codec_dai = &id95apm_dai,
	.init = imx_3stack_id95apm_init,
	.ops = &imx_3stack_ops,
};

static int imx_3stack_card_remove(struct platform_device *pdev)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat;
	if (priv->pdev) {
		plat = priv->pdev->dev.platform_data;
		if (plat->finit)
			plat->finit();
	}

	return 0;
}

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.remove = imx_3stack_card_remove,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &id95apm_soc_codec_dev,
};

static int __devinit imx_3stack_id95apm_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;
	struct snd_soc_dai *id95apm_cpu_dai = 0;

	int ret = 0;

	priv->pdev = pdev;

	gpio_activate_audio_ports();
	imx_3stack_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		id95apm_cpu_dai = imx_ssi_dai[2];
	else if (plat->src_port == 1)
		id95apm_cpu_dai = imx_ssi_dai[0];
	else if (plat->src_port == 7)
		id95apm_cpu_dai = imx_ssi_dai[4];


	imx_3stack_dai.cpu_dai = id95apm_cpu_dai;

	/* get mxc_audio_platform_data for pcm */
	imx_3stack_dai.cpu_dai->dev = &pdev->dev;

	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	if (ret < 0) {
		pr_err("%s:failed to create driver_attr_headphone\n", __func__);
		goto sysfs_err;
	}

	ret = -EINVAL;
	if (plat->init && plat->init())
		goto err_plat_init;

	priv->sysclk = plat->sysclk;

	/* The ID95APM has an internal reset that is deasserted 8 SYS_MCLK
	   cycles after all power rails have been brought up. After this time
	   communication can start */

	if (plat->hp_status) {
		if (plat->hp_status())
			ret = request_irq(plat->hp_irq,
					  imx_headphone_detect_handler,
					  IRQ_TYPE_EDGE_FALLING, pdev->name, priv);
		else
			ret = request_irq(plat->hp_irq,
					  imx_headphone_detect_handler,
					  IRQ_TYPE_EDGE_RISING, pdev->name, priv);
		if (ret < 0) {
			pr_err("%s: request irq %d failed\n", __func__, plat->hp_irq);
			goto err_card_reg;
		}
	}

	id95apm_jack_func = 1;
	id95apm_spk_func = 1;
	id95apm_line_in_func = 0;

	return 0;

err_card_reg:
	if (plat->finit)
		plat->finit();
err_plat_init:
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
sysfs_err:
	return ret;
}

static int imx_3stack_id95apm_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;

	free_irq(plat->hp_irq, priv);

	if (plat->finit)
		plat->finit();

	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);

	return 0;
}

static struct platform_driver imx_3stack_id95apm_audio_driver = {
	.probe = imx_3stack_id95apm_probe,
	.remove = imx_3stack_id95apm_remove,
	.driver = {
		   .name = "imx-3stack-id95apm",
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_id95apm_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 4);
	if (!imx_3stack_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);

	if (ret)
		platform_device_put(imx_3stack_snd_device);

	return ret;
}

static void __exit imx_3stack_exit(void)
{
	platform_driver_unregister(&imx_3stack_id95apm_audio_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_init);
module_exit(imx_3stack_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ID95APM Driver for i.MX 3STACK");
MODULE_LICENSE("GPL");

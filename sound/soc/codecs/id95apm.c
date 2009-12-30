/*
 * id95apm.c  --  IDT 95APM ALSA Soc Audio driver
 *
 * Copyright (C) 2009 Integrated Device Technologies, Inc.
 * Copyright 2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/mfd/id95apm.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>

/*
 * ID95APM register cache & default register settings
 * Start with ID95APM_DAC0L_VOL (0x1a0)
 */
static const u8 id95apm_reg[ID95APM_REG_NUM] = {
	0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, /* 0x1a0 ... */
	0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85, 0x85,
	0x80, 0x80, 0x00, 0x00, 0x8f, 0x8f, 0x33, 0x00, /* 0x1b0 ... */
	0x80, 0x00, 0x32, 0x00, 0x22, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa0, /* 0x1c0 ... */
	0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x1d0 ... */
	0x00, 0x00, 0x00, 0x0f, 0x0f, 0x33, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, /* 0x1e0 ... */
	0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x1f0 ... */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x55, 0x02, 0x00, 0x00, 0x7f, 0x8f, 0x00, 0x00, /* 0x200 ... */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x210 ... */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x220 ... */
	0x00, 0x00,
};

static int id95apm_reg_valid(unsigned int reg)
{
	if (likely((reg >= ID95APM_AUDIO_REG_FIRST) &&
		   (reg <= ID95APM_AUDIO_REG_LAST)))
		return 1;

	return 0;
}

static unsigned int id95apm_read(struct snd_soc_codec *codec,
				 unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (id95apm_reg_valid(reg))
		return cache[reg - ID95APM_AUDIO_REG_FIRST];

	pr_err("%s: invalid codec register 0x%x (read)\n", codec->name, reg);

	return -1;
}

static int id95apm_write(struct snd_soc_codec *codec,
			 unsigned int reg, unsigned int value)
{
	struct id95apm *id95apm = (struct id95apm *)codec->private_data;
	u8 *cache = codec->reg_cache;
	int status = 0;

	if (id95apm_reg_valid(reg)) {
		if (cache[reg - ID95APM_AUDIO_REG_FIRST] != (u8)value) {
			id95apm_reg_write(id95apm, reg, (u8)value);
			cache[reg - ID95APM_AUDIO_REG_FIRST] = (u8)value;
		}
	} else {
		pr_err("%s: invalid codec register 0x%x (write)\n",
		       codec->name, reg);
		status = -EINVAL;
	}

	return status;
}

/*
 * DAC volume control:
 * from -95.25 to 0 dB in 0.75 dB steps (mute instead of -95.25 dB)
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, -9525, 75, 1);

/*
 * ADC gain control:
 * from 0 to 22.5 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(adc_tlv, 0, 150, 0);

/*
 * Class-D volume control:
 * from -91.5 to +36.0 dB in 0.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(classd_tlv, -9150, 50, 0);

static const char *id95apm_io_select[] = { "I2S0", "I2S1", "ADC0", "ADC1" };
static const char *id95apm_out_select[] = { "Mixer", "DAC0", "DAC1",
					    "Line In" };
static const char *mic_bias_level_txt[] = { "High-Z (Off)", "0.5*AVDD",
					    "0.9*AVDD", "GND" };

static const struct soc_enum mic_bias_level_l =
	SOC_ENUM_SINGLE(ID95APM_MIC_CTRL, 0, 4, mic_bias_level_txt);
static const struct soc_enum mic_bias_level_r =
	SOC_ENUM_SINGLE(ID95APM_MIC_CTRL, 2, 4, mic_bias_level_txt);

static struct soc_enum id95apm_enum[] = {
	SOC_ENUM_SINGLE(ID95APM_I2S0_SOURCE, 0, 4, id95apm_io_select),
	SOC_ENUM_SINGLE(ID95APM_I2S1_SOURCE, 0, 4, id95apm_io_select),
	SOC_ENUM_SINGLE(ID95APM_DAC0_SOURCE, 0, 4, id95apm_io_select),
	SOC_ENUM_SINGLE(ID95APM_DAC1_SOURCE, 0, 4, id95apm_io_select),
	SOC_ENUM_DOUBLE(ID95APM_CLASSD_SOURCE, 2, 0, 4, id95apm_out_select),
	SOC_ENUM_DOUBLE(ID95APM_LINE_OUT_CTRL, 2, 0, 4, id95apm_out_select),
	SOC_ENUM_DOUBLE(ID95APM_HP_SCTRL, 2, 0, 4, id95apm_out_select),
};

static const struct snd_kcontrol_new id95apm_snd_controls[] = {
	SOC_ENUM("Mic Bias Level Left", mic_bias_level_l),
	SOC_ENUM("Mic Bias Level Right", mic_bias_level_r),

	SOC_DOUBLE_R_TLV("DAC0 Volume", ID95APM_DAC0L_VOL, ID95APM_DAC0R_VOL,
			 0, 0x7f, 1, dac_tlv),
	SOC_DOUBLE_R_TLV("DAC1 Volume", ID95APM_DAC1L_VOL, ID95APM_DAC1R_VOL,
			 0, 0x7f, 1, dac_tlv),
	SOC_DOUBLE_R("DAC0 Switch", ID95APM_DAC0L_VOL, ID95APM_DAC0R_VOL,
		     7, 1, 1),
	SOC_DOUBLE_R("DAC1 Switch", ID95APM_DAC1L_VOL, ID95APM_DAC1R_VOL,
		     7, 1, 1),

	SOC_DOUBLE_R_TLV("ADC0 Gain", ID95APM_ADC0L_IN_AGAIN,
			 ID95APM_ADC0R_IN_AGAIN, 0, 0x0f, 0, adc_tlv),
	SOC_DOUBLE_R_TLV("ADC1 Gain", ID95APM_ADC1L_IN_AGAIN,
			 ID95APM_ADC1R_IN_AGAIN, 0, 0x0f, 1, adc_tlv),
	SOC_DOUBLE_R("ADC0 Switch", ID95APM_ADC0L_IN_AGAIN,
		     ID95APM_ADC0R_IN_AGAIN, 7, 1, 1),
	SOC_DOUBLE_R("ADC1 Switch", ID95APM_ADC1L_IN_AGAIN,
		     ID95APM_ADC1R_IN_AGAIN, 7, 1, 1),

	SOC_DOUBLE("Mic Boost Gain", ID95APM_MIC_ABOOST, 4, 0, 3, 0),

	SOC_DOUBLE_R_TLV("ClassD Volume", ID95APM_CLASSD_VOLUME0,
			 ID95APM_CLASSD_VOLUME1, 0, 0xff, 1, classd_tlv),

	/* Input mux */
	SOC_ENUM("I2S0 Input", id95apm_enum[0]),
	SOC_ENUM("I2S1 Input", id95apm_enum[1]),
	SOC_ENUM("DAC0 Input", id95apm_enum[2]),
	SOC_ENUM("DAC1 Input", id95apm_enum[3]),
};

/* Add non-DAPM controls */
static int id95apm_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(id95apm_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&id95apm_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

static const struct snd_kcontrol_new id95apm_snd_out_selectors[] = {
		SOC_DAPM_ENUM("ClassD Select", id95apm_enum[4]),
		SOC_DAPM_ENUM("Line Out Select", id95apm_enum[5]),
		SOC_DAPM_ENUM("Headphone Select", id95apm_enum[6]),
};

static const struct snd_soc_dapm_widget id95apm_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC0L", "Playback", ID95APM_PWR_CTRL2, 0, 1),
	SND_SOC_DAPM_DAC("DAC0R", "Playback", ID95APM_PWR_CTRL2, 1, 1),
	SND_SOC_DAPM_DAC("DAC1L", "Playback", ID95APM_PWR_CTRL2, 2, 1),
	SND_SOC_DAPM_DAC("DAC1R", "Playback", ID95APM_PWR_CTRL2, 3, 1),

	SND_SOC_DAPM_ADC("ADC0L", "Capture", ID95APM_PWR_CTRL2, 4, 1),
	SND_SOC_DAPM_ADC("ADC0R", "Capture", ID95APM_PWR_CTRL2, 5, 1),
	SND_SOC_DAPM_ADC("ADC1L", "Capture", ID95APM_PWR_CTRL2, 6, 1),
	SND_SOC_DAPM_ADC("ADC1R", "Capture", ID95APM_PWR_CTRL2, 7, 1),

	SND_SOC_DAPM_MIXER("HPL PowerOn", ID95APM_PWR_CTRL3, 3, 1, NULL, 0),
	SND_SOC_DAPM_MIXER("HPR PowerOn", ID95APM_PWR_CTRL3, 2, 1, NULL, 0),
	SND_SOC_DAPM_MIXER("LOUTL PowerOn", ID95APM_PWR_CTRL3, 5, 1, NULL, 0),
	SND_SOC_DAPM_MIXER("LOUTR PowerOn", ID95APM_PWR_CTRL3, 4, 1, NULL, 0),

	SND_SOC_DAPM_PGA("HP Enable", ID95APM_HP_SCTRL, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("LOUT Enable", ID95APM_LINE_OUT_CTRL, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("SPKRL Enable", ID95APM_CLASSD_SOURCE, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("SPKRR Enable", ID95APM_CLASSD_SOURCE, 6, 1, NULL, 0),

	SND_SOC_DAPM_MUX("SPKR MUX", SND_SOC_NOPM, 0, 0,
			 &id95apm_snd_out_selectors[0]),
	SND_SOC_DAPM_MUX("LOUT MUX", SND_SOC_NOPM, 0, 0,
			&id95apm_snd_out_selectors[1]),
	SND_SOC_DAPM_MUX("HP MUX", SND_SOC_NOPM, 0, 0,
			&id95apm_snd_out_selectors[2]),

	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("LOUTL"),
	SND_SOC_DAPM_OUTPUT("LOUTR"),
	SND_SOC_DAPM_OUTPUT("SPKRL"),
	SND_SOC_DAPM_OUTPUT("SPKRR"),

	SND_SOC_DAPM_INPUT("LINL"),
	SND_SOC_DAPM_INPUT("LINR"),
	SND_SOC_DAPM_INPUT("MICL"),
	SND_SOC_DAPM_INPUT("MICR"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{ "HPL", NULL, "HP Enable" },
	{ "HPR", NULL, "HP Enable" },
	{ "HP Enable", NULL, "HPL PowerOn" },
	{ "HP Enable", NULL, "HPR PowerOn" },
	{ "HPL PowerOn", NULL, "HP MUX" },
	{ "HPR PowerOn", NULL, "HP MUX" },
	{ "HP MUX", "DAC0", "DAC0L" },
	{ "HP MUX", "DAC0", "DAC0R" },
	{ "HP MUX", "DAC1", "DAC1L" },
	{ "HP MUX", "DAC1", "DAC1R" },
	{ "HP MUX", "Line In", "LINL" },
	{ "HP MUX", "Line In", "LINR" },

	{ "LOUTL", NULL, "LOUT Enable" },
	{ "LOUTR", NULL, "LOUT Enable" },
	{ "LOUT Enable", NULL, "LOUTL PowerOn" },
	{ "LOUT Enable", NULL, "LOUTR PowerOn" },
	{ "LOUTL PowerOn", NULL, "LOUT MUX" },
	{ "LOUTR PowerOn", NULL, "LOUT MUX" },
	{ "LOUT MUX", "DAC0", "DAC0L" },
	{ "LOUT MUX", "DAC0", "DAC0R" },
	{ "LOUT MUX", "DAC1", "DAC1L" },
	{ "LOUT MUX", "DAC1", "DAC1R" },
	{ "LOUT MUX", "Line In", "LINL" },
	{ "LOUT MUX", "Line In", "LINR" },

	{ "SPKRL", NULL, "SPKRL Enable" },
	{ "SPKRR", NULL, "SPKRR Enable" },
	{ "SPKRL Enable", NULL, "SPKR MUX" },
	{ "SPKRR Enable", NULL, "SPKR MUX" },
	{ "SPKR MUX", "DAC0", "DAC0L" },
	{ "SPKR MUX", "DAC0", "DAC0R" },
	{ "SPKR MUX", "DAC1", "DAC1L" },
	{ "SPKR MUX", "DAC1", "DAC1R" },
	{ "SPKR MUX", "Line In", "LINL" },
	{ "SPKR MUX", "Line In", "LINR" },

	{ "ADC0L", NULL, "LINL" },
	{ "ADC0R", NULL, "LINR" },

	{ "ADC1L", NULL, "MICL" },
	{ "ADC1R", NULL, "MICR" },
};

static int id95apm_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, id95apm_dapm_widgets,
				  ARRAY_SIZE(id95apm_dapm_widgets));
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_new_widgets(codec);

	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int id95apm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u8 i2s_cfg = 0;

	pr_debug("%s\n", __func__);

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		i2s_cfg |= 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s_cfg |= 2;
		break;
	default:
		return -EINVAL;
	}

	/* sample rate */
	switch (params_rate(params)) {
	case 8000:
		i2s_cfg |= (5 << 2); /* :6 */
		break;
	case 11025:
		i2s_cfg |= 0x80 | (3 << 2); /* :4 */
		break;
	case 16000:
		i2s_cfg |= (2 << 2); /* :3 */
		break;
	case 22050:
		i2s_cfg |= 0x80 | (1 << 2); /* :2 */
		break;
	case 32000:
		i2s_cfg = (i2s_cfg | 0x20) | (2 << 2); /* *2:3 */
		break;
	case 44100:
		i2s_cfg |= 0x80;		/* 44.1k base rate */
		break;
	case 48000:
		/* 48k base rate */
		break;
	case 88200:
		i2s_cfg |= 0x80 | 0x20; /* *2 */
		break;
	case 96000:
		i2s_cfg |= 0x20; /* *2 */
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Write to both I2S controller, so both are usable
	 */
	id95apm_write(codec, ID95APM_I2S0_CFG1, i2s_cfg);
	id95apm_write(codec, ID95APM_I2S1_CFG1, i2s_cfg);

	return 0;
}

static int id95apm_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 i2s_cfg;

	pr_debug("%s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		/* ID95APM works only in SLAVE mode */
	case SND_SOC_DAIFMT_CBS_CFS:
		break;	/* Nothing to do, already setup */
	default:
		pr_err("%s: can't support any non-slave mode (%d)\n",
		       codec->name, fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	i2s_cfg = id95apm_read(codec, ID95APM_I2S0_CFG2);

	i2s_cfg &= ~3;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2s_cfg |= 1;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2s_cfg |= 2;
		break;
	default:
		pr_err("%s: DAI format (%d) not supported\n",
		       codec->name, fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	i2s_cfg &= ~0x30;
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		i2s_cfg |= 0x30;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		i2s_cfg |= 0x20;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		i2s_cfg |= 0x10;
		break;
	default:
		return -EINVAL;
	}

	/* Enable tx & rx */
	i2s_cfg |= 0x84;

	/*
	 * Write to both I2S controller, so both are usable
	 */
	id95apm_write(codec, ID95APM_I2S0_CFG2, i2s_cfg);
	id95apm_write(codec, ID95APM_I2S1_CFG2, i2s_cfg);

	return 0;
}

static int id95apm_set_clkdiv(struct snd_soc_dai *codec_dai,
			      int div_id, int div)
{
	/*
	 * Nothing in here for now.
	 */

	return 0;
}

static int id95apm_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct id95apm *id95apm = (struct id95apm *)codec->private_data;
	u8 *cache = codec->reg_cache;

	switch (level) {
	case SND_SOC_BIAS_ON:
		/* Restore MIC bias level from register cache */
		id95apm_reg_write(id95apm, ID95APM_MIC_CTRL,
				  cache[ID95APM_MIC_CTRL -
					ID95APM_AUDIO_REG_FIRST]);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		/* Turn off MIC bias, bypassing cache */
		id95apm_reg_write(id95apm, ID95APM_MIC_CTRL, 0x00);
		break;
	}

	codec->bias_level = level;

	return 0;
}

#define ID95IPM_RATES	 (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_88200 | \
			  SNDRV_PCM_RATE_96000)
#define ID95APM_FORMATS	 (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			  SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai id95apm_dai = {
	.name = "ID95APM",
	.id = 0,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ID95IPM_RATES,
		.formats = ID95APM_FORMATS,
	},
	.capture = {
		 .stream_name = "Capture",
		 .channels_min = 2,
		 .channels_max = 2,
		 .rates = ID95IPM_RATES,
		 .formats = ID95APM_FORMATS,},
	.ops = {
		 .hw_params = id95apm_hw_params,
	 },
	.dai_ops = {
		 .set_fmt = id95apm_set_dai_fmt,
		 .set_clkdiv = id95apm_set_clkdiv,
	 },
};
EXPORT_SYMBOL_GPL(id95apm_dai);

/*
 * Enable Audio and CLASSD modules in ID95APM chip
 *
 * Register accessed here are not from the ID95APM audio
 * part, but from other ID95APM subsystem.
 */
int id95apm_enable_audio(struct id95apm *id95apm)
{
	pr_debug("%s\n", __func__);

	/* Enable 48 MHz, SSC for DCDC, S1 */
	id95apm_reg_write(id95apm, ID95APM_PCON_PLL_CFG, 0xc2);

	/* use internal 48MHz as MCLK */
	id95apm_reg_write(id95apm, ID95APM_PCON_MCLK_RATE, 0x00);

	/* Enable LDO 1.8V for Audio */
	id95apm_reg_write(id95apm, ID95APM_LDO_AUDIO_18, 0x80);
	/* Enable LDO 3.3V for Audio */
	id95apm_reg_write(id95apm, ID95APM_LDO_AUDIO_33, 0x66 | 0x80);
	/* Enable DCDC for Audio */
	id95apm_clrset_bits(id95apm, ID95APM_GLOB_DCDC_ENABLE, 0x80, 0x80);
	/* ENABLE_CLSD25_0 (?) */
	id95apm_reg_write(id95apm, ID95APM_DCDC_CLASS_D_EN0, 0x85);
	/* SC_DISABLE_CLSD25_0 for CLASSD BTL (?) */
	id95apm_reg_write(id95apm, ID95APM_DCDC_CLASS_D_CFG0, 0x40);
	/* ENABLE_CLSD25_1 (?) */
	id95apm_reg_write(id95apm, ID95APM_DCDC_CLASS_D_EN1, 0x85);
	/* SC_DISABLE_CLSD25_1 for CLASSD BTL (?) */
	id95apm_reg_write(id95apm, ID95APM_DCDC_CLASS_D_CFG1, 0x40);

	/* enable audio */
	id95apm_reg_write(id95apm, ID95APM_PCON_AUDIO_CTRL, 0x02);
	/* reset audio */
	id95apm_reg_write(id95apm, ID95APM_PCON_AUDIO_CTRL, 0x03);

	return 0;
}

/*
 * Disable Audio and CLASSD modules in ID95APM chip
 *
 * Register accessed here are not from the ID95APM audio
 * part, but from other ID95APM subsystem.
 */
int id95apm_disable_audio(struct id95apm *id95apm)
{
	pr_debug("%s\n", __func__);

	/* Disable Audio (power down) */
	id95apm_reg_write(id95apm, ID95APM_PCON_AUDIO_CTRL, 0);
	/* Disable DCDC for Audio */
	id95apm_clrset_bits(id95apm, ID95APM_GLOB_DCDC_ENABLE, 0x80, 0);
	/* Disable LDO 1.8V for Audio */
	id95apm_reg_write(id95apm, ID95APM_LDO_AUDIO_18, 0);
	/* Disable LDO 3.3V for Audio */
	id95apm_reg_write(id95apm, ID95APM_LDO_AUDIO_33, 0x66);

	return 0;
}

/*
 * Initialize the ID95APM audio device
 * register the mixer and dsp interfaces with the kernel
 */
static int id95apm_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	struct id95apm *id95apm = (struct id95apm *)codec->private_data;
	int status;

	pr_debug("%s\n", __func__);

	codec->name = "ID95APM";
	codec->owner = THIS_MODULE;
	codec->read = id95apm_read;
	codec->write = id95apm_write;
	codec->dai = &id95apm_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(id95apm_reg);
	codec->reg_cache = kmemdup(id95apm_reg, sizeof(id95apm_reg),
				   GFP_KERNEL);
	codec->bias_level = SND_SOC_BIAS_OFF;

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	status = id95apm_enable_audio(id95apm);
	if (status < 0) {
		pr_err("%s: failed to enable AUDIO module\n", codec->name);
		goto hw_err;
	}

	/*
	 * Make initial settings (routing etc)
	 */

	id95apm_write(codec, ID95APM_PWR_CTRL2, 0xff); /* power down */
	/* ClassD stays on */
	id95apm_write(codec, ID95APM_PWR_CTRL3, 0x3c); /* power down */

	/* Mute all outputs and let DAPM control them */
	id95apm_write(codec, ID95APM_HP_SCTRL, 0x15);
	id95apm_write(codec, ID95APM_LINE_OUT_CTRL, 0x15);
	id95apm_write(codec, ID95APM_CLASSD_SOURCE, 0xc5);

	/* Start with I2S0 as DAC0 source */
	id95apm_write(codec, ID95APM_DAC0_SOURCE, 0x00);

	/* Mic setup */
	id95apm_write(codec, ID95APM_MIC_MODE, 0x00); /* no power down */
	id95apm_write(codec, ID95APM_MIC_ABOOST, 0x22); /* boost 20dB gain */

	/* ADC1 (Mic) as source to I2S0 -> recording from Mic */
	id95apm_write(codec, ID95APM_I2S0_SOURCE, 0x03);

	/* Set medium volume to start with */
	id95apm_write(codec, ID95APM_DAC0L_VOL, 0x18);
	id95apm_write(codec, ID95APM_DAC0R_VOL, 0x18);

	/* No mute and 0dB gain on ADC's */
	id95apm_write(codec, ID95APM_ADC0L_IN_AGAIN, 0x00);
	id95apm_write(codec, ID95APM_ADC0R_IN_AGAIN, 0x00);
	id95apm_write(codec, ID95APM_ADC1L_IN_AGAIN, 0x0f);
	id95apm_write(codec, ID95APM_ADC1R_IN_AGAIN, 0x0f);

	/* Class-D stuff */
	id95apm_write(codec, ID95APM_CLASSD_MUTE, 0x03);
	id95apm_write(codec, ID95APM_CLASSD_CONFIG0, 0x20);
	id95apm_write(codec, ID95APM_CLASSD_PWM2, 0x61);
	id95apm_write(codec, ID95APM_CLASSD_ATTEN, 0x00);
	id95apm_write(codec, ID95APM_CLASSD_VOLUME0, 0x48 - 10);
	id95apm_write(codec, ID95APM_CLASSD_VOLUME1, 0x48 - 10);
	id95apm_write(codec, ID95APM_CLASSD_MUTE, 0x00);

	status = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1,
				  SNDRV_DEFAULT_STR1);
	if (status < 0) {
		pr_err("%s: failed to create pcms\n", codec->name);
		goto pcm_err;
	}

	id95apm_add_controls(codec);
	id95apm_add_widgets(codec);

	status = snd_soc_register_card(socdev);
	if (status < 0) {
		pr_err("%s: failed to register card\n", codec->name);
		goto card_err;
	}
	return status;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
hw_err:
	return status;
}

static int id95apm_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int status = 0;
	struct id95apm *id95apm = id95apm_get_id95apm();

	if (id95apm == NULL) {
		dev_err(&pdev->dev, "ID95APM MFD core driver not loaded\n");
		return -ENODEV;
	}

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->private_data = id95apm;

	status = id95apm_init(socdev);
	if (status < 0)
		goto err_free;

	dev_info(&pdev->dev, "ID95APM Audio device found\n");

	return status;

err_free:
	kfree(codec);
	return status;
}

static int id95apm_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct id95apm *id95apm = (struct id95apm *)codec->private_data;

	dev_info(&pdev->dev, "ID95APM Audio device removed\n");

	id95apm_set_bias_level(codec, SND_SOC_BIAS_OFF);

	id95apm_disable_audio(id95apm);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

static int id95apm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	id95apm_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int id95apm_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	id95apm_set_bias_level(codec, SND_SOC_BIAS_ON);
	return 0;
}

struct snd_soc_codec_device id95apm_soc_codec_dev = {
	.probe = id95apm_codec_probe,
	.remove = id95apm_codec_remove,
	.suspend = id95apm_suspend,
	.resume = id95apm_resume,
};
EXPORT_SYMBOL_GPL(id95apm_soc_codec_dev);

MODULE_DESCRIPTION("ASoC ID95APM driver");
MODULE_AUTHOR("Vitaliy Kulikov <Vitaliy.Kulikov@idt.com>");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");

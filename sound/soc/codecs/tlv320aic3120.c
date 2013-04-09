/*
 * ALSA SoC TLV320AIC3120 codec driver
 *
 * Author:      South Pole
 * Copyright:   (C) 2013 South Pole
 *
 * Based on sound/soc/codecs/tlv320aic3x.c by Vladimir Barinov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/regmap.h>

#include "tlv320aic3120.h"

#define AIC3120_NUM_SUPPLIES	5
static const char *aic3120_supply_names[AIC3120_NUM_SUPPLIES] = {
	"AVDD",         /* Analog power supply */
	"DVDD",         /* Digital power - digital core */
	"HPVDD",        /* Headphone/line dirver and PLL power */
	"IOVDD",        /* Interface power */
	"SPKVDD",       /* Class-D speaker driver power supply */
};


struct aic3120_priv;

/* codec private data */
struct aic3120_priv {
	struct snd_soc_codec *codec;
	struct regulator_bulk_data supplies[AIC3120_NUM_SUPPLIES];
	unsigned int sysclk;
	int master;
	int power;
	int clk_id;
};

/* DAC mute*/
static const char *const dac_mute[] = {"Unmute", "Mute"};

/* DAC power */
static const char *const dac_pwr[] = {"Off", "On"};

/* Data Path of DAC */
static const char *const dac_path[] = {"Off", "Left", "Right", "(L + R)/2" };


/* DAC volume range */
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);

/* HeadPhone volume range */
static const DECLARE_TLV_DB_SCALE(hp_vol_tlv, -7830, 60, 0);

/* Speaker volume range */
static const DECLARE_TLV_DB_SCALE(sp_vol_tlv, -7830, 60, 0);

static const struct soc_enum aic3120_enum[] = {
	SOC_ENUM_SINGLE(AIC3120_DAC_DATA_PATH_SETUP, 7, 2, dac_pwr),
	SOC_ENUM_SINGLE(AIC3120_DAC_DATA_PATH_SETUP, 4, 4, dac_path),
	SOC_ENUM_SINGLE(AIC3120_DAC_VOLUME_CONTROL_MUTED, 3, 2, dac_mute),
};


#define AIC3120_SOC_SINGLE_TLV(xname, xreg, xshift, xmin, xmax, tlv_array)	\
{										\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,					\
	.name = (xname),							\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |				\
			SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
	.tlv.p = (tlv_array),							\
	.info = dac_info_volsw,							\
	.get = dac_get_volsw,							\
	.put = dac_put_volsw,							\
	.private_value = (unsigned long)&(struct soc_mixer_control)		\
			{							\
				.reg = xreg, .rreg = xreg,			\
				.shift = xshift,				\
				.rshift = xshift,				\
				.min = xmin, .max = xmax,			\
				.platform_max = xmax				\
			}							\
}




/* custom function to fetch info of DAC playback volume */
static int dac_info_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x130 - 0x81;
	return 0;
}


/*
 * custom function to get DAC playback volume
  *
  * volume with 0.5 dB steps from 24 to -63.5 dB
  *
  * Register 65 (0x41): "DAC Volume Control" is a 8-bit register
  * from bit 0 to bit 7
  * valid values for this register:
  * 0x00 -- 0x30 and 0x81 -- 0xff
  * added a virtual bit 8 to make our life easy.
  * 0x130 = 24 db
  * 0x100 = 0  dB
  * 0xff = -0.5 dB
  * 0x81 = -63.5 dB
  *
  * register value       0x(1)30(24dB)         0x81(-63.5dB)
  *                      ------------------------------
  * userspace value      175                         0
  */
static int dac_get_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	u8 reg;
	int value;
	int min = mc->min;

	reg = snd_soc_read(codec, AIC3120_DAC_VOLUME_CONTROL);
	if((reg >= 0) && (reg <= 0x30))
		value = reg | (1 << 8);
	else
		value = reg;

	value -= min;
	ucontrol->value.integer.value[0] = value;
	return 0;
}


/*
 * custom function to set DAC playback volume
 *
 * PCM volume with 0.5 dB steps from 24 to -63.5 dB
 *
 * Register 65 (0x41): DAC Volume Control is a 8-bit register
 * from bit 0 to bit 7
 * valid values for this register:
 * 0x00 -- 0x30 and 0x81 -- 0xff
 * added a virtual bit 8 to make our life easy.
 * 0x130 = 24 db
 * 0x100 = 0  dB
 * 0xff = -0.5 dB
 * 0x81 = -63.5 dB
 *
 *
 * userspace value      0x175                         0
 *                      ------------------------------
 * register value       0x(1)30(24dB)         0x81(-63.5dB)
 */
static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	u8 reg;
	int value;
	int min = mc->min;
	int max = mc->max;

	value = ucontrol->value.integer.value[0];
	if(value > max)
		value = max;
	reg = (value + min) & 0xff;
	snd_soc_write(codec, AIC3120_DAC_VOLUME_CONTROL, reg);

	return 0;
}


static const struct snd_kcontrol_new aic3120_snd_controls[] = {
	AIC3120_SOC_SINGLE_TLV("DAC Playback Volume", AIC3120_DAC_VOLUME_CONTROL,
				0, 0x81, 0x130, dac_vol_tlv),
	SOC_SINGLE_RANGE_TLV("HeadPhone Volume", AIC3120_ANALOG_VOLUME_HPOUT, 0,
				0x80, 0xff, 1, hp_vol_tlv),
	SOC_SINGLE_RANGE_TLV("Speaker Volume", AIC3120_ANALOG_VOLUME_CLASSD, 0,
				0x80, 0xff, 1, sp_vol_tlv),

	SOC_ENUM("DAC Power", aic3120_enum[0]),
	SOC_ENUM("DAC Data Path", aic3120_enum[1]),
	SOC_ENUM("DAC Mute", aic3120_enum[2]),
};

static int aic3120_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic3120_priv *aic3120 = snd_soc_codec_get_drvdata(codec);
	int codec_clk = 0, fsref, sysclk_k=0;
	u8 data, j, r, p, pll_p = 1, pll_r = 1, pll_j = 1;
	u16 d, pll_d = 1;
	u8 ndac, mdac, p_ndac = 1, p_mdac = 1;
	u16 dosr, p_dosr = 1;
	unsigned int closest = 0;
	int ret = 0;

	/* Clock must be set first */
	if (!aic3120->sysclk) {
		dev_err(codec->dev, "%s: set sysclk first!\n", __func__);
		return -EFAULT;
	}

	/* select data word length */
	data = snd_soc_read(codec,
		AIC3120_CODEC_INTERFACE1) & (~(0x3 << 4));
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x01 << 4);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x02 << 4);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x03 << 4);
		break;
	}

	snd_soc_write(codec, AIC3120_CODEC_INTERFACE1, data);

	/* Divided by 1000 to avoid the overflow */
	sysclk_k = aic3120->sysclk / 1000;

	/*
	 * please read TLV320AIC3120 datasheet, to find out how to
	 * set the following parameters.
	 */

	/* Use the internal PLL */
	if(aic3120->clk_id == 3){
		/* Select the P, R, J and D values*/
		for(p = 1; p <= 8; p++){
			for(r = 1; r <= 16; r++){
				for(j = 1; j <= 63; j++){
					int temp;
					if((sysclk_k / p < 512) ||
						(sysclk_k / p > 20000))
						continue;
					if((r * j < 4) || (r * j > 259))
						continue;
					temp = sysclk_k * j * r / p;
					if((temp < 80000) || (temp > 110000))
						continue;
					codec_clk = temp;
					pll_p = p;
					pll_r = r;
					pll_j = j;
					pll_d = 0;
					goto found_codec_clk;
				}
			}
		}
		for(p = 1; p <= 8; p++){
			for(j = 1; j <= 63; j++){
				for(d = 1; d <= 9999; d++){
					int temp , k;

					if((sysclk_k / p < 10000) ||
						(sysclk_k / p > 20000))
						continue;
					k = (j * 10000 + d);
					temp = (sysclk_k * k) / (p * 10000);
					if((temp < 80000) || (temp > 110000))
						continue;
					codec_clk = temp;
					pll_p = p;
					pll_r = 1;
					pll_j = j;
					pll_d = d;
					goto found_codec_clk;
				}
			}
		}

	} else {
		codec_clk = sysclk_k;
	}

	if(codec_clk == 0){
		ret = -EINVAL;
		goto out;
	}
found_codec_clk:
	fsref = params_rate(params);
	codec_clk *= 1000;
	for(ndac = 2; ndac <= 128; ndac++){
		for(mdac = 3; mdac <= 128; mdac++){
			for(dosr = 1; dosr <= 1024; dosr++){
				unsigned int temp;
				temp = codec_clk / (ndac * mdac * dosr);
				if(temp == fsref){
					p_ndac = ndac;
					p_mdac = mdac;
					p_dosr = dosr;
					goto found;
				}
				if (abs(fsref - temp) <
					abs(fsref - closest)) {
					p_ndac = ndac;
					p_mdac = mdac;
					p_dosr = dosr;
					closest = temp;
				}

			}
		}
	}
found:
	/* If internel PLL is used, set P, R, J and D values */
	if(aic3120->clk_id == 3){
		data = (1 << 7) | (pll_p << 4) | (pll_r << 0);
		snd_soc_write(codec, AIC3120_PLL_PR_VAL, data);
		snd_soc_write(codec, AIC3120_PLL_J_VAL, (pll_j & 0x3f));
		data = (pll_d >> 8 & 0x3f);
		snd_soc_write(codec, AIC3120_PLL_D_VAL_MSB, data);
		snd_soc_write(codec, AIC3120_PLL_D_VAL_LSB, (pll_d & 0xff));
	}

	snd_soc_write(codec, AIC3120_DAC_NDAC_VAL, (p_ndac & 0x7f) | 0x80);
	snd_soc_write(codec, AIC3120_DAC_MDAC_VAL, (p_mdac & 0x7f) | 0x80);
	data = (p_dosr >> 8) & 0x3;
	snd_soc_write(codec, AIC3120_DAC_DOSR_VAL_MSB, data);
	snd_soc_write(codec, AIC3120_DAC_DOSR_VAL_LSB, p_dosr & 0xff);

	data = snd_soc_read(codec, AIC3120_DAC_DATA_PATH_SETUP);
	/* If Data path is off, set Data Path to (L + R)/2 */
	if(((data >> 4) & 0x3) == 0){
		data |= (3 << 4);
		snd_soc_write(codec, AIC3120_DAC_DATA_PATH_SETUP, data);
	}
out:
	return ret;
}


static int aic3120_dac_mute(struct snd_soc_codec *codec, int mute)
{
	u8 mute_reg;

	mute_reg = snd_soc_read(codec, AIC3120_DAC_VOLUME_CONTROL_MUTED);
	mute_reg &= (~(1 << 3));

	if(mute){
		mute_reg |= (1 << 3);
	}

	snd_soc_write(codec, AIC3120_DAC_VOLUME_CONTROL_MUTED, mute_reg);
	return 0;
}

static int aic3120_mute(struct snd_soc_dai *dai, int mute)
{
	int ret;

	/* At the moment, we only support the DAC */
	ret = aic3120_dac_mute(dai->codec, mute);
	return ret;
}

static int aic3120_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3120_priv *aic3120 = snd_soc_codec_get_drvdata(codec);
	u8 reg = snd_soc_read(codec, AIC3120_PLL_PR_VAL) &  (~(1 << 7));

	/* Use the internal PLL */
	if(clk_id == 3){
		reg |= (1 << 7);
	}
	snd_soc_write(codec, AIC3120_PLL_PR_VAL, reg);

	/* set clock on MCLK or BCLK */
	snd_soc_update_bits(codec, AIC3120_CLOCK_GEN_MUXING, 0x0c,
				clk_id << 2);
	snd_soc_update_bits(codec, AIC3120_CLOCK_GEN_MUXING, 0x03,
				clk_id << 0);

	aic3120->sysclk = freq;
	aic3120->clk_id = clk_id;
	return 0;
}

static int aic3120_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3120_priv *aic3120 = snd_soc_codec_get_drvdata(codec);
	u8 cic_reg, cic_reg2;

	cic_reg = snd_soc_read(codec, AIC3120_CODEC_INTERFACE1);
	cic_reg2 = snd_soc_read(codec, AIC3120_CODEC_INTERFACE2);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			aic3120->master = 1;
			cic_reg |= (1 << 3) | (1 << 2);
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			aic3120->master = 0;
			cic_reg &= ~((1 << 3) | (1 << 2));
			break;
		default:
			return -EINVAL;
	}

	cic_reg &= 0x3f;

	/* Set PCM data format */
	switch (fmt & (SND_SOC_DAIFMT_FORMAT_MASK)) {
	case (SND_SOC_DAIFMT_I2S):
		break;
	case (SND_SOC_DAIFMT_DSP_A):
	case (SND_SOC_DAIFMT_DSP_B):
		cic_reg |= (0x1 << 6);
		break;
	case (SND_SOC_DAIFMT_RIGHT_J):
		cic_reg |= (0x02 << 6);
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		cic_reg |= (0x03 << 6);
		break;
	default:
		return -EINVAL;
	}

	cic_reg2 &= ~(0x1 << 2);
	 /* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		/* Normal BLCK */
		case SND_SOC_DAIFMT_NB_NF:
			break;
		/* Inverted BLCK */
		case SND_SOC_DAIFMT_IB_NF:
			cic_reg2 |= (0x1 << 2);
			break;
		default:
			return -EINVAL;
	}

	snd_soc_write(codec, AIC3120_CODEC_INTERFACE1, cic_reg);
	snd_soc_write(codec, AIC3120_CODEC_INTERFACE2, cic_reg2);

	return 0;
}

static int aic3120_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	dev_dbg(codec->dev, "aic3120 set bias lever: %d\n", level);
	if(level == codec->dapm.bias_level){
		dev_dbg(codec->dev, "aic3120: bias level does not change.\n");
		return 0;
	}
	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(codec->dev, "aic3120: SND_SOC_BIAS_ON:\n");
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "aic3120: SND_SOC_BIAS_PREPARE:\n");
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev, "aic3120: SND_SOC_BIAS_STANDBY:\n");
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(codec->dev, "aic3120: SND_SOC_BIAS_OFF:\n");
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

#define AIC3120_RATES	SNDRV_PCM_RATE_8000_192000
#define AIC3120_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops aic3120_dai_ops = {
	.hw_params	= aic3120_hw_params,
	.digital_mute	= aic3120_mute,
	.set_sysclk	= aic3120_set_dai_sysclk,
	.set_fmt	= aic3120_set_dai_fmt,
};

static struct snd_soc_dai_driver aic3120_dai = {
	.name = "tlv320aic3120-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3120_RATES,
		.formats = AIC3120_FORMATS,
	},
	.ops = &aic3120_dai_ops,
	.symmetric_rates = 1,
};

static int aic3120_suspend(struct snd_soc_codec *codec)
{
	aic3120_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int aic3120_resume(struct snd_soc_codec *codec)
{
	aic3120_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

/*
 * initialise the AIC3120 registers.
 */
static int aic3120_init(struct snd_soc_codec *codec)
{
	/* Soft reset*/
	snd_soc_write(codec, AIC3120_RESET, 0x01);
	mdelay(10);

	/* Configure MCLK as clock input */
	snd_soc_write(codec, AIC3120_CLOCK_GEN_MUXING, 0x00);

	snd_soc_write(codec, AIC3120_DAC_NDAC_VAL, 0x88);
	snd_soc_write(codec, AIC3120_DAC_MDAC_VAL, 0x82);
	snd_soc_write(codec, AIC3120_DAC_DOSR_VAL_MSB, 0x00);
	snd_soc_write(codec, AIC3120_DAC_DOSR_VAL_LSB, 0x80);

	/* mode is i2s, wordlength is 16, slave mode */
	snd_soc_write(codec, AIC3120_CODEC_INTERFACE1, 0x00);
	snd_soc_write(codec,
		AIC3120_DAC_PROCESSING_BLOCK_MINIDSP, 0x10);

	snd_soc_write(codec, AIC3120_DAC_Coefficient_RAM_Control, 0x04);

	snd_soc_write(codec, AIC3120_HEADPHONE_DRIVERS, 0x04);

	/* DAC is routed to the mixer amplifier */
	snd_soc_write(codec, AIC3120_DAC_OUTPUT_MIXER_ROUTING, 0x40);

	/* Unmute HP */
	snd_soc_write(codec, AIC3120_HPOUT_DRIVER, 0x06);

	/* Unmute Class-D */
	snd_soc_write(codec, AIC3120_CLASSD_OUTPUT_DRIVER, 0x1c);

	/* HPOUT oupt driver is powered up */
	snd_soc_write(codec, AIC3120_HEADPHONE_DRIVERS, 0x84);

	/* Class-D output driver is powered up */
	snd_soc_write(codec, AIC3120_CLASSD_SPEAKER_AMPLIFIER, 0x86);

	/* Enable HP output analog volume */
	snd_soc_write(codec, AIC3120_ANALOG_VOLUME_HPOUT, 0x92);

	/* Enable Class-D output analog volume */
	snd_soc_write(codec, AIC3120_ANALOG_VOLUME_CLASSD, 0x92);

	snd_soc_write(codec, AIC3120_BEEP_GENERATOR, 0x80);

	/* Powerup DAC (soft step enabled) */
	snd_soc_write(codec, AIC3120_DAC_DATA_PATH_SETUP, 0xb4);

	udelay(10);

	/* DAC gain */
	snd_soc_write(codec, AIC3120_DAC_VOLUME_CONTROL, 0x34);

	/* Unmute DAC */
	snd_soc_write(codec, AIC3120_DAC_VOLUME_CONTROL_MUTED, 0x04);

	return 0;
}

static int aic3120_probe(struct snd_soc_codec *codec)
{
	struct aic3120_priv *aic3120 = snd_soc_codec_get_drvdata(codec);
	int ret = 0, i;
	struct regmap_config config;

	aic3120->codec = codec;

	memset(&config, 0, sizeof(config));
	config.reg_bits = 8;
	config.val_bits = 8;

	codec->control_data = regmap_init_i2c(to_i2c_client(codec->dev),
					&config);
        if (IS_ERR(codec->control_data)) {
		ret = PTR_ERR(codec->control_data);
		dev_err(codec->dev, "regmap_init() failed: %d\n", ret);
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(aic3120->supplies); i++)
		aic3120->supplies[i].supply = aic3120_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(aic3120->supplies),
				 aic3120->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto out;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(aic3120->supplies),
				aic3120->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto out;
	}

	aic3120_init(codec);

	ret = snd_soc_add_codec_controls(codec, aic3120_snd_controls,
			     ARRAY_SIZE(aic3120_snd_controls));
	if(ret != 0) {
		dev_err(codec->dev, "Failed to add controls: %d\n", ret);
	}
out:
	return ret;
}

static int aic3120_remove(struct snd_soc_codec *codec)
{
	struct aic3120_priv *aic3120 = snd_soc_codec_get_drvdata(codec);

	aic3120_set_bias_level(codec, SND_SOC_BIAS_OFF);
	regulator_bulk_disable(ARRAY_SIZE(aic3120->supplies), aic3120->supplies);
	regulator_bulk_free(ARRAY_SIZE(aic3120->supplies), aic3120->supplies);

	return 0;
}

int aic3120_codec_write(struct snd_soc_codec *codec, unsigned int reg,
                        unsigned int value)
{
	unsigned int page = (reg >> 8) & 0x7f;
	unsigned int num = reg & 0xff;

	regmap_write(codec->control_data, AIC3120_PAGE_CONTROL, page);
	return regmap_write(codec->control_data, num, value & 0xff);
}

unsigned int aic3120_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int val;
	unsigned int page = (reg >> 8) & 0x7f;
	unsigned int num = reg  & 0xff;

	regmap_write(codec->control_data, AIC3120_PAGE_CONTROL, page);
	regmap_read(codec->control_data, num, &val);
	return val & 0xff;
}


static struct snd_soc_codec_driver soc_codec_dev_aic3120 = {
	.probe = aic3120_probe,
	.remove = aic3120_remove,
	.suspend = aic3120_suspend,
	.resume = aic3120_resume,
	.read = aic3120_codec_read,
	.write = aic3120_codec_write,
	.set_bias_level = aic3120_set_bias_level,
	.reg_word_size = sizeof(u8),
	.reg_cache_size = 0,
	.reg_cache_default = NULL,
};

static const struct i2c_device_id aic3120_i2c_id[] = {
	{ "tlv320aic3120", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic3120_i2c_id);


static int aic3120_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct aic3120_priv *aic3120;
	int ret;

	aic3120 = devm_kzalloc(&client->dev, sizeof(struct aic3120_priv), GFP_KERNEL);
	if (aic3120 == NULL) {
		dev_err(&client->dev, "failed to create private data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, aic3120);

	ret = snd_soc_register_codec(&client->dev,
			&soc_codec_dev_aic3120, &aic3120_dai, 1);
	return ret;
}

static int aic3120_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id tlv320aic3120_of_match[] = {
	{ .compatible = "ti,tlv320aic3120", },
	{},
};
MODULE_DEVICE_TABLE(of, tlv320aic3120_of_match);
#endif

/* machine i2c codec control layer */
static struct i2c_driver aic3120_i2c_driver = {
	.driver = {
		.name = "tlv320aic3120-codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tlv320aic3120_of_match),
	},
	.probe	= aic3120_i2c_probe,
	.remove = aic3120_i2c_remove,
	.id_table = aic3120_i2c_id,
};

module_i2c_driver(aic3120_i2c_driver);

MODULE_DESCRIPTION("ASoC TLV320AIC3120 codec driver");
MODULE_AUTHOR("South Pole");
MODULE_LICENSE("GPL");

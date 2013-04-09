/*
 * Copyright 2011 Freescale Semiconductor, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


/*
 * This file is based on the file mxs-sgtl5000.c, and modified by
 * South Pole AB for Tlv320aic3120 codec
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>

#include "../codecs/tlv320aic3120.h"
#include "mxs-saif.h"

static int mxs_tlv320aic3120_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int rate = params_rate(params);
	u32 dai_format, mclk;
	int ret;

	mclk = 512 * rate;

	/* SAIF MCLK */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, 0);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, MXS_SAIF_MCLK, mclk, 0);
	if (ret)
		return ret;

	/* set codec to slave mode */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret)
		return ret;

	return 0;
}

static struct snd_soc_ops mxs_tlv320aic3120_hifi_ops = {
	.hw_params = mxs_tlv320aic3120_hw_params,
};

static struct snd_soc_dai_link mxs_tlv320aic3120_dai[] = {
	{
		.name		= "HiFi Tx",
		.stream_name	= "HiFi Playback",
		.codec_dai_name	= "tlv320aic3120-hifi",
		.codec_name	= "tlv320aic3120.0-000a",
		.cpu_dai_name	= "mxs-saif.0",
		.platform_name	= "mxs-saif.0",
		.ops		= &mxs_tlv320aic3120_hifi_ops,
	},
};

static struct snd_soc_card mxs_tlv320aic3120 = {
	.name		= "mxs_tlv320aic3120",
	.owner		= THIS_MODULE,
	.dai_link	= mxs_tlv320aic3120_dai,
	.num_links	= ARRAY_SIZE(mxs_tlv320aic3120_dai),
};

static int mxs_tlv320aic3120_probe_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *saif_np[2], *codec_np;
	int ret = 0;

	if (!np)
		return 1; /* no device tree */

	saif_np[0] = of_parse_phandle(np, "saif-controllers", 0);
	saif_np[1] = of_parse_phandle(np, "saif-controllers", 1);
	codec_np = of_parse_phandle(np, "audio-codec", 0);

	if (!saif_np[0] || !saif_np[1] || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	mxs_tlv320aic3120_dai[0].codec_name = NULL;
	mxs_tlv320aic3120_dai[0].codec_of_node = codec_np;
	mxs_tlv320aic3120_dai[0].cpu_dai_name = NULL;
	mxs_tlv320aic3120_dai[0].cpu_of_node = saif_np[0];
	mxs_tlv320aic3120_dai[0].platform_name = NULL;
	mxs_tlv320aic3120_dai[0].platform_of_node = saif_np[0];

	of_node_put(codec_np);
	of_node_put(saif_np[0]);
	of_node_put(saif_np[1]);

	return ret;
}

static int mxs_tlv320aic3120_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mxs_tlv320aic3120;
	int ret;

	ret = mxs_tlv320aic3120_probe_dt(pdev);
	if (ret < 0)
		return ret;

	ret = mxs_saif_get_mclk(0, 44100 * 512, 44100);
	if (ret)
		return ret;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return ret;
	}
	return 0;
}

static int mxs_tlv320aic3120_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	mxs_saif_put_mclk(0);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id mxs_tlv320aic3120_dt_ids[] = {
	{ .compatible = "fsl,mxs-audio-tlv320aic3120", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_tlv320aic3120_dt_ids);

static struct platform_driver mxs_tlv320aic3120_audio_driver = {
	.driver = {
		.name = "mxs-tlv320aic3120",
		.owner = THIS_MODULE,
		.of_match_table = mxs_tlv320aic3120_dt_ids,
	},
	.probe = mxs_tlv320aic3120_probe,
	.remove = mxs_tlv320aic3120_remove,
};

module_platform_driver(mxs_tlv320aic3120_audio_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXS ALSA SoC Machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxs-tlv320aic3120");

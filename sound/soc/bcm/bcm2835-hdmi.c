/*
 * BCM2835 (Raspberry Pi) HDMI audio driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

/* Emitting HDMI audio on the BCM2835 (Raspberry Pi) involves DMAing
 * S/PDIF frames into the HD_MAI_DATA reg of the HDMI encoder while
 * the HDMI encoder has an audio infoframe and audio clocks running.
 *
 * ASoC splits the sound stream into a few parts:
 *
 * CPU DAI: The CPU DAI's job is to DMA the PCM data.  For us, the DMA
 * will be performed by the platform's generic DMA engine in
 * drivers/dma/bcm2835-dma.c, and this driver's job is to just tell
 * soc-generic-dmaengine-pcm.c where our register is.
 *
 * Codec DAI: This is what manages the format/rate/etc. of the
 * streams.  For us, this will be the generic hdmi-codec driver in
 * S/PDIF mode.
 *
 * Machine driver: This is what sets up the ALSA sound card and routes
 * between the DAIs.  For us, we use simple-audio-card DT binding that
 * references the CPU DAI (this device) and the hdmi-codec (the HDMI
 * encoder that the DRM binds);
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include <sound/bcm2835-hdmi.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* General device struct */
struct bcm2835_hdmi_audio {
	struct device *dev;
	struct snd_dmaengine_dai_dma_data dma_data;

	const struct bcm2835_hdmi_codec_ops *encoder_ops;
	void *encoder_data;
};

static int bcm2835_hdmi_set_dai_fmt(struct snd_soc_dai *dai,
				      unsigned int fmt)
{
	return 0;
}

static int bcm2835_hdmi_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	return 0;
}

static int bcm2835_hdmi_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static int bcm2835_hdmi_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	return 0;
}

static void bcm2835_hdmi_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
}

static const struct snd_soc_dai_ops bcm2835_hdmi_dai_ops = {
	.startup	= bcm2835_hdmi_startup,
	.shutdown	= bcm2835_hdmi_shutdown,
	.prepare	= bcm2835_hdmi_prepare,
	.hw_params	= bcm2835_hdmi_hw_params,
	.set_fmt	= bcm2835_hdmi_set_dai_fmt,
};

static int bcm2835_hdmi_dai_probe(struct snd_soc_dai *dai)
{
	struct bcm2835_hdmi_audio *hdmi = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &hdmi->dma_data, NULL);

	return 0;
}

static struct snd_soc_dai_driver bcm2835_hdmi_cpu_dai = {
	.name	= "bcm2835-hdmi-cpu-dai",
	.probe	= bcm2835_hdmi_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates =	SNDRV_PCM_RATE_8000_192000,
		.formats =	SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE
	},
	.ops = &bcm2835_hdmi_dai_ops,
	.symmetric_rates = 1,
	.id = 1, /* XXX */
};

static const struct snd_soc_component_driver bcm2835_hdmi_component = {
	.name = "bcm2835-hdmi-cpu-dai",
};

static int bcm2835_hdmi_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_hdmi_audio *hdmi;
	int ret;
	const __be32 *addr;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* Get the DMA address from the DT.  We parse ourselves,
	 * because we want the bus address, not a CPU physical
	 * address.
	 */
	addr = of_get_address(dev->of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(dev, "could not get DMA-register address\n");
		return -EINVAL;
	}

	hdmi->dma_data.addr = be32_to_cpup(addr);
	hdmi->dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	hdmi->dma_data.maxburst = 2;

	/* Store the pdev */
	hdmi->dev = dev;
	dev_set_drvdata(dev, hdmi);

	ret = devm_snd_soc_register_component(dev,
					      &bcm2835_hdmi_component,
					      &bcm2835_hdmi_cpu_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register CPU DAI: %d\n", ret);
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		return ret;
	}

	dev_info(dev, "PROBED\n");

	return 0;
}

static const struct of_device_id bcm2835_hdmi_audio_of_match[] = {
	{ .compatible = "brcm,bcm2835-hdmi-audio", },
	{},
};

static struct platform_driver bcm2835_hdmi_audio_driver = {
	.probe		= bcm2835_hdmi_audio_probe,
	.driver		= {
		.name	= BCM2835_HDMI_AUDIO_DRV_NAME,
		.of_match_table = bcm2835_hdmi_audio_of_match,
	},
};

module_platform_driver(bcm2835_hdmi_audio_driver);

MODULE_DESCRIPTION("BCM2835 HDMI audio interface");
MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_LICENSE("GPL v2");

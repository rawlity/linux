/*
 * hdmi-codec.h - HDMI Codec driver API
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Jyri Sarha <jsarha@ti.com>
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

#ifndef __BCM2835_HDMI_CODEC_H__
#define __BCM2835_HDMI_CODEC_H__

#include <linux/hdmi.h>
#include <drm/drm_edid.h>
#include <sound/asoundef.h>
#include <uapi/sound/asound.h>

/*
 * Protocol between ASoC cpu-dai and HDMI-encoder
 */
struct bcm2835_hdmi_codec_daifmt {
	int bit_clk_inv:1;
	int frame_clk_inv:1;
	int bit_clk_master:1;
	int frame_clk_master:1;
};

/*
 * HDMI audio parameters
 */
struct bcm2835_hdmi_codec_params {
	struct hdmi_audio_infoframe cea;
	struct snd_aes_iec958 iec;
	int sample_rate;
	int sample_width;
	int channels;
};

struct bcm2835_hdmi_audio_pdata;
struct bcm2835_hdmi_audio_ops {
	/*
	 * Called when ASoC starts an audio stream setup.
	 */
	int (*audio_startup)(struct device *dev, void *data);

	/*
	 * Configures HDMI-encoder for audio stream.
	 */
	int (*hw_params)(struct device *dev, void *data,
			 struct bcm2835_hdmi_codec_daifmt *fmt,
			 struct bcm2835_hdmi_codec_params *hparms);

	/*
	 * Shuts down the audio stream.
	 */
	void (*audio_shutdown)(struct device *dev, void *data);

	/*
	 * Mute/unmute HDMI audio stream.
	 */
	int (*digital_mute)(struct device *dev, void *data, bool enable);

	/*
	 * Provides EDID-Like-Data from connected HDMI device.
	 */
	int (*get_eld)(struct device *dev, void *data,
		       uint8_t *buf, size_t len);
};

/* HDMI codec initalization data */
struct bcm2835_hdmi_audio_pdata {
	const struct bcm2835_hdmi_codec_ops *ops;
	u32 dma_addr;
	void *data;
};

#define BCM2835_HDMI_AUDIO_DRV_NAME "bcm2835-hdmi-audio"

#endif /* __HDMI_CODEC_H__ */

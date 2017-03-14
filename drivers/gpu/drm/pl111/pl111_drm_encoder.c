/*
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

/**
 * pl111_drm_encoder.c
 * Implementation of the encoder functions for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include "pl111_drm.h"

static const struct drm_encoder_funcs pl111_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

int pl111_encoder_init(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct drm_encoder *encoder = &priv->encoder;
	int ret;

	ret = drm_encoder_init(dev, encoder, &pl111_encoder_funcs,
			       DRM_MODE_ENCODER_DAC, NULL);
	if (ret)
		return ret;

	encoder->crtc = &priv->pl111_crtc->crtc;
	encoder->possible_crtcs = BIT(drm_crtc_index(encoder->crtc));

	return 0;
}

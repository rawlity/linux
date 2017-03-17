/*
 *
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
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

#ifndef _PL111_DRM_H_
#define _PL111_DRM_H_

#include <drm/drm_gem.h>

#define CLCD_IRQ_NEXTBASE_UPDATE (1u<<2)

struct pl111_drm_connector {
	struct drm_connector connector;
	struct drm_panel *panel;
};

struct pl111_drm_dev_private {
	struct drm_device *drm;

	struct pl111_drm_connector connector;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_plane primary;
	struct drm_fbdev_cma *fbdev;

	void *regs;
	struct clk *clk;
};

#define to_pl111_connector(x) \
	container_of(x, struct pl111_drm_connector, connector)

/* CRTC Functions */
int pl111_crtc_create(struct drm_device *dev);
irqreturn_t pl111_irq(int irq, void *data);

int pl111_primary_plane_init(struct drm_device *dev);

/* Connector Functions */
int pl111_connector_create(struct drm_device *dev);

/* Encoder Functions */
int pl111_encoder_init(struct drm_device *dev);

/* GEM Functions */
int pl111_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev,
		      struct drm_mode_create_dumb *args);

#endif /* _PL111_DRM_H_ */

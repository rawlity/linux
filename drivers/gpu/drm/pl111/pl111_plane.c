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

#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "pl111_drm.h"

static int pl111_primary_plane_atomic_check(struct drm_plane *plane,
					    struct drm_plane_state *state)
{
	return 0;
}

static void pl111_primary_plane_atomic_update(struct drm_plane *plane,
					      struct drm_plane_state *old_state)
{
	struct drm_device *dev = plane->dev;
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct drm_framebuffer *fb = plane->state->fb;
	struct drm_gem_cma_object *obj = (fb ? drm_fb_cma_get_gem_obj(fb, 0) :
					  NULL);
	u32 addr;

	if (!fb)
		return;

	addr = obj->paddr + fb->offsets[0];
	addr += fb->format->cpp[0] * plane->state->src_x;
	addr += fb->pitches[0] * plane->state->src_y;

	writel(addr, priv->regs + CLCD_UBAS);
	writel(addr + (fb->height - 1 * fb->pitches[0]), priv->regs + CLCD_LBAS);
}

static const struct drm_plane_helper_funcs pl111_primary_plane_helper_funcs = {
	.atomic_check = pl111_primary_plane_atomic_check,
	.atomic_update = pl111_primary_plane_atomic_update,
};


static const struct drm_plane_funcs pl111_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int pl111_primary_plane_init(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct drm_plane *plane = &priv->primary;
	u32 formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_RGB565 };
	int ret;

	ret = drm_universal_plane_init(dev, plane, 0,
				       &pl111_primary_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &pl111_primary_plane_helper_funcs);

	return 0;
}



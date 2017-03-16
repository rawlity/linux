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
#include <linux/of_graph.h>
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
	struct drm_gem_cma_object *obj;
	u32 addr, cntl;

	if (!fb)
		return;

	obj = drm_fb_cma_get_gem_obj(fb, 0);
	addr = obj->paddr + fb->offsets[0];
	addr += fb->format->cpp[0] * plane->state->src_x;
	addr += fb->pitches[0] * plane->state->src_y;

	writel(addr, priv->regs + CLCD_UBAS);
	writel(addr + (fb->height - 1 * fb->pitches[0]), priv->regs + CLCD_LBAS);

	cntl = readl(priv->regs + CLCD_PL111_CNTL);
	cntl &= ~(7 << 1);

	/* Note that the the hardware's format reader takes 'r' from
	 * the low bit, while DRM formats list channels from high bit
	 * to low bit as you read left to right.
	 */
	switch (fb->format->format) {
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		cntl |= CNTL_LCDBPP24;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		cntl |= CNTL_LCDBPP24 | CNTL_BGR;
		break;
	case DRM_FORMAT_BGR565:
		cntl |= CNTL_LCDBPP16_565;
		break;
	case DRM_FORMAT_RGB565:
		cntl |= CNTL_LCDBPP16_565 | CNTL_BGR;
		break;
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		cntl |= CNTL_LCDBPP16;
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		cntl |= CNTL_LCDBPP16 | CNTL_BGR;
		break;
	case DRM_FORMAT_ABGR4444:
	case DRM_FORMAT_XBGR4444:
		cntl |= CNTL_LCDBPP16_444;
		break;
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_XRGB4444:
		cntl |= CNTL_LCDBPP16_444 | CNTL_BGR;
		break;
	}

	writel(cntl, priv->regs + CLCD_PL111_CNTL);
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

int pl111_primary_plane_init(struct drm_device *drm)
{
	struct pl111_drm_dev_private *priv = drm->dev_private;
	struct drm_plane *plane = &priv->primary;
	struct device *dev = drm->dev;
	static const u32 formats[] = {
		DRM_FORMAT_ABGR8888,
		DRM_FORMAT_XBGR8888,
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_BGR565,
		DRM_FORMAT_RGB565,
		DRM_FORMAT_ABGR1555,
		DRM_FORMAT_XBGR1555,
		DRM_FORMAT_ARGB1555,
		DRM_FORMAT_XRGB1555,
		DRM_FORMAT_ABGR4444,
		DRM_FORMAT_XBGR4444,
		DRM_FORMAT_ARGB4444,
		DRM_FORMAT_XRGB4444,
	};
	struct device_node *endpoint;
	u32 tft_r0b0g0[3];
	int ret;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint)
		return -ENODEV;

	if (of_property_read_u32_array(endpoint,
				       "arm,pl11x,tft-r0g0b0-pads",
				       tft_r0b0g0,
				       ARRAY_SIZE(tft_r0b0g0)) != 0) {
		dev_err(dev, "arm,pl11x,tft-r0g0b0-pads should be 3 ints\n");
		of_node_put(endpoint);
		return -ENOENT;
	}
	of_node_put(endpoint);

	if (tft_r0b0g0[0] != 0 ||
	    tft_r0b0g0[1] != 8 ||
	    tft_r0b0g0[2] != 16) {
		dev_err(dev, "arm,pl11x,tft-r0g0b0-pads != [0,8,16] not yet supported\n");
		return -EINVAL;
	}

	ret = drm_universal_plane_init(drm, plane, 0,
				       &pl111_primary_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &pl111_primary_plane_helper_funcs);

	return 0;
}



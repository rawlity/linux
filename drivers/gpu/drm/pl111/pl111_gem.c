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
 * pl111_drm_gem.c
 * Implementation of the GEM functions for PL111 DRM
 */
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include "pl111_drm.h"

int pl111_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	return drm_gem_cma_dumb_create_internal(file_priv, dev, args);
}

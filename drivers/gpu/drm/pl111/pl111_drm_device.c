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
 * pl111_drm_device.c
 * Implementation of the Linux device driver entrypoints for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "pl111_drm.h"

struct pl111_drm_dev_private priv;


void pl111_drm_preclose(struct drm_device *dev, struct drm_file *file_priv)
{
	DRM_DEBUG_KMS("DRM %s on dev=%p\n", __func__, dev);
}

void pl111_drm_lastclose(struct drm_device *dev)
{
	DRM_DEBUG_KMS("DRM %s on dev=%p\n", __func__, dev);
}

struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
};

static int pl111_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct pl111_drm_connector *pl111_connector;
	struct pl111_drm_encoder *pl111_encoder;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	drm_mode_config_init(dev);
	mode_config = &dev->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 1024;
	mode_config->min_height = 1;
	mode_config->max_height = 768;

	priv->pl111_crtc = pl111_crtc_create(dev);
	if (priv->pl111_crtc == NULL) {
		pr_err("Failed to create pl111_drm_crtc\n");
		ret = -ENOMEM;
		goto out_config;
	}

	priv->number_crtcs = 1;

	pl111_connector = pl111_connector_create(dev);
	if (pl111_connector == NULL) {
		pr_err("Failed to create pl111_drm_connector\n");
		ret = -ENOMEM;
		goto out_config;
	}

	pl111_encoder = pl111_encoder_create(dev, 1);
	if (pl111_encoder == NULL) {
		pr_err("Failed to create pl111_drm_encoder\n");
		ret = -ENOMEM;
		goto out_config;
	}

	ret = drm_mode_connector_attach_encoder(&pl111_connector->connector,
						&pl111_encoder->encoder);
	if (ret != 0) {
		DRM_ERROR("Failed to attach encoder\n");
		goto out_config;
	}

	pl111_connector->connector.encoder = &pl111_encoder->encoder;

	ret = pl111_cursor_plane_init(dev, &priv->pl111_crtc->cursor, 1);
	if (ret != 0) {
		pr_err("Failed to init cursor plane\n");
		goto out_config;
	}

	goto finish;

out_config:
	drm_mode_config_cleanup(dev);
finish:
	DRM_DEBUG("%s returned %d\n", __func__, ret);
	return ret;
}

static void pl111_modeset_fini(struct drm_device *dev)
{
	drm_mode_config_cleanup(dev);
}

static int pl111_drm_load(struct drm_device *dev, unsigned long chipset)
{
	int ret = 0;

	pr_info("DRM %s\n", __func__);

	mutex_init(&priv.export_dma_buf_lock);
	atomic_set(&priv.nr_flips_in_flight, 0);
	init_waitqueue_head(&priv.wait_for_flips);

	/* Create a cache for page flips */
	priv.page_flip_slab = kmem_cache_create("page flip slab",
			sizeof(struct pl111_drm_flip_resource), 0, 0, NULL);
	if (priv.page_flip_slab == NULL) {
		DRM_ERROR("Failed to create slab\n");
		ret = -ENOMEM;
		goto out_kds_callbacks;
	}

	dev->dev_private = &priv;

	ret = pl111_modeset_init(dev);
	if (ret != 0) {
		pr_err("Failed to init modeset\n");
		goto out_slab;
	}

	ret = pl111_device_init(dev);
	if (ret != 0) {
		DRM_ERROR("Failed to init MMIO and IRQ\n");
		goto out_modeset;
	}

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		DRM_ERROR("Failed to init vblank\n");
		goto out_vblank;
	}

	goto finish;

out_vblank:
	pl111_device_fini(dev);
out_modeset:
	pl111_modeset_fini(dev);
out_slab:
	kmem_cache_destroy(priv.page_flip_slab);
out_kds_callbacks:
finish:
	DRM_DEBUG_KMS("pl111_drm_load returned %d\n", ret);
	return ret;
}

static void pl111_drm_unload(struct drm_device *dev)
{
	pr_info("DRM %s\n", __func__);

	kmem_cache_destroy(priv.page_flip_slab);

	drm_vblank_cleanup(dev);
	pl111_modeset_fini(dev);
	pl111_device_fini(dev);
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.read = drm_read,
};

static struct drm_driver driver = {
	.driver_features =
		DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load = pl111_drm_load,
	.unload = pl111_drm_unload,
	.context_dtor = NULL,
	.preclose = pl111_drm_preclose,
	.lastclose = pl111_drm_lastclose,
	.ioctls = NULL,
	.fops = &drm_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
	.dumb_create = pl111_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = drm_gem_prime_export,
};

int pl111_drm_init(struct platform_device *dev)
{
	int ret;
	pr_info("DRM %s\n", __func__);
	pr_info("PL111 DRM initialize, driver name: %s, version %d.%d\n",
		DRIVER_NAME, DRIVER_MAJOR, DRIVER_MINOR);
	driver.num_ioctls = 0;
	ret = 0;
	return drm_platform_init(&driver, dev);

}

void pl111_drm_exit(struct platform_device *dev)
{
	pr_info("DRM %s\n", __func__);
	drm_platform_exit(&driver, dev);
}
